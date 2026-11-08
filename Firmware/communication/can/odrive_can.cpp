#include "odrive_can.hpp"

#include <can.h>
#include <cmsis_os.h>
#include <odrive_main.h>

#include "freertos_vars.h"
#include "utils.hpp"

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;

bool ODriveCAN::apply_config() {
    config_.parent = this;
    set_baud_rate(config_.baud_rate);
    return true;
}

bool ODriveCAN::reinit() {
    HAL_CAN_Stop(handle_);
    HAL_CAN_ResetError(handle_);
    return (HAL_CAN_Init(handle_) == HAL_OK) && (HAL_CAN_Start(handle_) == HAL_OK) && (HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) == HAL_OK);
}

bool ODriveCAN::start_server(CAN_HandleTypeDef* handle) {
    handle_ = handle;

    handle_->Init.Prescaler = CAN_FREQ / config_.baud_rate;
    if (!reinit()) {
        return false;
    }

    auto wrapper = [](void* ctx) {
        ((ODriveCAN*)ctx)->can_server_thread();
    };
    osThreadDef(can_server_thread_def, wrapper, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);

    return true;
}

void ODriveCAN::can_server_thread() {
    Protocol protocol = config_.protocol;

    if (protocol & PROTOCOL_SIMPLE) {
        can_simple_.init();
    }

    for (;;) {
        uint32_t status = HAL_CAN_GetError(handle_);
        if (status == HAL_CAN_ERROR_NONE) {
            uint32_t next_service_time = UINT32_MAX;

            if (protocol & PROTOCOL_SIMPLE) {
                next_service_time = std::min(can_simple_.service_stack(), next_service_time);
            }

            next_service_time = std::min(
                service_bike_telemetry(),
                next_service_time);

            process_rx_fifo(CAN_RX_FIFO0);
            process_rx_fifo(CAN_RX_FIFO1);
            HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);

            // wait at least 1ms to prevent busy-spin on failed sends
            osSemaphoreWait(sem_can, std::max(next_service_time, 1UL));
        } else if (status == HAL_CAN_ERROR_TIMEOUT) {
            HAL_CAN_ResetError(handle_);
            status = HAL_CAN_Start(handle_);
            if (status == HAL_OK)
                status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
        }
    }
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
bool ODriveCAN::set_baud_rate(uint32_t baud_rate) {
    uint32_t prescaler = CAN_FREQ / baud_rate;
    if (prescaler * baud_rate == CAN_FREQ) {
        // valid baud rate
        config_.baud_rate = baud_rate;
        if (handle_) {
            handle_->Init.Prescaler = prescaler;
            return reinit();
        }
        return true;
    } else {
        // invalid baud rate - ignore
        return false;
    }
}

void ODriveCAN::process_rx_fifo(uint32_t fifo) {
    while (HAL_CAN_GetRxFifoFillLevel(handle_, fifo)) {
        CAN_RxHeaderTypeDef header;
        can_Message_t rxmsg;
        HAL_CAN_GetRxMessage(handle_, fifo, &header, rxmsg.buf);

        rxmsg.isExt = header.IDE;
        rxmsg.id = rxmsg.isExt ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
        rxmsg.len = header.DLC;
        rxmsg.rtr = header.RTR;

        // TODO: this could be optimized with an ahead-of-time computed
        // index-to-filter map

        size_t fifo0_idx = 0;
        size_t fifo1_idx = 0;

        // Find the triggered subscription item based on header.FilterMatchIndex
        auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [&](auto& s) {
            size_t current_idx = (s.fifo == 0 ? fifo0_idx : fifo1_idx)++;
            return (header.FilterMatchIndex == current_idx) && (s.fifo == fifo);
        });

        if (it == subscriptions_.end()) {
            continue;
        }

        it->callback(it->ctx, rxmsg);
    }
}

// Send a CAN message on the bus
bool ODriveCAN::send_message(const can_Message_t& txmsg) {
    if (HAL_CAN_GetError(handle_) != HAL_CAN_ERROR_NONE) {
        return false;
    }

    CAN_TxHeaderTypeDef header;
    header.StdId = txmsg.id;
    header.ExtId = txmsg.id;
    header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = txmsg.len;
    header.TransmitGlobalTime = FunctionalState::DISABLE;

    uint32_t retTxMailbox = 0;
    if (!HAL_CAN_GetTxMailboxesFreeLevel(handle_)) {
        return false;
    }

    return HAL_CAN_AddTxMessage(handle_, &header, (uint8_t*)txmsg.buf, &retTxMailbox) == HAL_OK;
}

// void ODriveCAN::set_error(Error error) {
//     error_ |= error;
// }

bool ODriveCAN::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
    auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [](auto& subscription) {
        return subscription.fifo == kCanFifoNone;
    });

    if (it == subscriptions_.end()) {
        return false;  // all subscription slots in use
    }

    it->callback = callback;
    it->ctx = ctx;
    it->fifo = CAN_RX_FIFO0;  // TODO: make customizable
    if (handle) {
        *handle = &*it;
    }

    bool is_extended = filter.id.index() == 1;
    uint32_t id = is_extended ? ((std::get<1>(filter.id) << 3) | (1 << 2)) : (std::get<0>(filter.id) << 21);
    uint32_t mask = (is_extended ? (filter.mask << 3) : (filter.mask << 21)) | (1 << 2);  // care about the is_extended bit

    CAN_FilterTypeDef hal_filter;
    hal_filter.FilterActivation = ENABLE;
    hal_filter.FilterBank = &*it - &subscriptions_[0];
    hal_filter.FilterFIFOAssignment = it->fifo;
    hal_filter.FilterIdHigh = (id >> 16) & 0xffff;
    hal_filter.FilterIdLow = id & 0xffff;
    hal_filter.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    hal_filter.FilterMaskIdLow = mask & 0xffff;
    hal_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    hal_filter.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(handle_, &hal_filter) != HAL_OK) {
        return false;
    }
    return true;
}

bool ODriveCAN::unsubscribe(CanSubscription* handle) {
    ODriveCanSubscription* subscription = static_cast<ODriveCanSubscription*>(handle);
    if (subscription < subscriptions_.begin() || subscription >= subscriptions_.end()) {
        return false;
    }
    if (subscription->fifo != kCanFifoNone) {
        return false;  // not in use
    }

    subscription->fifo = kCanFifoNone;

    CAN_FilterTypeDef hal_filter = {};
    hal_filter.FilterActivation = DISABLE;
    return HAL_CAN_ConfigFilter(handle_, &hal_filter) == HAL_OK;
}

// Transmit one bicycle telemetry frame approximately
// every 15 ms, distributing load across the CAN bus.
uint32_t ODriveCAN::service_bike_telemetry() {
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed =
        now - bike_telemetry_last_ms_;

    if (elapsed < BIKE_CAN_INTERVAL_MS) {
        return BIKE_CAN_INTERVAL_MS - elapsed;
    }

    if (send_bike_telemetry_frame(
            bike_telemetry_frame_)) {
        bike_telemetry_last_ms_ = now;

        bike_telemetry_frame_ =
            (bike_telemetry_frame_ + 1) %
            BIKE_CAN_FRAME_COUNT;

        return BIKE_CAN_INTERVAL_MS;
    }

    // No free CAN mailbox. Retry without blocking.
    return 2;
}

bool ODriveCAN::send_bike_telemetry_frame(
    uint8_t frame) {
    const auto& bike = odrv.bike_controller_;

    can_Message_t msg = {};
    msg.id = BIKE_CAN_BASE_ID + frame;
    msg.isExt = true;
    msg.rtr = false;
    msg.len = 8;

    float value0 = 0.0f;
    float value1 = 0.0f;

    switch (frame) {
        case 0:
            // Bicycle speed measurements
            value0 = bike.cadence_estimate_;
            value1 = bike.wheel_speed_estimate_;
            break;

        case 1:
            // Rider mechanical input
            value0 = bike.rider_torque_estimate_;
            value1 = bike.rider_power_estimate_;
            break;

        case 2:
            // Virtual drivetrain ratio
            value0 = bike.input_output_gear_ratio_;
            value1 = bike.target_input_output_gear_ratio_;
            break;

        case 3:
            // Final torque commands after limiting
            value0 = bike.crank_torque_command_;
            value1 = bike.wheel_torque_command_;
            break;

        case 4:
            // Electrical power
            value0 = bike.power_monitor_.battery_power;
            value1 =
                bike.power_monitor_.brake_resistor_power;
            break;

        case 5: {
            // Status and diagnostics
            value0 = bike.power_limit_factor_;

            // Bytes 0-3: float power limit factor
            std::memcpy(msg.buf, &value0, sizeof(float));

            // Byte 4: bike controller state
            msg.buf[4] =
                static_cast<uint8_t>(bike.current_state_);

            // Byte 5: operating mode
            msg.buf[5] =
                static_cast<uint8_t>(bike.mode_);

            // Byte 6: diagnostic flags
            uint8_t flags = 0;

            if (bike.battery_discharge_limited_)
                flags |= 0x01;

            if (bike.generator_power_limited_)
                flags |= 0x02;

            msg.buf[6] = flags;

            // Byte 7: protocol version
            msg.buf[7] = 1;

            return send_message(msg);
        }

        default:
            return false;
    }

    // Never transmit NaN or infinity as telemetry.
    if (!std::isfinite(value0)) {
        value0 = 0.0f;
    }

    if (!std::isfinite(value1)) {
        value1 = 0.0f;
    }

    std::memcpy(&msg.buf[0], &value0, sizeof(float));
    std::memcpy(&msg.buf[4], &value1, sizeof(float));

    return send_message(msg);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef* hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef* hcan) {}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
    // HAL_CAN_ResetError(hcan);
}
