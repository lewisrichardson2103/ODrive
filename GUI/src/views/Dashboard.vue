<template>
  <div class="dashboard">
    <!-- PARAMETER DROPDOWN MENU -->
    <div v-show="paramsVisible" class="dropdown">
      <div class="card dropdown-content">
        <div>
          <button class="close-button" @click="hideTree">X</button>
          Parameters
        </div>
        <div class="param-tree">
        <json-view
          :data="treeParams"
          :rootKey="'odrives'"
          v-on:selected="addVarToElement"
        />
        </div>
      </div>
    </div>
    <div class="dashboard_container">
      <div class="controls">
        <template v-for="(control, index) in dash.controls">
          <component
            :is="control.controlType"
            :key="index + '-control'"
            :path="control.path"
            :name="control.name"
            :options="getEnumOptions(control.path) || control.options"
            :odrives="odrives"
            :dashID="dash.id"
          />
        </template>
        <div class="control-buttons">
          <div class="add-button card" @click="addComponent('control')">Add Control</div>
          <div class="add-button card" @click="addComponent('slider')">Add Slider</div>
          <div class="add-button card" @click="addComponent('function')">Add Function</div>
        </div>
        <template v-for="(action, index) in dash.actions">
          <component
            :is="action.actionType"
            :id="action.id"
            :key="index + '-action'"
            :path="action.path"
            :odrives="odrives"
            :initVal="action.val"
            :dashID="dash.id"
            :options="getEnumOptions(action.path) || action.options"
          />
        </template>
        <div class="add-button card" @click="addComponent('action')">Add Action</div>
      </div>
      <div class="plots">
        <template v-for="(plot, index) in dash.plots">
          <plot
            :plot="plot"
            :key="index"
            :name="plot.name"
            :dashID="dash.id"
            v-on:add-var="currentPlot=plot.name;addComponent('plot')"
          />
        </template>
        <div class="add-button card" @click="addPlot">Add Plot</div>
      </div>
    </div>
  </div>
</template>

<script>
//this component will get passed a list of controls and plots
//display controls on the left and plots on the right?
//leave full names for plots
//collate controls into individual cards based on the deepest common level
import CtrlBoolean from "../components/controls/CtrlBoolean.vue";
import CtrlNumeric from "../components/controls/CtrlNumeric.vue";
import CtrlFunction from "../components/controls/CtrlFunction.vue";
import CtrlSlider from "../components/controls/CtrlSlider.vue";
import CtrlEnum from "../components/controls/CtrlEnum.vue";
import Plot from "../components/plots/Plot.vue";
import Action from "../components/actions/Action.vue";
import ActionEnum from "../components/actions/ActionEnum.vue";
import generatedEnums from "../assets/odriveEnums.json";
import { JSONView } from "vue-json-component";
import { v4 as uuidv4 } from "uuid";

let plotColors = [
  "#1f77b4", // blue
  "#ff7f0e", // orange
  "#2ca02c", // green
  "#d62728", // red
  "#9467bd", // purple
  "#8c564b", // brown
  "#e377c2", // pink
  "#7f7f7f", // gray
  "#bcbd22", // olive
  "#17becf", // cyan
];

// Match enum types by their complete device-relative property path.
// Several unrelated properties share names such as "mode" and "current_state".
const enumPrefixes = {
  "bike_controller.current_state": "BIKE_STATE_",
  "bike_controller.requested_state": "BIKE_STATE_",
  "bike_controller.mode": "BIKE_MODE_",

  "axis0.current_state": "AXIS_STATE_",
  "axis0.requested_state": "AXIS_STATE_",
  "axis1.current_state": "AXIS_STATE_",
  "axis1.requested_state": "AXIS_STATE_",

  "axis0.encoder.config.mode": "ENCODER_MODE_",
  "axis1.encoder.config.mode": "ENCODER_MODE_",

  "axis0.motor.config.motor_type": "MOTOR_TYPE_",
  "axis1.motor.config.motor_type": "MOTOR_TYPE_",

  "axis0.controller.config.control_mode": "CONTROL_MODE_",
  "axis1.controller.config.control_mode": "CONTROL_MODE_",
  "axis0.controller.config.input_mode": "INPUT_MODE_",
  "axis1.controller.config.input_mode": "INPUT_MODE_",
};

// json-view paths can contain the "odrives" root, whereas existing
// dashboard definitions commonly begin directly with "odrive0".
function getRelativePath(path) {
  if (typeof path !== "string") return "";
  return path
    .replace(/^odrives\./, "")
    .replace(/^odrive\d+\./, "");
}

function getEnumOptions(path) {
  const prefix = enumPrefixes[getRelativePath(path)];
  if (!prefix) return null;

  const options = Object.entries(generatedEnums)
    .filter(([name, value]) =>
      name.startsWith(prefix) && Number.isFinite(value)
    )
    .map(([name, value]) => ({
      text: name.slice(prefix.length)
        .toLowerCase()
        .replace(/_/g, " ")
        .replace(/\b\w/g, char => char.toUpperCase()),
      value,
    }))
    .sort((a, b) => a.value - b.value);

  return options.length > 0 ? options : null;
}

export default {
  name: "Dashboard",
  components: {
    CtrlBoolean,
    CtrlNumeric,
    CtrlFunction,
    CtrlSlider,
    CtrlEnum,
    Plot,
    Action,
    ActionEnum,
    "json-view": JSONView,
  },
  props: ["dash", "odrives"],
  data() {
    return {
      paramsVisible: false,
      addCompType: undefined,
      currentPlot: undefined,
      treeParams: undefined,
    };
  },
  methods: {
    getEnumOptions,
    deleteAction(e) {
      this.$emit("delete-action", e);
      console.log(e);
    },
    deletePlot(e) {
      this.$emit("delete-plot", e);
    },
    addVar(e) {
      this.$emit("add-var", e);
    },
    showTree() {
      //show the parameter tree
      this.paramsVisible = true;
    },
    hideTree() {
      this.paramsVisible = false;
      this.addCompType = undefined;
    },
    addComponent(componentType) {
      this.addCompType = componentType;
      switch (this.addCompType) {
        case "control":
          this.treeParams = this.$store.state.odriveConfigs['params'];
          console.log(this.treeParams);
          break;
        case "plot":
          this.treeParams = this.$store.state.odriveConfigs['params'];
          break;
        case "slider":
          this.treeParams = this.$store.state.odriveConfigs['writeAbleNumeric'];
          break;
        case "action":
          this.treeParams = this.$store.state.odriveConfigs['writeAble'];
          break;
        case "function":
          this.treeParams = this.$store.state.odriveConfigs['functions'];
          break;
        default:
          break;
      }
      this.paramsVisible = true;
    },
    addVarToElement(e) {
      //when the parameter tree is open and a parameter is clicked,
      //add the clicked parameter to the list of controls for the
      //current dashboard
      console.log(e);
      switch (this.addCompType) {
        case "control":
          switch (typeof e.value) {
            case "boolean":
              this.dash.controls.push({
                controlType: "CtrlBoolean",
                path: e.path,
              });
              //this.$store.commit("addSampledProperty", e.path);
              break;
            case "number": {
              const options = getEnumOptions(e.path);
              if (options) {
                this.dash.controls.push({
                  controlType: "CtrlEnum",
                  path: e.path,
                  options,
                })
              }
              else {
                this.dash.controls.push({
                  controlType: "CtrlNumeric",
                  path: e.path,
                });
              }
              //this.$store.commit("addSampledProperty", e.path);
              break;
            }
            case "string":
              this.dash.controls.push({
                controlType: "CtrlFunction",
                path: e.path,
              });
              break;
            default:
              break;
          }
          break;
        case "function":
          this.dash.controls.push({
            controlType: "CtrlFunction",
            path: e.path,
          });
          break;
        case "plot":
          // add the selected element to the plot var list
          // add the selected element to the sampling var list
          // find the plot, append path to plot.vars
          console.log(e);
          for (const plot of this.dash.plots) {
            if (plot.name == this.currentPlot) {
              plot.vars.push({
                path: e.path,
                color: plotColors[plot.vars.length % plotColors.length],
              });
              this.$store.commit("addSampledProperty", e.path);
              console.log(plot);
              break;
            }
          }
          break;
        case "action":
          {
            // add an action to the current dash
            let id = uuidv4();
            console.log("action, key = " + e.key);
            const options = getEnumOptions(e.path);
            if (options) {
              this.dash.actions.push({
                actionType: "ActionEnum",
                id: id,
                path: e.path,
                options,
              })
            }
            else {
              this.dash.actions.push({
                actionType: "Action",
                id: id,
                path: e.path,
                val: undefined,
              });
            }
          }
          break;
        case "slider":
          // add a slider to the list of controls if the selected item is valid (numeric)
          switch (typeof e.value) {
            case "number":
              this.dash.controls.push({
                controlType: "CtrlSlider",
                path: e.path,
              });
              break;
            default:
              break;
          }
      }
    },
    addPlot() {
      let plotId = uuidv4();
      this.dash.plots.push({
        name: plotId,
        vars: [],
      });
    },
  },
};
</script>

<style scoped>
.dashboard {
  background-color: var(--bg-color);
  height: 100vh;
  max-height: 100vh;
  width: 100vw;
  padding-top: var(--top-height);
  padding-bottom: var(--bottom-height);
}

.dashboard_container {
  display: flex;
  flex-direction: row;
  height: 95vh;
}
.controls {
  flex-grow: 1;
  border-right: 1px solid lightgrey;
  overflow-y: scroll;
}

.add-button {
  background-color: lightcyan;
  width: 120px;
  margin: 10px auto;
  margin-left: 10px;
  margin-right: 10px;
  text-align: center;
  border-radius: 20px;
  cursor: pointer;
  user-select: none;
}

.add-button:active {
  background-color: lightblue;
}

.plots {
  flex-grow: 1;
  max-height: 94vh;
  overflow-y: scroll;
  display: flex;
  flex-direction: column;
}

.control-buttons {
  display: flex;
  flex-direction: row;
}

.dropdown {
  position: absolute;
  padding: 0 0;
  display: inline-block;
}

.dropdown-content {
  position: absolute;
  z-index: 1;
}

.param-tree {
  overflow-y: scroll;
  max-height: 85vh;
}
</style>