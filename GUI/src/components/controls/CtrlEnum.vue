<template>
  <div class="card">
    <button class="close-button" @click="deleteCtrl">X</button>
    <span class="ctrlName">{{ name }}:</span>
    <div class="right">
      <!-- Writable enum: show dropdown -->
      <select
        v-if="writeAccess"
        class="ctrl-enum"
        v-model="selected"
        @change="putVal"
      >
        <option
          v-for="option in options"
          :key="option.value"
          :value="option.value"
        >
          {{ option.text }}
        </option>
      </select>

      <!-- Read-only enum: show current state as text -->
      <span v-else class="ctrlVal">
        {{ selectedLabel }}
      </span>
    </div>
  </div>
</template>

<script>
import {
  getVal,
  getReadonly,
  putVal,
  fetchParam,
} from "../../lib/odrive_utils.js";

export default {
  name: "CtrlEnum",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
    options: Array,
  },
  data() {
    return {
      selected: undefined,
    };
  },
  computed: {
    value: function() {
      let keys = this.path.split(".");
      keys.shift();
      return parseFloat(getVal(keys.join(".")));
    },
    name: function() {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function() {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getReadonly(keys.join(".")) === false;
    },
    selectedLabel: function() {
      const currentValue = this.value;

      const option = (this.options || []).find(
        (option) => Number(option.value) === currentValue,
      );

      if (option) {
        return option.text;
      }

      // Fallback if the firmware reports an unknown enum value
      return Number.isNaN(currentValue)
        ? "Unknown"
        : `Unknown (${currentValue})`;
    },
  },
  watch: {
    value: function(newVal) {
      this.selected = newVal;
    },
  },
  methods: {
    putVal: function(e) {
      let keys = this.path.split(".");
      keys.shift();
      console.log("Calling putVal from CtrlEnum");
      putVal(keys.join("."), parseFloat(e.target.value));
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {
        dashID: this.dashID,
        path: this.path,
      });
    },
  },
  created() {
    // update parameter value on component creation
    let keys = this.path.split(".");
    keys.shift();
    fetchParam(keys.join("."));

    this.selected = this.value;
    console.log(this.value);
  },
};
</script>

<style scoped>
.ctrlVal {
  font-weight: bold;
}

input {
  width: 5rem;
  font-family: inherit;
  border-style: none;
  border-bottom: 1px solid grey;
  text-align: center;
}

.right {
  display: flex;
  flex-direction: row;
  margin-left: auto;
}

.card {
  display: flex;
}

.ctrl-enum {
  border: none;
  border-bottom: 1px solid grey;
  background-color: var(--fg-color);
  text-align-last: right;
}
</style>
