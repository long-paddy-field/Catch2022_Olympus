<template>
  <div class="hello">
    {{ servoAngle?.data }}
    <v-btn @click="()=>{servoAngleTopic.publish({data:50})}">hello</v-btn>
     <v-slider
      v-model="servoRef"
      label="track-color"
      v-on:update:model-value="()=>{servoAngleTopic.publish({data:servoRef})}"
    ></v-slider>
    <v-slider
      v-model="moveCmdRef0"
      label="track-color"
      v-on:update:model-value="()=>{
        moveCmdTopic.publish({data:[moveCmdRef0,moveCmdRef1]})}"
    ></v-slider>
    <v-slider
      v-model="moveCmdRef1"
      label="track-color"
      v-on:update:model-value="()=>{moveCmdTopic.publish({data:[moveCmdRef0,moveCmdRef1]})}"
    ></v-slider>
  </div>
</template>

<script lang="ts" setup>
import { defineComponent, onMounted, reactive, ref } from 'vue';
import { createTopic, useSubscriber, connectRos } from '@/script/rosHook';

type floatType = {
  data: number
}

const servoRef=ref<number>(0)
const moveCmdRef0=ref<number>(0)
const moveCmdRef1=ref<number>(0)
const stepperStateRef=ref<number>(0)
const pmpStateRef=ref<number>(0)
const emergencyRef=ref<number>(0)

const servoAngleTopic = createTopic<floatType>({
  name: '/servo_angle',
  messageType: 'std_msgs/Float32',
});
const servoAngle = useSubscriber(servoAngleTopic);
const moveCmdTopic = createTopic<{data:number[]}>({
  name: '/move_cmd',
  messageType: 'std_msgs/Float32',
});
const moveCmd = useSubscriber(moveCmdTopic);
const stepperStateTopic = createTopic<floatType>({
  name: '/stepper_state',
  messageType: 'std_msgs/Int8',
});
const stepperState = useSubscriber(stepperStateTopic);
const pmpStateTopic = createTopic<{data:boolean}>({
  name: '/pmp_state',
  messageType: 'std_msgs/Bool',
});
const pmpState = useSubscriber(pmpStateTopic);
const emergencyTopic = createTopic<{data:number}>({
  name: '/emergency',
  messageType: 'std_msgs/Int8',
});
const emergency = useSubscriber(emergencyTopic);

onMounted(
  () => {
    connectRos('ws://0.0.0.0:9090');
  },
);

</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
h3 {
  margin: 40px 0 0;
}

ul {
  list-style-type: none;
  padding: 0;
}

li {
  display: inline-block;
  margin: 0 10px;
}

a {
  color: #42b983;
}
</style>
