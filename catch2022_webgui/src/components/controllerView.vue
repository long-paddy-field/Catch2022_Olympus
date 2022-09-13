<template>
  <div class="hello no_scroll">
    <!-- {{ servoAngle?.data }} -->
    <!-- <v-slider v-model="servoRef" label="track-color" v-on:update:model-value="() => { servoAngleTopic.publish({ data: servoRef }) }"></v-slider> -->
    Arm0 angle
    <v-slider v-model="moveRadRef0" label="track-color" max=250 thumb-label v-on:update:model-value="() => { moveRadTopic.publish({ data: [moveRadRef0*Math.PI/180, moveRadRef1*Math.PI/180] }) }"></v-slider>
    Arm1 angle
    <v-slider v-model="moveRadRef1" label="track-color" max=276 thumb-label v-on:update:model-value="() => { moveRadTopic.publish({ data: [moveRadRef0*Math.PI/180, moveRadRef1*Math.PI/180] }) }"></v-slider>
    Servo
    <v-slider v-model="servoAngleRef" label="track-color" max=360 thumb-label v-on:update:model-value="() => { servoAngleTopic.publish({ data: servoAngleRef*Math.PI/180 }) }"></v-slider>
    Stepper
    <v-slider v-model="stepperStateRef" :min="0" :max="5" :step="1" thumb-label v-on:update:model-value="() => { stepperStateTopic.publish({ data: stepperStateRef }) }"></v-slider>
    Pmp0
    <v-switch v-model="pmpStateRef0" v-on:update:model-value="() => { pmpStateTopic.publish({ data:Number(pmpStateRef1)<<1|Number(pmpStateRef0) }) }"></v-switch>
    Pmp1
    <v-switch v-model="pmpStateRef1" v-on:update:model-value="() => { pmpStateTopic.publish({ data:Number(pmpStateRef1)<<1|Number(pmpStateRef0)  }) }"></v-switch>

    <v-slider v-model="ledHsvRef0" label="track-color" max=360 :step="1" thumb-label v-on:update:model-value="() => { ledHsvTopic.publish({ data: [ledHsvRef0, ledHsvRef1, ledHsvRef2] }) }"></v-slider>
    <v-slider v-model="ledHsvRef1" label="track-color" max=255 :step="1" thumb-label v-on:update:model-value="() => { ledHsvTopic.publish({ data: [ledHsvRef0, ledHsvRef1, ledHsvRef2] }) }"></v-slider>
    <v-slider v-model="ledHsvRef2" label="track-color" max=255 :step="1" thumb-label v-on:update:model-value="() => { ledHsvTopic.publish({ data: [ledHsvRef0, ledHsvRef1, ledHsvRef2] }) }"></v-slider>
    
    Sensor Status
    {{isGrabbed?.data}}
  </div>

</template>

<script lang="ts" setup>
import { defineComponent, onMounted, reactive, ref } from 'vue';
import { createTopic, useSubscriber, connectRos } from '@/script/rosHook';
import { RoundSlider } from "vue-round-slider/src/index2.js"

type floatType = {
  data: number
}
let drawer = ref<boolean>(false);
const servoAngleRef = ref<number>(0)
const moveRadRef0 = ref<number>(0)
const moveRadRef1 = ref<number>(0)
const stepperStateRef = ref<number>(0)
const pmpStateRef0 = ref<boolean>(false)
const pmpStateRef1 = ref<boolean>(false)
const emergencyRef = ref<number>(0)
const ledHsvRef0 = ref<number>(0)
const ledHsvRef1 = ref<number>(0)
const ledHsvRef2 = ref<number>(0)

const servoAngleTopic = createTopic<floatType>({
  name: '/servo_angle',
  messageType: 'std_msgs/Float32',
});
const servoAngle = useSubscriber(servoAngleTopic);
const moveRadTopic = createTopic<{ data: number[] }>({
  name: '/move_rad',
  messageType: 'std_msgs/Float32MultiArray',
});
const moveRad = useSubscriber(moveRadTopic);
const stepperStateTopic = createTopic<floatType>({
  name: '/stepper_state',
  messageType: 'std_msgs/Int8',
});
const stepperState = useSubscriber(stepperStateTopic);
const pmpStateTopic = createTopic<{ data: number }>({
  name: '/pmp_state',
  messageType: 'std_msgs/Int8',
});
const pmpState = useSubscriber(pmpStateTopic);
const emergencyTopic = createTopic<{ data: number }>({
  name: '/emergency',
  messageType: 'std_msgs/Int8',
});
const emergency = useSubscriber(emergencyTopic);
const isGrabbedTopic = createTopic<{ data: number }>({
  name: '/is_grabbed',
  messageType: 'std_msgs/Int8',
});
const isGrabbed = useSubscriber(isGrabbedTopic);


const ledHsvTopic = createTopic<{ data: number[] }>({
  name: '/led_hsv',
  messageType: 'std_msgs/Int16MultiArray',
});

onMounted(
  () => {
    connectRos('ws://tomato.local:9090');
  },
);



</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style>
.no_scroll {
  position: fixed;
  left: 0;
  right: 0;
  overflow: hidden;
}
</style>

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

.hello {
  position: fixed;
  left: 0;
  right: 0;
  overflow: hidden;
  margin: 10%;
}
</style>
