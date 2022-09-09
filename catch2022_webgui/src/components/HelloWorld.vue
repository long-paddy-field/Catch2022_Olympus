<template>
  <div class="no_scroll">
    <v-app id="inspire">
      <v-navigation-drawer v-model="drawer" temporary>
        <v-list density="compact" nav>
          <v-list-item prepend-icon="mdi-view-dashboard" title="Home" value="home"></v-list-item>
          <v-list-item prepend-icon="mdi-forum" title="About" value="about"></v-list-item>
          <v-list-item prepend-icon="mdi-reload" title="Reload" value="reload" @click="() => {
            $router.go(0)
          }"></v-list-item>
        </v-list>
      </v-navigation-drawer>

      <v-app-bar>
        <v-app-bar-nav-icon @click="drawer = !drawer" />

        <v-toolbar-title>ApoloCtrl</v-toolbar-title>
      </v-app-bar>

      <v-main>
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
          Pmp
          <v-switch v-model="pmpStateRef" v-on:update:model-value="() => { pmpStateTopic.publish({ data:pmpStateRef }) }"></v-switch>
        </div>
      </v-main>
    </v-app>
  </div>

</template>

<script lang="ts" setup>
import { defineComponent, onMounted, reactive, ref } from 'vue';
import { createTopic, useSubscriber, connectRos } from '@/script/rosHook';

type floatType = {
  data: number
}
let drawer = ref<boolean>(false);
const servoAngleRef = ref<number>(0)
const moveRadRef0 = ref<number>(0)
const moveRadRef1 = ref<number>(0)
const stepperStateRef = ref<number>(0)
const pmpStateRef = ref<boolean>(false)
const emergencyRef = ref<number>(0)
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
const pmpStateTopic = createTopic<{ data: boolean }>({
  name: '/pmp_state',
  messageType: 'std_msgs/Bool',
});
const pmpState = useSubscriber(pmpStateTopic);
const emergencyTopic = createTopic<{ data: number }>({
  name: '/emergency',
  messageType: 'std_msgs/Int8',
});
const emergency = useSubscriber(emergencyTopic);

onMounted(
  () => {
    connectRos('ws://zephyrus.local:9090');
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
