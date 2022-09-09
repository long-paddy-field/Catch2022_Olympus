<template>
  <controller-view />
</template>

<script lang="ts">
import { defineComponent } from 'vue';

// Components
import ControllerView from '@/components/controllerView.vue';

export default defineComponent({
  name: 'HomeView',

  components: {
    ControllerView
},
});
</script>
