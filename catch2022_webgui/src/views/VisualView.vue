<template>
  <VisualController></VisualController>
</template>

<script lang="ts">
import { defineComponent } from 'vue';

// Components
import VisualController from '@/components/visualController.vue';

export default defineComponent({
  name: 'VisualView',

  components: {
    VisualController
},
});
</script>
