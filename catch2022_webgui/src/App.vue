<template>
  <div class="no_scroll">
    <v-app>
      <v-navigation-drawer v-model="drawer" temporary>
        <v-list density="compact" nav>
          <v-list-item prepend-icon="mdi-view-dashboard" title="Home" value="home" to='/'></v-list-item>
          <v-list-item prepend-icon="mdi-forum" title="About" value="about" to='/about'></v-list-item>
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
        <router-view></router-view>
      </v-main>
    </v-app>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';

export default defineComponent({
  name: 'App',

  data() {
    return {
      drawer: false//
    };
  },
});
</script>
