// https://stackoverflow.com/questions/69491226/after-using-vitevue3-to-build-the-project-and-importing-roslibjs-an-error-is-r
import ROSLIB from 'roslib';
import 'roslib/build/roslib';
import {
  onMounted, ref, onUnmounted, reactive, computed, toRefs,
} from 'vue';

const roslib = window.ROSLIB;
export type TopicInfo = {
  name: string;
  messageType: string;
};

export type ActionInfo = {
  serverName: string;
  actionName: string;
};

export type ActionStatus = {
  // eslint-disable-next-line @typescript-eslint/naming-convention, camelcase
  goal_id: {
    stamp: {
      secs: number;
      nsecs: number;
    };
    id: string;
  };
  status: number;
  text: string;
};

export class GoalTyped<TGoal, TFeedback, TResult> extends roslib.Goal {
  // eslint-disable-next-line no-useless-constructor
  constructor(options: { actionClient: ROSLIB.ActionClient; goalMessage: TGoal }) {
    super(options);
  }

  on(eventName: 'feedback', callback: (event: TFeedback) => void): void;

  on(eventName: 'result', callback: (event: TResult) => void): void;

  on(eventName: 'status', callback: (event: ActionStatus) => void): void;

  on(
    eventName: 'feedback' | 'result' | 'status',
    callback:
      | ((event: TFeedback) => void)
      | ((event: TResult) => void)
      | ((event: ActionStatus) => void),
  ) {
    super.on(eventName, callback);
  }
}

const ros = new roslib.Ros({
  // url: 'ws://localhost:9090',
});

export function connectRos(url: string) {
  ros.connect(url);
}

type RosStatus = {
  isConnected: boolean;
  isError: boolean;
  errorEvent?: Event;
};

export function useRosStatus() {
  const status = reactive<RosStatus>({
    isConnected: ros.isConnected,
    isError: false,
  });
  const connectonListener = computed(() => () => {
    status.isConnected = true;
    console.log('ros connected');
  });
  const closeListener = computed(() => () => {
    status.isConnected = false;
  });
  const errorListener = computed(() => (event: Event) => {
    status.isError = true;
    status.errorEvent = event;
  });
  onMounted(() => {
    ros.on('connection', connectonListener.value);
    ros.on('close', closeListener.value);
    ros.on('error', errorListener.value);
  });
  onUnmounted(() => {
    ros.off('connected', connectonListener.value);
    ros.off('close', closeListener.value);
    ros.off('error', errorListener.value);
  });
  return toRefs(status);
}

export const createTopic = <T extends object>(topicInfo: TopicInfo) => new roslib.Topic<T>({
  ros,
  name: topicInfo.name,
  messageType: topicInfo.messageType,
});

export const useSubscriber = <T>(topic: ROSLIB.Topic<T>) => {
  const subscribeData = ref<T>();
  onMounted(() => {
    topic.subscribe((msg) => {
      subscribeData.value = msg;
    });
  });
  onUnmounted(() => {
    topic.unsubscribe();
  });
  return subscribeData;
};

// eslint-disable-next-line max-len
export const createActionClient = ({ serverName, actionName }: ActionInfo, timeout: number) => new roslib.ActionClient({
  ros,
  serverName,
  actionName,
  timeout,
});

export const useGoal = <TGoal, TFeedback, TResult>(actionClient: ROSLIB.ActionClient) => {
  const result = ref<TResult>();
  const feedback = ref<TFeedback>();
  const status = ref<ActionStatus>();
  const cancel = ref<() => void>();

  const goal = ref<GoalTyped<TGoal, TFeedback, TResult>>();

  function sendGoal(data: TGoal) {
    goal.value = new GoalTyped({
      actionClient,
      goalMessage: data,
    });
    goal.value.on('feedback', (e) => {
      feedback.value = e;
    });
    goal.value.on('result', (e) => {
      result.value = e;
    });
    goal.value.on('status', (e) => {
      status.value = e;
    });
    cancel.value = () => {
      goal.value?.cancel();
    };
    goal.value.send();
  }

  return {
    sendGoal,
    result,
    feedback,
    status,
    cancel,
  };
};

export const publishTopic = <T>(topic: ROSLIB.Topic<T>, data: T) => {
  topic.publish(data);
};
