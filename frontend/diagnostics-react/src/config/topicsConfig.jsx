// topicsConfig.jsx
import { TOPIC_TYPES } from "./topicTypes";

export const IGNORED_TOPICS = [
  "/rosout",
  "/client_count",
  "/connected_clients",
  "/parameter_events",
];

export const TOPICS_CONFIG = [
  {
    name: "/turtle1/cmd_vel",
    type: "Twist",
    ...TOPIC_TYPES["Twist"],
  },
  {
    name: "/turtle_enabled",
    type: "Bool",
    ...TOPIC_TYPES["Bool"],
  },
  {
    name: "/sin_wave",
    type: "Float64",
    ...TOPIC_TYPES["Float64"],
  },
  {
    name: "/odom",
    type: "Odometry",
    ...TOPIC_TYPES["Odometry"],
  },
  // Add new topics here
];
