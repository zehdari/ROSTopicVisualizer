import { TOPIC_TYPES } from "./topicTypes";

export const IGNORED_TOPICS = [
  "/rosout",
  "/client_count",
  "/connected_clients",
  "/parameter_events",
];

// Helper function to generate topic configuration
const createTopicConfig = (name, type, overrides = {}) => {
  if (!TOPIC_TYPES[type]) {
    throw new Error(`Type ${type} is not defined in TOPIC_TYPES`);
  }
  return {
    name,
    type,
    ...TOPIC_TYPES[type], // Default settings for the type
    ...overrides, // Apply specific overrides for this topic
  };
};

// Topics Configuration
// For overloading topic types config for specific topics
export const TOPICS_CONFIG = [
  // createTopicConfig("/odom", "nav_msgs/msg/Odometry", {
  //   parser: (message) => ({
  //     time: new Date().toISOString(), // Example of an override
  //     positionX: message.pose?.pose?.position?.x || 0,
  //     positionY: message.pose?.pose?.position?.y || 0,
  //     positionZ: message.pose?.pose?.position?.z || 0,
  //   }),
  // }),
  // Add more topics here
];
