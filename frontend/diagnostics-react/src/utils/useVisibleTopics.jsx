import { useState } from "react";
import { TOPICS_CONFIG } from "../config/topicsConfig";
import { TOPIC_TYPES } from "../config/topicTypes";

export const useVisibleTopics = () => {
  const [visibleTopics, setVisibleTopics] = useState([]);

  const handleAddGraph = (newTopic) => {
    // Check if the topic is already visible
    if (!visibleTopics.some((topic) => topic.name === newTopic.name)) {
      const { name, type } = newTopic;

      // First, check if the topic is in TOPICS_CONFIG
      const configTopic = TOPICS_CONFIG.find((topic) => topic.name === name);
      if (configTopic) {
        setVisibleTopics((prev) => [...prev, configTopic]);
        return;
      }

      // If not in TOPICS_CONFIG, check if type exists in TOPIC_TYPES
      if (TOPIC_TYPES[type]) {
        const dynamicTopicConfig = {
          name,
          type,
          ...TOPIC_TYPES[type], // Use the default configuration from TOPIC_TYPES
        };
        setVisibleTopics((prev) => [...prev, dynamicTopicConfig]);
      } else {
        console.error(`Type ${type} is not defined in TOPIC_TYPES`);
      }
    }
  };

  const updateVisibleTopics = (updatedTopics) => {
    setVisibleTopics(updatedTopics);
  };

  return {
    visibleTopics,
    handleAddGraph,
    updateVisibleTopics,
  };
};
