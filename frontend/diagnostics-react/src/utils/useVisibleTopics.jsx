import { useState } from "react";
import { TOPICS_CONFIG } from "../config/topicsConfig";

export const useVisibleTopics = () => {
  const [visibleTopics, setVisibleTopics] = useState(
    TOPICS_CONFIG.filter((topic) => false) // Initially show no topics
  );

  const handleAddGraph = (newTopic) => {
    if (!visibleTopics.some((topic) => topic.name === newTopic.name)) {
      setVisibleTopics((prev) => [...prev, newTopic]);
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
