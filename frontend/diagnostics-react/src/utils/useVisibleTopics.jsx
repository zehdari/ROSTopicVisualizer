import { useState } from "react";
import { TOPICS_CONFIG } from "../config/topicsConfig";
import { TOPIC_TYPES } from "../config/topicTypes";

export const useVisibleTopics = () => {
  const [visibleTopics, setVisibleTopics] = useState([]);
  const [visibleVideos, setVisibleVideos] = useState([]);
  const [visiblePointClouds, setVisiblePointClouds] = useState([]);

  const handleAddGraph = (newTopic) => {
    // Handle PointCloud2 separately
    if (newTopic.type === "sensor_msgs/msg/PointCloud2") {
      handleAddPointCloud(newTopic);
      return;
    }

    // Regular graph handling
    if (!visibleTopics.some((topic) => topic.name === newTopic.name)) {
      const { name, type } = newTopic;
      const configTopic = TOPICS_CONFIG.find((topic) => topic.name === name);
      if (configTopic) {
        setVisibleTopics((prev) => [...prev, configTopic]);
        return;
      }
      if (TOPIC_TYPES[type]) {
        const dynamicTopicConfig = {
          name,
          type,
          ...TOPIC_TYPES[type],
        };
        setVisibleTopics((prev) => [...prev, dynamicTopicConfig]);
      } else {
        console.error(`Type ${type} is not defined in TOPIC_TYPES`);
      }
    }
  };

  const handleAddPointCloud = (newTopic) => {
    if (!visiblePointClouds.some((topic) => topic.name === newTopic.name)) {
      setVisiblePointClouds((prev) => [...prev, newTopic]);
    }
  };

  const handleAddVideo = (newVideo) => {
    if (!visibleVideos.some((video) => video.topic === newVideo.topic)) {
      setVisibleVideos((prev) => [...prev, newVideo]);
    }
  };

  const updateVisibleTopics = (updatedTopics) => {
    setVisibleTopics(updatedTopics);
  };

  const updateVisibleVideos = (updatedVideos) => {
    setVisibleVideos(updatedVideos);
  };

  const updateVisiblePointClouds = (updatedPointClouds) => {
    setVisiblePointClouds(updatedPointClouds);
  };

  return {
    visibleTopics,
    visibleVideos,
    visiblePointClouds,
    handleAddGraph,
    handleAddVideo,
    handleAddPointCloud,
    updateVisibleTopics,
    updateVisibleVideos,
    updateVisiblePointClouds,
  };
};
