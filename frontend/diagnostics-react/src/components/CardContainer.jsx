import React from "react";
import GraphCard from "./GraphCard";
import VideoCard from "./VideoCard";
import PointCloudCard from "./PointCloudCard";
import "../styles/CardContainer.css";
import { NETWORK_CONFIG } from "../config/networkConfig";

const CardContainer = ({
  visibleTopics,
  visibleVideos,
  visiblePointClouds,
  updateVisibleTopics,
  updateVisibleVideos,
  updateVisiblePointClouds,
}) => {
  // Combine graphs, videos, and point clouds into a single array with type and timestamp
  const allCards = [
    ...visibleTopics.map((topic) => ({
      type: "graph",
      data: topic,
      timestamp: topic.timestamp || Date.now(),
      key: topic.name,
    })),
    ...visibleVideos.map((video) => ({
      type: "video",
      data: video,
      timestamp: video.timestamp || Date.now(),
      key: video.topic,
    })),
    ...visiblePointClouds.map((pointCloud) => ({
      type: "pointcloud",
      data: pointCloud,
      timestamp: pointCloud.timestamp || Date.now(),
      key: pointCloud.name,
    })),
  ].sort((a, b) => a.timestamp - b.timestamp);

  const handleRemoveGraph = (topicName) => {
    const updatedTopics = visibleTopics.filter(
      (topic) => topic.name !== topicName
    );
    updateVisibleTopics(updatedTopics);
  };

  const handleRemoveVideo = async (topic) => {
    try {
      const response = await fetch(
        `${NETWORK_CONFIG.FLASK_SERVER_URL}/stop-video-server`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ topic }),
        }
      );
      if (response.ok) {
        const updatedVideos = visibleVideos.filter(
          (video) => video.topic !== topic
        );
        updateVisibleVideos(updatedVideos);
      }
    } catch (error) {
      const updatedVideos = visibleVideos.filter(
        (video) => video.topic !== topic
      );
      updateVisibleVideos(updatedVideos);
    }
  };

  const handleRemovePointCloud = (topicName) => {
    const updatedPointClouds = visiblePointClouds.filter(
      (pc) => pc.name !== topicName
    );
    updateVisiblePointClouds(updatedPointClouds);
  };

  return (
    <div className="card-container">
      <div className="cards-grid">
        {allCards.map((card) => {
          if (card.type === "video") {
            return (
              <VideoCard
                key={card.key}
                topic={card.data.topic}
                port={card.data.port}
                onRemoveVideo={handleRemoveVideo}
              />
            );
          } else if (card.type === "pointcloud") {
            return (
              <PointCloudCard
                key={card.key}
                topic={card.data.name}
                onRemovePointCloud={handleRemovePointCloud}
              />
            );
          } else {
            return (
              <GraphCard
                key={card.key}
                topicConfig={card.data}
                onRemoveGraph={handleRemoveGraph}
              />
            );
          }
        })}
      </div>
    </div>
  );
};

export default CardContainer;
