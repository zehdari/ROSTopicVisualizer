import React from "react";
import GraphCard from "./GraphCard";
import VideoCard from "./VideoCard";
import "../styles/RealtimeGraph.css";
import { NETWORK_CONFIG } from "../config/networkConfig";

const RealtimeGraph = ({
  visibleTopics,
  visibleVideos,
  updateVisibleTopics,
  updateVisibleVideos,
}) => {
  // Combine graphs and videos into a single array with type and timestamp
  const allCards = [
    ...visibleTopics.map((topic) => ({
      type: "graph",
      data: topic,
      timestamp: topic.timestamp || Date.now(), // Use existing timestamp or current time
      key: topic.name,
    })),
    ...visibleVideos.map((video) => ({
      type: "video",
      data: video,
      timestamp: video.timestamp || Date.now(), // Use existing timestamp or current time
      key: video.topic,
    })),
  ].sort((a, b) => a.timestamp - b.timestamp); // Sort by timestamp

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
        console.log(`Stopping video server for ${topic}`);
        updateVisibleVideos(updatedVideos);
      } else {
        const updatedVideos = visibleVideos.filter(
          (video) => video.topic !== topic
        );
        console.error("Failed to stop video server");
        updateVisibleVideos(updatedVideos);
      }
    } catch (error) {
      const updatedVideos = visibleVideos.filter(
        (video) => video.topic !== topic
      );
      console.error("Error stopping video server:", error);
      updateVisibleVideos(updatedVideos);
    }
  };

  return (
    <div className="realtime-graph-container">
      <div className="visible-graphs">
        {allCards.map((card) =>
          card.type === "video" ? (
            <VideoCard
              key={card.key}
              topic={card.data.topic}
              port={card.data.port}
              onRemoveVideo={handleRemoveVideo}
            />
          ) : (
            <GraphCard
              key={card.key}
              topicConfig={card.data}
              onRemoveGraph={handleRemoveGraph}
            />
          )
        )}
      </div>
    </div>
  );
};

export default RealtimeGraph;
