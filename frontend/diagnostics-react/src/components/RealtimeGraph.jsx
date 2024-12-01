import React, { useState, useEffect } from "react";
import {
  DndContext,
  closestCenter,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import {
  arrayMove,
  SortableContext,
  sortableKeyboardCoordinates,
  verticalListSortingStrategy,
} from "@dnd-kit/sortable";
import { SortableCard } from "./SortableCard";
import GraphCard from "./GraphCard";
import VideoCard from "./VideoCard";
import PointCloudCard from "./PointCloudCard";
import "../styles/RealtimeGraph.css";
import { NETWORK_CONFIG } from "../config/networkConfig";

const RealtimeGraph = ({
  visibleTopics,
  visibleVideos,
  visiblePointClouds,
  updateVisibleTopics,
  updateVisibleVideos,
  updateVisiblePointClouds,
}) => {
  const [items, setItems] = useState([]);

  // Update items whenever props change
  useEffect(() => {
    const newItems = [
      ...visibleTopics.map((topic) => ({
        type: "graph",
        data: topic,
        timestamp: topic.timestamp || Date.now(),
        id: `graph-${topic.name}`,
      })),
      ...visibleVideos.map((video) => ({
        type: "video",
        data: video,
        timestamp: video.timestamp || Date.now(),
        id: `video-${video.topic}`,
      })),
      ...visiblePointClouds.map((pointCloud) => ({
        type: "pointcloud",
        data: pointCloud,
        timestamp: pointCloud.timestamp || Date.now(),
        id: `pointcloud-${pointCloud.name}`,
      })),
    ].sort((a, b) => a.timestamp - b.timestamp);

    setItems(newItems);
  }, [visibleTopics, visibleVideos, visiblePointClouds]);

  const sensors = useSensors(
    useSensor(PointerSensor),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    })
  );

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
      }
    } catch (error) {
      console.error("Error stopping video server:", error);
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

  const handleDragEnd = (event) => {
    const { active, over } = event;

    if (active.id !== over.id) {
      setItems((items) => {
        const oldIndex = items.findIndex((item) => item.id === active.id);
        const newIndex = items.findIndex((item) => item.id === over.id);
        return arrayMove(items, oldIndex, newIndex);
      });
    }
  };

  const renderCard = (card) => {
    switch (card.type) {
      case "video":
        return (
          <VideoCard
            topic={card.data.topic}
            port={card.data.port}
            onRemoveVideo={handleRemoveVideo}
          />
        );
      case "pointcloud":
        return (
          <PointCloudCard
            topic={card.data.name}
            onRemovePointCloud={handleRemovePointCloud}
          />
        );
      default:
        return (
          <GraphCard
            topicConfig={card.data}
            onRemoveGraph={handleRemoveGraph}
          />
        );
    }
  };

  return (
    <div className="realtime-graph-container">
      <DndContext
        sensors={sensors}
        collisionDetection={closestCenter}
        onDragEnd={handleDragEnd}
      >
        <SortableContext
          items={items.map((item) => item.id)}
          strategy={verticalListSortingStrategy}
        >
          <div className="visible-graphs">
            {items.map((card) => (
              <SortableCard key={card.id} id={card.id}>
                {renderCard(card)}
              </SortableCard>
            ))}
          </div>
        </SortableContext>
      </DndContext>
    </div>
  );
};

export default RealtimeGraph;
