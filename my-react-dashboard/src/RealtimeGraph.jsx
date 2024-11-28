import React from "react";
import GraphCard from "./GraphCard";
import "./RealtimeGraph.css";

const RealtimeGraph = ({ visibleTopics, updateVisibleTopics }) => {
  const handleRemoveGraph = (topicName) => {
    // Filter out the topic to be removed
    const updatedTopics = visibleTopics.filter(
      (topic) => topic.name !== topicName
    );
    // Update the visible topics in the parent component
    updateVisibleTopics(updatedTopics);
  };

  return (
    <div className="realtime-graph-container">
      <div className="visible-graphs">
        {visibleTopics.map((topic) => (
          <GraphCard
            key={topic.name} // Use unique topic name as the key
            topicConfig={topic}
            onRemoveGraph={handleRemoveGraph}
          />
        ))}
      </div>
    </div>
  );
};

export default RealtimeGraph;
