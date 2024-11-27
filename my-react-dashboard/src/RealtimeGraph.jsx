// RealtimeGraph.jsx
import React, { useState } from "react";
import GraphCard from "./GraphCard";
import { TOPICS_CONFIG } from "./topicsConfig";
import "./RealtimeGraph.css";

const RealtimeGraph = () => {
  // Initialize visibility state for each topic
  const initialVisibility = TOPICS_CONFIG.reduce((acc, topic) => {
    acc[topic.name] = true; // All graphs are visible by default
    return acc;
  }, {});

  const [visibility, setVisibility] = useState(initialVisibility);

  // Toggle visibility handler
  const toggleVisibility = (topicName) => {
    setVisibility((prev) => ({
      ...prev,
      [topicName]: !prev[topicName],
    }));
  };

  // Separate visible and hidden topics
  const visibleTopics = TOPICS_CONFIG.filter((topic) => visibility[topic.name]);
  const hiddenTopics = TOPICS_CONFIG.filter((topic) => !visibility[topic.name]);

  return (
    <div className="realtime-graph-container">
      <div className="visible-graphs">
        {visibleTopics.map((topic) => (
          <GraphCard
            key={topic.name}
            topicConfig={topic}
            graphVisible={true}
            toggleVisibility={toggleVisibility}
          />
        ))}
      </div>
      <div className="hidden-graphs">
        {hiddenTopics.map((topic) => (
          <GraphCard
            key={topic.name}
            topicConfig={topic}
            graphVisible={false}
            toggleVisibility={toggleVisibility}
          />
        ))}
      </div>
    </div>
  );
};

export default RealtimeGraph;
