// RealtimeGraph.jsx
import React from "react";
import GraphCard from "./GraphCard";
import { TOPICS_CONFIG } from "./topicsConfig";
import "./RealtimeGraph.css";

const RealtimeGraph = () => {
  return (
    <div className="graph-container">
      {TOPICS_CONFIG.map((topic) => (
        <GraphCard key={topic.name} topicConfig={topic} />
      ))}
    </div>
  );
};

export default RealtimeGraph;
