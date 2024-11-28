import React from "react";
import RealtimeGraph from "./RealtimeGraph";
import "./App.css";
import TopicsTable from "./TopicsTable";
import { useVisibleTopics } from "./useVisibleTopics"; // Import the custom hook

const App = () => {
  const { visibleTopics, handleAddGraph, updateVisibleTopics } =
    useVisibleTopics();

  return (
    <div className="app-container">
      <h1 className="app-header">Talos Diagnostics</h1>
      <RealtimeGraph
        visibleTopics={visibleTopics}
        updateVisibleTopics={updateVisibleTopics}
      />
      <TopicsTable onAddGraph={handleAddGraph} visibleTopics={visibleTopics} />
    </div>
  );
};

export default App;
