import React from "react";
import RealtimeGraph from "./components/RealtimeGraph";
import "./styles/App.css";
import TopicsTable from "./components/TopicsTable";
import { useVisibleTopics } from "./utils/useVisibleTopics"; // Import the custom hook
import TfTree from "./components/TfTree";
import ImageDisplay from "./components/ImageCard";

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
      <div className="main-container">
        <TopicsTable
          onAddGraph={handleAddGraph}
          visibleTopics={visibleTopics}
        />
      </div>
      <TfTree />
      <ImageDisplay />
    </div>
  );
};

export default App;
