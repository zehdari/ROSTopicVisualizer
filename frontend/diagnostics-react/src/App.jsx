import React from "react";
import RealtimeGraph from "./components/RealtimeGraph";
import "./styles/App.css";
import TopicsTable from "./components/TopicsTable";
import { useVisibleTopics } from "./utils/useVisibleTopics"; // Import the custom hook
import TfTree from "./components/TfTree";

const App = () => {
  const {
    visibleTopics,
    visibleVideos,
    handleAddGraph,
    handleAddVideo,
    updateVisibleTopics,
    updateVisibleVideos,
  } = useVisibleTopics();

  return (
    <div className="app-container">
      <h1 className="app-header">Talos Diagnostics</h1>
      <RealtimeGraph
        visibleTopics={visibleTopics}
        visibleVideos={visibleVideos}
        updateVisibleTopics={updateVisibleTopics}
        updateVisibleVideos={updateVisibleVideos}
      />
      <div className="main-container">
        <TopicsTable
          onAddGraph={handleAddGraph}
          onAddVideo={handleAddVideo}
          visibleTopics={visibleTopics}
          visibleVideos={visibleVideos}
        />
      </div>
      <TfTree />
    </div>
  );
};

export default App;
