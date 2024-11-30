import React, { useState } from "react";
import RealtimeGraph from "./components/RealtimeGraph";
import TopicsTable from "./components/TopicsTable";
import TfTree from "./components/TfTree";
import TerminalComponent from "./components/Terminal";
import { useVisibleTopics } from "./utils/useVisibleTopics";
import "./styles/App.css";

const App = () => {
  const [isTerminalOpen, setIsTerminalOpen] = useState(false);
  const [isTreeOpen, setIsTreeOpen] = useState(false);

  const {
    visibleTopics,
    visibleVideos,
    handleAddGraph,
    handleAddVideo,
    updateVisibleTopics,
    updateVisibleVideos,
  } = useVisibleTopics();

  const toggleTerminal = () => setIsTerminalOpen((prev) => !prev);
  const toggleTree = () => setIsTreeOpen((prev) => !prev);

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
        {isTreeOpen && <TfTree />}
        {isTerminalOpen && <TerminalComponent />}
        <TopicsTable
          onAddGraph={handleAddGraph}
          onAddVideo={handleAddVideo}
          visibleTopics={visibleTopics}
          visibleVideos={visibleVideos}
          isTerminalOpen={isTerminalOpen}
          isTreeOpen={isTreeOpen}
          onToggleTerminal={toggleTerminal}
          onToggleTree={toggleTree}
        />
      </div>
    </div>
  );
};

export default App;
