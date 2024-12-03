import React, { useState } from "react";
import CardContainer from "./components/CardContainer";
import TopicsTable from "./components/TopicsTable";
import TfTree from "./components/TfTree";
import TerminalComponent from "./components/Terminal";
import { useVisibleTopics } from "./utils/useVisibleTopics";
import "./styles/App.css";
import RobotMonitorCard from "./components/RobotMonitorCard";

const App = () => {
  const [isTerminalOpen, setIsTerminalOpen] = useState(false);
  const [isTreeOpen, setIsTreeOpen] = useState(false);

  const {
    visibleGraphs,
    visibleVideos,
    visiblePointClouds,
    visibleStatus,
    visibleDiagnostics,
    handleAddGraph,
    handleAddVideo,
    handleAddPointCloud,
    handleAddStatus,
    handleAddDiagnostic,
    updateVisibleGraphs,
    updateVisibleVideos,
    updateVisiblePointClouds,
    updateVisibleStatus,
    updateVisibleDiagnostics,
  } = useVisibleTopics();

  const toggleTerminal = () => setIsTerminalOpen((prev) => !prev);
  const toggleTree = () => setIsTreeOpen((prev) => !prev);

  return (
    <div className="app-container">
      <h1 className="app-header">Talos Diagnostics</h1>
      <CardContainer
        visibleGraphs={visibleGraphs}
        visibleVideos={visibleVideos}
        visiblePointClouds={visiblePointClouds}
        visibleStatus={visibleStatus}
        visibleDiagnostics={visibleDiagnostics}
        updateVisibleGraphs={updateVisibleGraphs}
        updateVisibleVideos={updateVisibleVideos}
        updateVisiblePointClouds={updateVisiblePointClouds}
        updateVisibleStatus={updateVisibleStatus}
        updateVisibleDiagnostics={updateVisibleDiagnostics}
      />
      <div className="main-container">
        {isTreeOpen && <TfTree />}
        {isTerminalOpen && <TerminalComponent />}
        <TopicsTable
          onAddGraph={handleAddGraph}
          onAddVideo={handleAddVideo}
          onAddPointCloud={handleAddPointCloud}
          onAddStatus={handleAddStatus}
          onAddDiagnostic={handleAddDiagnostic}
          visibleGraphs={visibleGraphs}
          visibleVideos={visibleVideos}
          visiblePointClouds={visiblePointClouds}
          visibleStatus={visibleStatus}
          visibleDiagnostics={visibleDiagnostics}
          isTerminalOpen={isTerminalOpen}
          isTreeOpen={isTreeOpen}
          onToggleTerminal={toggleTerminal}
          onToggleTree={toggleTree}
        />
      </div>
      <RobotMonitorCard />
    </div>
  );
};

export default App;
