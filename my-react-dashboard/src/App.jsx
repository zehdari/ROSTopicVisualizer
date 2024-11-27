import React from "react";
import RealtimeGraph from "./RealtimeGraph";
import "./App.css";
import TopicsTable from "./TopicsTable";

const App = () => {
  return (
    <div className="app-container">
      <h1 className="app-header">Talos Diagnostics</h1>
      <RealtimeGraph />
      <TopicsTable />
    </div>
  );
};

export default App;
