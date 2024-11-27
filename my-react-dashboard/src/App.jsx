import React from "react";
import RealtimeGraph from "./RealtimeGraph";
import "./App.css";

const App = () => {
  return (
    <div className="app-container">
      <h1 className="app-header">Talos Diagnostics</h1>
      <RealtimeGraph />
    </div>
  );
};

export default App;
