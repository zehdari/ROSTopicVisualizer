import React from "react";
import RealtimeGraph from "./RealtimeGraph";

const App = () => {
  return (
    <div style={{ padding: "20px", fontFamily: "Arial, sans-serif" }}>
      <h1>Robot Diagnostics Dashboard</h1>
      <p>Monitoring real-time data from ROS 2 topics</p>
      <RealtimeGraph />
    </div>
  );
};

export default App;
