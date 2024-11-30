import React, { useState, useEffect } from "react";

const ImageDisplay = () => {
  const [streamStarted, setStreamStarted] = useState(false);
  const [status, setStatus] = useState("");
  const [topic, setTopic] = useState("/video_topic");
  const port = 9091;
  const [refreshTrigger, setRefreshTrigger] = useState(0);

  // Auto-refresh when stream starts
  useEffect(() => {
    if (streamStarted) {
      // Trigger refresh
      const timer = setTimeout(() => {
        setRefreshTrigger((prev) => prev + 1);
      }, 300); // Small delay to ensure stream is fully initialized
      return () => clearTimeout(timer);
    }
  }, [streamStarted]);

  const startStream = async () => {
    try {
      setStreamStarted(false);
      setStatus("Starting video stream...");

      const response = await fetch("http://localhost:5001/start-video-server", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ topic, port }),
      });

      const data = await response.json();
      console.log(response);
      if (response.ok) {
        setRefreshTrigger((prev) => prev + 1);
        setStreamStarted(true);
        setStatus(`Stream started for topic: ${topic}`);
        // Additional trigger to ensure immediate render
      } else {
        setStatus(data.error || "Failed to start stream");
      }
    } catch (error) {
      setStatus(`Error: ${error.message}`);
    }
  };

  const getImageSrc = () => {
    return streamStarted
      ? `http://localhost:${port}/stream?topic=${topic}&t=${Date.now()}`
      : "";
  };

  return (
    <div>
      <h2>Video Stream</h2>
      <div>
        <label>
          Topic:
          <input
            type="text"
            value={topic}
            onChange={(e) => setTopic(e.target.value)}
          />
        </label>
      </div>
      <button onClick={startStream}>
        {streamStarted ? "Restart Stream" : "Start Stream"}
      </button>
      <p>{status}</p>
      {streamStarted && (
        <div>
          <img
            key={`${refreshTrigger}-${topic}-${port}`}
            src={getImageSrc()}
            alt="Live Feed"
            style={{ width: "100%", height: "auto" }}
          />
        </div>
      )}
    </div>
  );
};

export default ImageDisplay;
