import React, { useState, useEffect } from "react";
import "../styles/VideoCard.css";
import { FaTimes } from "react-icons/fa";
import { RefreshCcw } from "lucide-react";
import { NETWORK_CONFIG } from "../config/networkConfig";

const VideoCard = ({ topic, port, onRemoveVideo, headerProps }) => {
  const [streamStarted, setStreamStarted] = useState(false);
  const [status, setStatus] = useState("");
  const [refreshTrigger, setRefreshTrigger] = useState(0);
  const [isStartingStream, setIsStartingStream] = useState(false);

  useEffect(() => {
    if (!streamStarted && !isStartingStream) {
      startStream();
    }
    return () => {
      stopStream();
    };
  }, [topic, port]);

  useEffect(() => {
    if (streamStarted) {
      const timer = setTimeout(() => {
        setRefreshTrigger((prev) => prev + 1);
      }, 1000);
      return () => clearTimeout(timer);
    }
  }, [streamStarted]);

  const startStream = async () => {
    if (isStartingStream) return;
    try {
      setIsStartingStream(true);
      setStatus("Starting video stream...");
      const response = await fetch(
        `${NETWORK_CONFIG.FLASK_SERVER_URL}/start-video-server`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ topic, port }),
        }
      );
      const data = await response.json();
      if (response.ok) {
        setStreamStarted(true);
        setStatus("");
      } else {
        setStatus(data.error || "Failed to start stream");
        setStreamStarted(false);
      }
    } catch (error) {
      setStatus(`Error: ${error.message}`);
      setStreamStarted(false);
    } finally {
      setIsStartingStream(false);
    }
  };

  const stopStream = async () => {
    if (!streamStarted) return;
    try {
      await fetch(`${NETWORK_CONFIG.FLASK_SERVER_URL}/stop-video-server`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ topic }),
      });
    } catch (error) {
      console.error("Error stopping stream:", error);
    }
  };

  const handleRefresh = async () => {
    await stopStream();
    setStreamStarted(false);
    startStream();
  };

  const getImageSrc = () => {
    return streamStarted
      ? `${
          NETWORK_CONFIG.VIDEO_STREAM_URL
        }/stream?topic=${topic}&t=${Date.now()}`
      : "";
  };

  return (
    <div className="video-card graph-card">
      <div className="video-card-buttons graph-card-buttons">
        <button
          className="settings-video-btn settings-graph-btn"
          title="Refresh Stream"
          onClick={handleRefresh}
          disabled={isStartingStream}
        >
          <RefreshCcw size={16} className="refresh-icon" />
        </button>
        <button
          onClick={() => onRemoveVideo(topic)}
          className="remove-video-btn remove-graph-btn"
          title="Remove Video"
        >
          <FaTimes />
        </button>
      </div>
      <div {...headerProps}>
        {" "}
        <h3>{topic}</h3>
      </div>
      <p>{status}</p>
      {streamStarted && (
        <div className="video-container">
          <img
            key={`${refreshTrigger}-${topic}-${port}`}
            src={getImageSrc()}
            alt="Live Feed"
            className="video-stream"
          />
        </div>
      )}
    </div>
  );
};

export default VideoCard;
