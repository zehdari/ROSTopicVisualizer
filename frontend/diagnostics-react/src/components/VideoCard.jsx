// VideoCard.jsx
import React, { useState, useEffect } from "react";
import BaseCard from "./BaseCard";
import { NETWORK_CONFIG } from "../config/networkConfig";

const VideoCard = ({ topic, port, onRemoveVideo }) => {
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
    } catch (error) {}
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
    <BaseCard
      title={topic}
      onRemove={() => onRemoveVideo(topic)}
      onRefresh={handleRefresh}
      showRefresh={true}
      className="video-card"
      status={status}
    >
      <div className="video-content-wrapper">
        {streamStarted && (
          <img
            key={`${refreshTrigger}-${topic}-${port}`}
            src={getImageSrc()}
            alt="Live Feed"
            className="video-stream"
          />
        )}
      </div>
    </BaseCard>
  );
};

export default VideoCard;
