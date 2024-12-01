import React, { useRef } from "react";
import { FaTimes } from "react-icons/fa";
import { RefreshCcw } from "lucide-react";
import FastPointCloudViewer from "./PointCloudViewer";
import "../styles/PointCloudCard.css";

const PointCloudCard = ({ topic, onRemovePointCloud }) => {
  const resetCameraRef = useRef(null);

  const handleReset = (resetFn) => {
    resetCameraRef.current = resetFn;
  };

  return (
    <div className="graph-card pointcloud-card">
      <div className="graph-card-buttons">
        <button
          className="settings-graph-btn"
          title="Reset Camera"
          onClick={() => {
            if (resetCameraRef.current) {
              resetCameraRef.current();
            }
          }}
        >
          <RefreshCcw size={16} className="refresh-icon" />
        </button>
        <button
          onClick={() => onRemovePointCloud(topic)}
          className="remove-graph-btn"
          title="Remove Point Cloud"
        >
          <FaTimes />
        </button>
      </div>
      <div className="graph-header">
        <h3>{topic}</h3>
      </div>
      <div className="pointcloud-container">
        <FastPointCloudViewer
          topicName={topic}
          frameId="base_link"
          onReset={handleReset}
        />
      </div>
    </div>
  );
};

export default PointCloudCard;
