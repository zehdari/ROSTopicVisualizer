import React, { useRef } from "react";
import BaseCard from "./BaseCard";
import FastPointCloudViewer from "./PointCloudViewer";

const PointCloudCard = ({ topic, onRemovePointCloud }) => {
  const resetCameraRef = useRef(null);

  const handleReset = (resetFn) => {
    resetCameraRef.current = resetFn;
  };

  return (
    <BaseCard
      title={topic}
      onRemove={() => onRemovePointCloud(topic)}
      onRefresh={() => resetCameraRef.current?.()}
      showRefresh={true}
      className="pointcloud-card"
    >
      <div className="pointcloud-container">
        <FastPointCloudViewer
          topicName={topic}
          frameId="base_link"
          onReset={handleReset}
        />
      </div>
    </BaseCard>
  );
};

export default PointCloudCard;
