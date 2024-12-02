// TopicRow.jsx
import React from "react";

const TopicRow = ({
  topic,
  isConfigured,
  onToggleDetails,
  onAddGraph,
  onAddVideo,
  onAddPointCloud,
  isVisible,
  isVideoVisible,
  isPointCloudVisible,
}) => {
  return (
    <tr
      className="topic-row hidden-graph-item"
      onClick={() => onToggleDetails(topic.name)}
    >
      <td className="topic-name">{topic.name}</td>
      <td>{topic.type}</td>
      <td className="topic-actions">
        {isConfigured &&
          topic.type !== "sensor_msgs/msg/PointCloud2" &&
          !isVisible && (
            <button
              onClick={(e) => {
                e.stopPropagation();
                onAddGraph(topic);
              }}
              className="add-graph-btn"
            >
              +
            </button>
          )}
        {topic.type === "sensor_msgs/msg/Image" && !isVideoVisible && (
          <button
            onClick={(e) => {
              e.stopPropagation();
              onAddVideo(topic.name);
            }}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {topic.type === "sensor_msgs/msg/PointCloud2" &&
          !isPointCloudVisible && (
            <button
              onClick={(e) => {
                e.stopPropagation();
                onAddPointCloud(topic);
              }}
              className="add-graph-btn"
            >
              +
            </button>
          )}
      </td>
    </tr>
  );
};

export default TopicRow;
