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
  const handleButtonClick = (e, action) => {
    e.preventDefault(); // Prevent default touch behavior
    e.stopPropagation(); // Stop event from bubbling up
    action();
  };

  return (
    <tr
      className="topic-row hidden-graph-item"
      onClick={(e) => {
        e.preventDefault(); // Prevent default touch behavior
        onToggleDetails(topic.name);
      }}
    >
      <td className="topic-name">{topic.name}</td>
      <td>{topic.type}</td>
      <td className="topic-actions">
        {isConfigured &&
          topic.type !== "sensor_msgs/msg/PointCloud2" &&
          !isVisible && (
            <button
              onClick={(e) => handleButtonClick(e, () => onAddGraph(topic))}
              className="add-graph-btn"
            >
              +
            </button>
          )}
        {topic.type === "sensor_msgs/msg/Image" && !isVideoVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddVideo(topic.name))}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {topic.type === "sensor_msgs/msg/PointCloud2" &&
          !isPointCloudVisible && (
            <button
              onClick={(e) =>
                handleButtonClick(e, () => onAddPointCloud(topic))
              }
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
