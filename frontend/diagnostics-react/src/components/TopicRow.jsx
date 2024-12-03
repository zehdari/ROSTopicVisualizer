import React from "react";
import { TOPIC_TYPES } from "../config/topicTypes";
const TopicRow = ({
  topic,
  onToggleDetails,
  onAddGraph,
  onAddVideo,
  onAddPointCloud,
  onAddStatus,
  onAddDiagnostic,
  isGraphVisible,
  isVideoVisible,
  isPointCloudVisible,
  isStatusVisible,
  isDiagnosticVisible,
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
        {TOPIC_TYPES[topic.type]?.isGraph && !isGraphVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddGraph(topic))}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {TOPIC_TYPES[topic.type]?.isVideo && !isVideoVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddVideo(topic.name))}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {TOPIC_TYPES[topic.type]?.isPointCloud && !isPointCloudVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddPointCloud(topic))}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {TOPIC_TYPES[topic.type]?.isStatus && !isStatusVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddStatus(topic))}
            className="add-graph-btn"
          >
            +
          </button>
        )}
        {TOPIC_TYPES[topic.type]?.isDiagnostic && !isDiagnosticVisible && (
          <button
            onClick={(e) => handleButtonClick(e, () => onAddDiagnostic(topic))}
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
