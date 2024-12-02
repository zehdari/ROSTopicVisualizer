import React from "react";

const TopicDetails = ({ topic, topicDetails, onToggleDetails }) => {
  const { frequency, latestMessage } = topicDetails || {};

  const handleBackgroundClick = (e) => {
    // Only trigger if clicking the background elements
    if (
      e.target === e.currentTarget ||
      e.target.className === "topic-details-row" ||
      e.target.className === "graph-card topic-details-content"
    ) {
      onToggleDetails(topic.name);
    }
  };

  return (
    <tr className="topic-details-row" onClick={handleBackgroundClick}>
      <td colSpan="3" onClick={handleBackgroundClick}>
        <div
          className="graph-card topic-details-content"
          onClick={handleBackgroundClick}
        >
          <p>
            <strong>Type:</strong> {topic.type}
          </p>
          <p>
            <strong>Frequency:</strong>{" "}
            {frequency ? `${frequency} Hz` : "Calculating..."}
          </p>
          {latestMessage ? (
            <>
              <p>
                <strong>Latest Message:</strong>
              </p>
              <pre>{latestMessage}</pre>
            </>
          ) : (
            <p>Waiting for message...</p>
          )}
        </div>
      </td>
    </tr>
  );
};

export default TopicDetails;
