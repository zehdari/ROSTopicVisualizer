import React from "react";

const TopicDetails = ({ topic, topicDetails }) => {
  const { frequency, latestMessage } = topicDetails || {};

  return (
    <tr className="topic-details-row">
      <td colSpan="3">
        <div className="graph-card topic-details-content">
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
