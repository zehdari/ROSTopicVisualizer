import React, { useState, useEffect } from "react";
import BaseCard from "./BaseCard";
import { useRosTopic } from "../utils/useRosTopic";
import "../styles/StatusCard.css";

const StatusCard = ({ topicConfig, onRemoveStatus }) => {
  const [messageData, setMessageData] = useState(null);
  const [frequency, setFrequency] = useState(0);
  const message = useRosTopic(topicConfig.name, topicConfig.type);
  const lastMessageTimeRef = React.useRef(Date.now());

  useEffect(() => {
    const intervalId = setInterval(() => {
      const currentTime = Date.now();
      if (currentTime - lastMessageTimeRef.current > 5000) {
        setFrequency(0);
      }
    }, 1000);

    return () => clearInterval(intervalId);
  }, []);

  useEffect(() => {
    if (message) {
      const currentTime = Date.now();
      const timeDiff = currentTime - lastMessageTimeRef.current;
      lastMessageTimeRef.current = currentTime;

      if (timeDiff > 0) {
        setFrequency((1000 / timeDiff).toFixed(2));
      }

      setMessageData(topicConfig.parser(message));
    }
  }, [message, topicConfig]);

  const renderValue = (value) => {
    if (Array.isArray(value)) {
      return (
        <div className="pl-4">
          {value.map((item, index) => (
            <div key={index} className="mb-2">
              {renderMessageContent(item)}
            </div>
          ))}
        </div>
      );
    }

    if (typeof value === "object" && value !== null) {
      return renderMessageContent(value);
    }

    return <span className="text-gray-700">{String(value)}</span>;
  };

  const renderMessageContent = (content) => {
    if (!content) return null;

    return (
      <div className="grid grid-cols-1 gap-1">
        {Object.entries(content).map(([key, value]) => (
          <div key={key} className="flex items-start gap-2">
            <span className="font-medium min-w-24">{key}:</span>
            {renderValue(value)}
          </div>
        ))}
      </div>
    );
  };

  return (
    <BaseCard
      title={topicConfig.name}
      onRemove={() => onRemoveStatus(topicConfig.name)}
      showFrequency={true}
      frequency={frequency}
      className="status-card"
    >
      <div className="p-4 overflow-auto max-h-96">
        {messageData ? (
          renderMessageContent(messageData)
        ) : (
          <p className="text-gray-500">Waiting for messages...</p>
        )}
      </div>
    </BaseCard>
  );
};

export default StatusCard;
