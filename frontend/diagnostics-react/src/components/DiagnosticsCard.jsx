import React, { useState, useEffect } from "react";
import { CheckCircle, AlertTriangle, XCircle, HelpCircle } from "lucide-react";
import BaseCard from "./BaseCard";
import { useRosTopic } from "../utils/useRosTopic";
import "../styles/DiagnosticsCard.css";

const DiagnosticCard = ({ topicConfig, onRemoveStatus }) => {
  const [messageData, setMessageData] = useState(null);
  const [frequency, setFrequency] = useState(0);
  const [expandedItems, setExpandedItems] = useState({});
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

  const getLevelInfo = (level) => {
    const levels = {
      0: { text: "OK", class: "text-green-500", icon: CheckCircle },
      1: { text: "WARN", class: "text-yellow-500", icon: AlertTriangle },
      2: { text: "ERROR", class: "text-red-500", icon: XCircle },
      3: { text: "STALE", class: "text-blue-500", icon: HelpCircle },
    };
    return levels[level] || levels[3];
  };

  const toggleExpand = (id) => {
    setExpandedItems((prev) => ({
      ...prev,
      [id]: !prev[id],
    }));
  };

  const renderDiagnosticStatus = (status, isSingleItem) => {
    const levelInfo = getLevelInfo(status.level);
    const Icon = levelInfo.icon;
    const isExpanded = isSingleItem || expandedItems[status.hardware_id];

    return (
      <div
        className={`diagnostic-entry ${isSingleItem ? "single-item" : ""}`}
        style={{
          border: isSingleItem ? "none" : undefined,
          padding: isSingleItem ? "0.25rem" : "0.5rem",
        }}
      >
        <div
          className="diagnostic-header"
          onClick={() => !isSingleItem && toggleExpand(status.hardware_id)}
        >
          <div className={`diagnostic-level ${levelInfo.class}`}>
            <Icon size={20} />
            <span className="level-text">{levelInfo.text}</span>
          </div>
          <div className="diagnostic-title">
            <h3>{status.name}</h3>
            <span className="hardware-id">ID: {status.hardware_id}</span>
          </div>
        </div>
        {isExpanded && (
          <>
            <div className="diagnostic-message">{status.message}</div>
            {status.values && status.values.length > 0 && (
              <div className="diagnostic-values">
                <table>
                  <thead>
                    <tr>
                      <th>Key</th>
                      <th>Value</th>
                    </tr>
                  </thead>
                  <tbody>
                    {status.values.map((value, index) => (
                      <tr key={index}>
                        <td>{value.key}</td>
                        <td>{value.value}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
          </>
        )}
      </div>
    );
  };

  const getSeverityOrder = (level) => {
    const order = {
      2: 0, // ERROR (highest priority)
      1: 1, // WARN
      3: 2, // STALE
      0: 3, // OK (lowest priority)
    };
    return order[level] ?? 4;
  };

  const sortedStatusArray = messageData
    ? Object.values(messageData).sort(
        (a, b) => getSeverityOrder(a.level) - getSeverityOrder(b.level)
      )
    : [];

  const isSingleItem = sortedStatusArray.length === 1;

  return (
    <BaseCard
      title={topicConfig.name}
      onRemove={() => onRemoveStatus(topicConfig.name)}
      showFrequency={true}
      frequency={frequency}
      className="diagnostic-card"
    >
      <div className="diagnostic-container">
        {messageData ? (
          <div className="diagnostic-list">
            {sortedStatusArray.map((status, index) => (
              <div key={index} className="diagnostic-item">
                {renderDiagnosticStatus(status, isSingleItem)}
              </div>
            ))}
          </div>
        ) : (
          <p className="waiting-message">Waiting for diagnostics...</p>
        )}
      </div>
    </BaseCard>
  );
};

export default DiagnosticCard;
