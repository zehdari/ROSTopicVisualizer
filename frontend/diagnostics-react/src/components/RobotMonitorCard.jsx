import React, { useState, useEffect } from "react";
import { useRosTopic } from "../utils/useRosTopic";
import "../styles/RobotMonitorCard.css";

const DiagnosticStatus = {
  OK: 0,
  WARN: 1,
  ERROR: 2,
};

const CurrentStatus = ({ status }) => {
  switch (status) {
    case DiagnosticStatus.WARN:
      return <div className="status-warning">WARNING</div>;
    case DiagnosticStatus.ERROR:
      return <div className="status-error">ERROR</div>;
    default:
      return <div className="status-ok">OK</div>;
  }
};

const DiagnosticMessage = ({ diagnostic }) => (
  <div className="diagnostic-message">
    <CurrentStatus status={diagnostic.level} />
    <div className="message-content">
      <div className="message-name">{diagnostic.name}</div>
      <div className="message-text">{diagnostic.message}</div>
      <div className="hardware-id">{diagnostic.hardware_id}</div>
    </div>
  </div>
);

const RobotMonitorCard = () => {
  const [combinedStatuses, setCombinedStatuses] = useState({});
  const message = useRosTopic(
    "/diagnostics",
    "diagnostic_msgs/msg/DiagnosticArray"
  );

  useEffect(() => {
    if (message?.status) {
      const newStatuses = { ...combinedStatuses };
      message.status.forEach((status) => {
        newStatuses[`${status.hardware_id}-${status.name}`] = status;
      });
      setCombinedStatuses(newStatuses);
    }
  }, [message]);

  const statusArray = Object.values(combinedStatuses);

  return (
    <div className="monitor-container">
      <div className="status-list">
        {statusArray.length > 0 ? (
          statusArray.map((diagnostic) => (
            <DiagnosticMessage
              key={`${diagnostic.hardware_id}-${diagnostic.name}`}
              diagnostic={diagnostic}
            />
          ))
        ) : (
          <div className="no-status">No hardware status available</div>
        )}
      </div>
    </div>
  );
};

export default RobotMonitorCard;
