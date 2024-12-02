import React from "react";
import { FaTimes } from "react-icons/fa";
import { RefreshCcw, Settings } from "lucide-react";
import "../styles/BaseCard.css";

const BaseCard = ({
  title,
  onRemove,
  onRefresh,
  onSettings,
  showRefresh = false,
  showSettings = false,
  children,
  className = "",
  headerContent,
  status,
  frequency,
  showFrequency = false,
}) => {
  return (
    <div className={`base-card ${className}`}>
      <div className="base-card-buttons">
        {showSettings && (
          <button
            className="base-card-btn"
            title="Settings"
            onClick={onSettings}
          >
            <Settings size={16} />
          </button>
        )}
        {showRefresh && (
          <button className="base-card-btn" title="Refresh" onClick={onRefresh}>
            <RefreshCcw size={16} className="refresh-icon" />
          </button>
        )}
        <button onClick={onRemove} className="base-card-btn" title="Remove">
          <FaTimes />
        </button>
      </div>
      <div className="base-card-header">
        <h3>{title}</h3>
        {showFrequency && (
          <div className="frequency-display">
            {frequency > 0 ? `${frequency} Hz` : "0 Hz"}
          </div>
        )}
        {headerContent}
      </div>
      {status && <p className="base-card-status">{status}</p>}
      <div className="base-card-content">{children}</div>
    </div>
  );
};

export default BaseCard;
