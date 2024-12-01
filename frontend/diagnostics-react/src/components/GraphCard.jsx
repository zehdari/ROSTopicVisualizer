import React, { useEffect, useState, useRef } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  Legend,
  ResponsiveContainer,
} from "recharts";
import { useRosTopic } from "../utils/useRosTopic";
import { FaCog, FaSave, FaTimes } from "react-icons/fa";

const frequencyWindow = 5;
const frequencyTimeout = 5000;

const getCSSColorVariables = () => {
  const styles = getComputedStyle(document.documentElement);
  const colorVars =
    Array.from(document.styleSheets)
      .flatMap((sheet) => {
        try {
          return Array.from(sheet.cssRules);
        } catch {
          return [];
        }
      })
      .find((rule) => rule.selectorText === ":root")?.style || [];

  return Array.from(colorVars)
    .filter((prop) => prop.startsWith("--color-"))
    .map((prop) => `var(${prop})`);
};

const generateArrayGraphKeys = (length) => {
  const colors = getCSSColorVariables();
  return Array.from({ length }, (_, index) => ({
    key: `value${index}`,
    name: `Value ${index + 1}`,
    stroke: colors[index % colors.length],
  }));
};

const isMultiArrayType = (topicConfig) => {
  return (
    topicConfig.isDynamicKeys === true && topicConfig.graphKeys.length === 0
  );
};

const GraphCard = ({ topicConfig, onRemoveGraph }) => {
  const [data, setData] = useState([]);
  const [graphKeys, setGraphKeys] = useState(topicConfig.graphKeys || []);
  const [frequency, setFrequency] = useState(0);
  const [activeLines, setActiveLines] = useState(
    (topicConfig.graphKeys || []).reduce((acc, key) => {
      acc[key.key] = true;
      return acc;
    }, {})
  );
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [maxDataPoints, setMaxDataPoints] = useState(50);
  const [inputMaxDataPoints, setInputMaxDataPoints] = useState(maxDataPoints);

  const message = useRosTopic(topicConfig.name, topicConfig.type);
  const timestampsRef = useRef([]);
  const lastMessageTimeRef = useRef(Date.now());
  const isInitializedRef = useRef(false);

  // Initialize dynamic graph keys when first message arrives
  useEffect(() => {
    if (message && topicConfig.isDynamicKeys && !isInitializedRef.current) {
      if (topicConfig.type === "std_msgs/msg/UInt8MultiArray") {
        const binaryString = atob(message.data);
        const length = binaryString.length;

        const newGraphKeys = generateArrayGraphKeys(length);

        setGraphKeys(newGraphKeys);

        const newActiveLines = newGraphKeys.reduce((acc, key) => {
          acc[key.key] = true;
          return acc;
        }, {});
        setActiveLines(newActiveLines);
        isInitializedRef.current = true;
      } else if (isMultiArrayType(topicConfig) && Array.isArray(message.data)) {
        // Handle other multi-array types as before
        const newGraphKeys = generateArrayGraphKeys(message.data.length);
        setGraphKeys(newGraphKeys);
        const newActiveLines = newGraphKeys.reduce((acc, key) => {
          acc[key.key] = true;
          return acc;
        }, {});
        setActiveLines(newActiveLines);
        isInitializedRef.current = true;
      }
    }
  }, [message, topicConfig.isDynamicKeys, topicConfig.type]);

  useEffect(() => {
    const intervalId = setInterval(() => {
      const currentTime = Date.now();
      if (currentTime - lastMessageTimeRef.current > frequencyTimeout) {
        setFrequency(0);
      }
    }, 1000);

    return () => clearInterval(intervalId);
  }, []);

  useEffect(() => {
    const currentTime = Date.now();

    if (message) {
      lastMessageTimeRef.current = currentTime;
      timestampsRef.current = [...timestampsRef.current, currentTime].slice(
        -frequencyWindow
      );

      if (timestampsRef.current.length >= 2) {
        const timeDifferences = timestampsRef.current
          .slice(1)
          .map((time, index) => time - timestampsRef.current[index]);
        const averageInterval =
          timeDifferences.reduce((a, b) => a + b, 0) / timeDifferences.length;
        const calculatedFrequency = 1000 / averageInterval;
        setFrequency(calculatedFrequency.toFixed(2));
      }

      const dataEntry = topicConfig.parser(message);
      setData((prevData) => [...prevData.slice(-maxDataPoints + 1), dataEntry]);
    }
  }, [message, topicConfig, maxDataPoints]);

  const handleLegendClick = (dataKey) => {
    setActiveLines((prevActiveLines) => ({
      ...prevActiveLines,
      [dataKey]: !prevActiveLines[dataKey],
    }));
  };

  const renderCustomLegend = (props) => {
    const { payload } = props;

    return (
      <div className="custom-legend">
        {payload.map((entry) => {
          const { dataKey, color, value } = entry;
          const isActive = activeLines[dataKey];
          const style = {
            display: "inline-block",
            marginRight: 15,
            cursor: "pointer",
            textDecoration: isActive ? "none" : "line-through",
            color: isActive ? color : "#AAA",
            fontWeight: isActive ? "normal" : "bold",
          };
          return (
            <span
              key={dataKey}
              style={style}
              onClick={() => handleLegendClick(dataKey)}
            >
              {value}
            </span>
          );
        })}
      </div>
    );
  };

  const customTooltip = ({ payload, label }) => {
    if (!payload || payload.length === 0) return null;

    return (
      <div className="custom-tooltip">
        <p className="tooltip-time">{`Time: ${label}`}</p>
        {payload.map((entry) => {
          const { dataKey, value, stroke, name } = entry;
          const roundedValue = value ? value.toFixed(6) : "N/A";
          return (
            <p
              key={dataKey}
              className="tooltip-entry"
              style={{ color: stroke }}
            >
              <strong>{name}:</strong> {roundedValue}
            </p>
          );
        })}
      </div>
    );
  };

  return (
    <div className="graph-card">
      <div className="graph-card-buttons">
        <button
          onClick={() => setIsSettingsOpen(true)}
          className="settings-graph-btn"
          title="Settings"
        >
          <FaCog />
        </button>
        <button
          onClick={() => onRemoveGraph(topicConfig.name)}
          className="remove-graph-btn"
          title="Remove Graph"
        >
          <FaTimes />
        </button>
      </div>

      <div className="graph-header">
        <h3>{topicConfig.name}</h3>
        <div className="frequency-display">
          {frequency > 0 ? `${frequency} Hz` : "0 Hz"}
        </div>
      </div>

      <div style={{ width: "100%", maxWidth: "450px", height: "300px" }}>
        <ResponsiveContainer>
          <LineChart
            data={data}
            margin={{ top: 20, right: 20, bottom: 20, left: 20 }}
          >
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="time" />
            <YAxis />
            <Tooltip content={customTooltip} />
            <Legend content={renderCustomLegend} />
            {graphKeys.map((keyConfig) => (
              <Line
                key={`${topicConfig.name}-${keyConfig.key}`}
                type="monotone"
                dataKey={keyConfig.key}
                stroke={keyConfig.stroke}
                name={keyConfig.name}
                dot={false}
                isAnimationActive={false}
                hide={!activeLines[keyConfig.key]}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>

      {isSettingsOpen && (
        <div className="settings-modal">
          <div className="settings-modal-content">
            <h4>Settings</h4>
            <div className="settings-modal-field">
              <label htmlFor="maxDataPoints">Max Data Points: </label>
              <input
                type="number"
                id="maxDataPoints"
                value={inputMaxDataPoints}
                onChange={(e) => setInputMaxDataPoints(e.target.value)}
                min="1"
              />
            </div>
            <div className="settings-modal-buttons">
              <button
                onClick={() => {
                  setInputMaxDataPoints(maxDataPoints);
                  setIsSettingsOpen(false);
                }}
                className="settings-modal-btn"
              >
                <FaTimes /> Cancel
              </button>
              <button
                onClick={() => {
                  const parsedValue = parseInt(inputMaxDataPoints, 10);
                  if (!isNaN(parsedValue) && parsedValue > 0) {
                    setMaxDataPoints(parsedValue);
                    setIsSettingsOpen(false);
                  }
                }}
                className="settings-modal-btn"
              >
                <FaSave /> Save
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default GraphCard;
