import React, { useEffect, useState, useRef } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  Legend,
} from "recharts";
import { useRosTopic } from "./useRosTopic";

const maxDataPoints = 50;
const frequencyWindow = 5;
const frequencyTimeout = 5000;

const GraphCard = ({ topicConfig, onRemoveGraph }) => {
  const [data, setData] = useState([]);
  const [frequency, setFrequency] = useState(0);
  const [activeLines, setActiveLines] = useState(
    topicConfig.graphKeys.reduce((acc, key) => {
      acc[key.key] = true;
      return acc;
    }, {})
  );
  const [isHovering, setIsHovering] = useState(false);

  const message = useRosTopic(
    topicConfig.name,
    `${topicConfig.package}/msg/${topicConfig.type}`
  );

  const timestampsRef = useRef([]);
  const lastMessageTimeRef = useRef(Date.now());

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
  }, [message, topicConfig]);

  useEffect(() => {
    const intervalId = setInterval(() => {
      const currentTime = Date.now();
      if (currentTime - lastMessageTimeRef.current > frequencyTimeout) {
        setFrequency(0);
      }
    }, 1000);

    return () => clearInterval(intervalId);
  }, []);

  const handleLegendClick = (dataKey) => {
    setActiveLines((prevActiveLines) => {
      const newState = {
        ...prevActiveLines,
        [dataKey]: !prevActiveLines[dataKey],
      };
      return newState;
    });
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

  const memoizedRenderLegend = React.useCallback(renderCustomLegend, [
    activeLines,
    topicConfig,
  ]);

  // Custom Tooltip formatter to round values
  // Custom Tooltip formatter to round values and match graph card styles
  const customTooltip = ({ payload, label }) => {
    if (!payload || payload.length === 0) return null;

    return (
      <div className="custom-tooltip">
        <p className="tooltip-time">{`Time: ${label}`}</p>
        {payload.map((entry) => {
          const { dataKey, value, stroke, name } = entry;
          // Round the value to 6 decimal places
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
    <div
      key={topicConfig.name}
      className="graph-card"
      onMouseEnter={() => setIsHovering(true)}
      onMouseLeave={() => setIsHovering(false)}
      style={{ position: "relative" }}
    >
      {isHovering && (
        <button
          onClick={() => onRemoveGraph(topicConfig.name)}
          className="remove-graph-btn"
          style={{
            position: "absolute",
            top: "5px",
            right: "5px",
          }}
        >
          ✕
        </button>
      )}
      <div className="graph-header">
        <h3>{topicConfig.name}</h3>
        <div className="frequency-display">
          {frequency > 0 ? `${frequency} Hz` : "0 Hz"}
        </div>
      </div>
      <LineChart
        width={500}
        height={300}
        data={data}
        margin={{ top: 20, right: 20, bottom: 20, left: 20 }}
      >
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="time" />
        <YAxis />
        <Tooltip content={customTooltip} />
        <Legend content={memoizedRenderLegend} />
        {topicConfig.graphKeys.map((keyConfig) => (
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
    </div>
  );

  return (
    <div
      key={topicConfig.name}
      className="graph-card"
      onMouseEnter={() => setIsHovering(true)}
      onMouseLeave={() => setIsHovering(false)}
      style={{ position: "relative" }}
    >
      {isHovering && (
        <button
          onClick={() => onRemoveGraph(topicConfig.name)}
          className="remove-graph-btn"
          style={{
            position: "absolute",
            top: "5px",
            right: "5px",
          }}
        >
          ✕
        </button>
      )}
      <div className="graph-header">
        <h3>{topicConfig.name}</h3>
        <div className="frequency-display">
          {frequency > 0 ? `${frequency} Hz` : "0 Hz"}
        </div>
      </div>
      <LineChart
        width={500}
        height={300}
        data={data}
        margin={{ top: 20, right: 20, bottom: 20, left: 20 }}
      >
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="time" />
        <YAxis />
        <Tooltip content={customTooltip} />
        <Legend content={memoizedRenderLegend} />
        {topicConfig.graphKeys.map((keyConfig) => (
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
    </div>
  );
};

export default GraphCard;
