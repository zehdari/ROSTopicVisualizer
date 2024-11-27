// GraphCard.jsx
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

const maxDataPoints = 50; // Limit the number of points displayed on the graph
const frequencyWindow = 5; // Number of recent messages to calculate frequency

const GraphCard = ({ topicConfig, graphVisible, toggleVisibility }) => {
  const [data, setData] = useState([]);
  const [frequency, setFrequency] = useState(0);
  const [activeLines, setActiveLines] = useState(
    topicConfig.graphKeys.reduce((acc, key) => {
      acc[key.key] = true; // Initialize all lines as active
      return acc;
    }, {})
  );

  const message = useRosTopic(
    topicConfig.name,
    `${topicConfig.package}/msg/${topicConfig.type}`
  );

  const timestampsRef = useRef([]);

  useEffect(() => {
    if (message) {
      const currentTime = Date.now();
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

  return (
    <div
      key={topicConfig.name}
      className={`graph-card ${graphVisible ? "visible" : "hidden"}`}
    >
      <div
        className="graph-header"
        onClick={() => toggleVisibility(topicConfig.name)}
        style={{ cursor: "pointer" }}
      >
        <h3>{topicConfig.name}</h3>
        <div className="frequency-display">
          {frequency > 0 ? `${frequency} Hz` : "Calculating..."}
        </div>
      </div>
      {graphVisible && (
        <LineChart
          width={500}
          height={300}
          data={data}
          margin={{ top: 20, right: 20, bottom: 20, left: 20 }}
        >
          <CartesianGrid strokeDasharray="3 3" />
          <XAxis dataKey="time" />
          <YAxis />
          <Tooltip />
          <Legend content={memoizedRenderLegend} />
          {topicConfig.graphKeys.map((keyConfig) => (
            <Line
              key={keyConfig.key}
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
      )}
    </div>
  );
};

export default GraphCard;
