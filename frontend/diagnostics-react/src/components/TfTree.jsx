import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import Tree from "react-d3-tree";
import "../styles/TfTree.css";
import { NETWORK_CONFIG } from "../config/networkConfig";

const TfTree = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [transforms, setTransforms] = useState({});
  const [transformData, setTransformData] = useState({}); // Store detailed transform data
  const [isListening, setIsListening] = useState(false);
  const [isListeningComplete, setIsListeningComplete] = useState(false);
  const [treeDimensions, setTreeDimensions] = useState({
    width: 0,
    height: 0,
  });
  const [treeTranslate, setTreeTranslate] = useState({ x: 0, y: 0 });
  const [hasData, setHasData] = useState(false);
  const [tooltip, setTooltip] = useState({
    show: false,
    content: null,
    position: { x: 0, y: 0 },
  });
  const treeContainerRef = useRef(null);
  const tooltipRef = useRef(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: NETWORK_CONFIG.ROS_BRIDGE_URL,
    });

    setRos(rosInstance);

    rosInstance.on("error", (error) => {
      setConnected(false);
    });

    rosInstance.on("connection", () => {
      setConnected(true);
      setIsListening(true);
    });

    rosInstance.on("close", () => {
      setConnected(false);
    });

    return () => {
      if (rosInstance) {
        rosInstance.close();
      }
    };
  }, []);

  useEffect(() => {
    const updateDimensions = () => {
      setTreeDimensions({
        width: window.innerWidth - 50,
        height: window.innerHeight - 150,
      });
    };

    window.addEventListener("resize", updateDimensions);
    updateDimensions();

    return () => window.removeEventListener("resize", updateDimensions);
  }, []);

  useEffect(() => {
    if (treeContainerRef.current) {
      const dimensions = treeContainerRef.current.getBoundingClientRect();
      setTreeTranslate({
        x: dimensions.width / 2,
        y: dimensions.height / 16,
      });
    }
  }, [treeDimensions]);

  const startListening = () => {
    setIsListening(true);
    setIsListeningComplete(false);
    setTransforms({});
    setTransformData({});
    setHasData(false);
    setTooltip({ show: false, content: null, position: { x: 0, y: 0 } });

    if (treeContainerRef.current) {
      const dimensions = treeContainerRef.current.getBoundingClientRect();
      setTreeTranslate({
        x: dimensions.width / 2,
        y: dimensions.height / 16,
      });
    }
  };

  useEffect(() => {
    let tfListener = null;

    if (ros && connected && isListening) {
      const buffer = [];
      const transformDataBuffer = {};
      let receivedData = false;

      const processTransforms = (message) => {
        if (message.transforms && message.transforms.length > 0) {
          receivedData = true;
          const uniqueTransforms = new Set();

          message.transforms.forEach((tf) => {
            const parentFrame = tf.header.frame_id;
            const childFrame = tf.child_frame_id;
            const transformKey = `${parentFrame}->${childFrame}`;

            if (!uniqueTransforms.has(transformKey)) {
              uniqueTransforms.add(transformKey);
              buffer.push({ parentFrame, childFrame });

              // Store transform data for child frame
              transformDataBuffer[childFrame] = {
                translation: tf.transform.translation,
                rotation: tf.transform.rotation,
                timestamp: tf.header.stamp,
                parentFrame,
              };

              // Store transform data for parent frame if it doesn't exist
              if (!transformDataBuffer[parentFrame]) {
                transformDataBuffer[parentFrame] = {
                  translation: { x: 0, y: 0, z: 0 },
                  rotation: { x: 0, y: 0, z: 0, w: 1 },
                  timestamp: tf.header.stamp,
                  parentFrame: null, // root frame has no parent
                };
              }
            }
          });
        }
      };

      tfListener = new ROSLIB.Topic({
        ros: ros,
        name: "/tf",
        messageType: "tf2_msgs/TFMessage",
      });

      tfListener.subscribe(processTransforms);

      setTimeout(() => {
        setIsListening(false);
        setIsListeningComplete(true);
        tfListener.unsubscribe();
        setHasData(receivedData);

        const updatedTransforms = {};

        buffer.forEach(({ parentFrame, childFrame }) => {
          if (!updatedTransforms[parentFrame]) {
            updatedTransforms[parentFrame] = { children: [] };
          }
          if (!updatedTransforms[childFrame]) {
            updatedTransforms[childFrame] = { children: [] };
          }

          if (!updatedTransforms[parentFrame].children.includes(childFrame)) {
            updatedTransforms[parentFrame].children.push(childFrame);
          }
        });

        setTransforms(updatedTransforms);
        setTransformData(transformDataBuffer);
      }, 5000);

      return () => {
        if (tfListener) tfListener.unsubscribe();
      };
    }
  }, [ros, connected, isListening]);

  const buildTreeData = (transforms) => {
    const findRootFrame = () => {
      const allFrames = new Set(Object.keys(transforms));
      const childFrames = new Set();

      Object.values(transforms).forEach(({ children }) => {
        children.forEach((child) => childFrames.add(child));
      });

      for (const frame of allFrames) {
        if (!childFrames.has(frame)) {
          return frame;
        }
      }

      return Object.keys(transforms)[0];
    };

    const createTreeNode = (frameId) => {
      if (!frameId || !transforms[frameId]) return null;

      const node = {
        name: frameId,
        children: [],
        nodeSvgShape: {
          shape: "circle",
          shapeProps: {
            r: 10,
            fill: transformData[frameId] ? "var(--card-bg)" : "var(--hover-bg)",
          },
        },
      };

      transforms[frameId].children.forEach((childFrame) => {
        const childNode = createTreeNode(childFrame);
        if (childNode) node.children.push(childNode);
      });

      return node;
    };

    const rootFrame = findRootFrame();
    const rootNode = createTreeNode(rootFrame);
    return rootNode || { name: "No valid transform tree", children: [] };
  };

  const formatTime = (timestamp) => {
    if (!timestamp || (timestamp.sec === 0 && timestamp.nanosec === 0)) {
      return "N/A";
    }
    // Convert ROS 2 timestamp to milliseconds
    const milliseconds = timestamp.sec * 1000 + timestamp.nanosec / 1000000;
    return new Date(milliseconds).toLocaleTimeString();
  };

  const quaternionToEuler = (q) => {
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    const pitch =
      Math.abs(sinp) >= 1 ? Math.copySign(Math.PI / 2, sinp) : Math.asin(sinp);

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    // Convert to degrees
    return {
      roll: ((roll * 180) / Math.PI).toFixed(3),
      pitch: ((pitch * 180) / Math.PI).toFixed(3),
      yaw: ((yaw * 180) / Math.PI).toFixed(3),
    };
  };

  const handleNodeMouseEnter = (nodeData, event) => {
    const frameName = nodeData.data.name;
    const data = transformData[frameName];

    if (!data) {
      console.log("No transform data found for frame:", frameName);
      return;
    }

    const eulerAngles = quaternionToEuler(data.rotation);
    const formattedTime = formatTime(data.timestamp);

    const content = (
      <div className="custom-tooltip">
        <div className="tooltip-entry">
          <strong>Frame:</strong> {frameName}
        </div>
        <div className="tooltip-entry">
          <strong>Parent:</strong> {data.parentFrame}
        </div>
        <div className="tooltip-entry">
          <strong>Translation:</strong>
          <div>X: {data.translation.x.toFixed(3)}</div>
          <div>Y: {data.translation.y.toFixed(3)}</div>
          <div>Z: {data.translation.z.toFixed(3)}</div>
        </div>
        <div className="tooltip-entry">
          <strong>Rotation:</strong>
          <div className="rotation-columns">
            <div className="rotation-column">
              <div className="rotation-header">Euler</div>
              <div>R: {eulerAngles.roll}°</div>
              <div>P: {eulerAngles.pitch}°</div>
              <div>Y: {eulerAngles.yaw}°</div>
            </div>
            <div className="rotation-column">
              <div className="rotation-header">Quaternion</div>
              <div>X: {data.rotation.x.toFixed(3)}</div>
              <div>Y: {data.rotation.y.toFixed(3)}</div>
              <div>Z: {data.rotation.z.toFixed(3)}</div>
              <div>W: {data.rotation.w.toFixed(3)}</div>
            </div>
          </div>
        </div>
        <div className="tooltip-time">Time: {formattedTime}</div>
      </div>
    );

    const nodeElement = event.target.closest("g");
    const nodeRect = nodeElement.getBoundingClientRect();

    setTooltip({
      show: true,
      content,
      position: {
        x: nodeRect.x + nodeRect.width / 2,
        y: nodeRect.y + nodeRect.height / 2,
      },
    });
  };

  const handleNodeMouseLeave = () => {
    setTooltip({
      show: false,
      content: null,
      position: { x: 0, y: 0 },
    });
  };

  return (
    <div
      className={`tree-container ${
        isListeningComplete && hasData ? "with-tree" : ""
      }`}
      ref={treeContainerRef}
      onClick={(e) => {
        if (e.target === e.currentTarget) {
          setTooltip({ show: false, content: null, position: { x: 0, y: 0 } });
        }
      }}
    >
      <div className="tree-header">
        <h1 className="tree-title">TF Tree</h1>
        <button
          onClick={startListening}
          disabled={isListening}
          className="start-listening-btn"
        >
          {isListening ? "Listening..." : "Refresh Tree"}
        </button>
      </div>

      {isListeningComplete &&
        (hasData ? (
          <div className="tree-wrapper">
            <Tree
              data={buildTreeData(transforms)}
              orientation="vertical"
              width={treeDimensions.width}
              height={treeDimensions.height}
              margins={{ top: 20, bottom: 20, left: 10, right: 10 }}
              nodeSize={{ x: 200, y: 80 }}
              leafNodeClassName="node"
              rootNodeClassName="node"
              branchNodeClassName="node"
              svgClassName="custom-tree-svg"
              separation={{ siblings: 1.5, nonSiblings: 2.0 }}
              translate={treeTranslate}
              collapsible={false}
              onNodeMouseOver={handleNodeMouseEnter}
              onNodeMouseOut={handleNodeMouseLeave}
            />
            {tooltip.show && (
              <div
                ref={tooltipRef}
                className="tooltip-container"
                style={{
                  "--x": `${tooltip.position.x}px`,
                  "--y": `${tooltip.position.y}px`,
                }}
              >
                {tooltip.content}
              </div>
            )}
          </div>
        ) : (
          <div className="no-data-message">No transform data available.</div>
        ))}
    </div>
  );
};

export default TfTree;
