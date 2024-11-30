import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import Tree from "react-d3-tree";
import "../styles/TfTree.css";
import { NETWORK_CONFIG } from "../config/networkConfig";

const TfTree = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [transforms, setTransforms] = useState({});
  const [isListening, setIsListening] = useState(false);
  const [isListeningComplete, setIsListeningComplete] = useState(false);
  const [treeDimensions, setTreeDimensions] = useState({
    width: 0,
    height: 0,
  });
  const [treeTranslate, setTreeTranslate] = useState({ x: 0, y: 0 });
  const treeContainerRef = useRef(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: NETWORK_CONFIG.ROS_BRIDGE_URL,
    });

    setRos(rosInstance);

    rosInstance.on("error", (error) => {
      console.error("Error connecting to ROS:", error);
      setConnected(false);
    });

    rosInstance.on("connection", () => {
      console.log("Connected to ROS!");
      setConnected(true);
      // Start listening automatically when connected
      setIsListening(true);
    });

    rosInstance.on("close", () => {
      console.log("Connection to ROS closed.");
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

      const processTransforms = (message) => {
        console.log("Raw transforms:", message.transforms);

        const uniqueTransforms = new Set();

        message.transforms.forEach((tf) => {
          const parentFrame = tf.header.frame_id;
          const childFrame = tf.child_frame_id;

          const transformKey = `${parentFrame}->${childFrame}`;
          if (!uniqueTransforms.has(transformKey)) {
            uniqueTransforms.add(transformKey);
            buffer.push({ parentFrame, childFrame });
          }
        });
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
      }, 5000);

      return () => {
        if (tfListener) tfListener.unsubscribe();
      };
    }
  }, [ros, connected, isListening]);

  const buildTreeData = (transforms) => {
    console.log("Building tree from transforms:", transforms);

    const createTreeNode = (frameId) => {
      if (!transforms[frameId]) return null;

      const node = {
        name: frameId,
        children: [],
      };

      transforms[frameId].children.forEach((childFrame) => {
        const childNode = createTreeNode(childFrame);
        if (childNode) node.children.push(childNode);
      });

      return node;
    };

    return createTreeNode("world");
  };

  return (
    <div
      className={`tree-container ${isListeningComplete ? "with-tree" : ""}`}
      ref={treeContainerRef}
    >
      <div className="tree-header">
        <h1 className="tree-title">TF Tree</h1>
        {transforms && (
          <button
            onClick={startListening}
            disabled={isListening}
            className="start-listening-btn"
          >
            {isListening ? "Listening..." : "Refresh Tree"}
          </button>
        )}
      </div>

      {isListeningComplete && transforms && (
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
            separation={{
              siblings: 1.5,
              nonSiblings: 2.0,
            }}
            translate={treeTranslate}
          />
        </div>
      )}
    </div>
  );
};

export default TfTree;
