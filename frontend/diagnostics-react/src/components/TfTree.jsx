import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import Tree from "react-d3-tree";
import { Eye, EyeOff } from "lucide-react";
import "../styles/TfTree.css";

const TfTree = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [transforms, setTransforms] = useState({});
  const [isListening, setIsListening] = useState(false);
  const [isListeningComplete, setIsListeningComplete] = useState(false);
  const [treeDimensions, setTreeDimensions] = useState({
    width: 800,
    height: 600,
  });
  const [treeTranslate, setTreeTranslate] = useState({ x: 0, y: 0 });
  const [isTreeVisible, setIsTreeVisible] = useState(true);
  const treeContainerRef = useRef(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: "ws://192.168.1.19:9090",
    });

    setRos(rosInstance);

    rosInstance.on("error", (error) => {
      console.error("Error connecting to ROS:", error);
      setConnected(false);
    });

    rosInstance.on("connection", () => {
      console.log("Connected to ROS!");
      setConnected(true);
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

  // Recalculate translate when tree dimensions change
  useEffect(() => {
    if (treeContainerRef.current) {
      const dimensions = treeContainerRef.current.getBoundingClientRect();
      setTreeTranslate({
        x: dimensions.width / 2,
        y: dimensions.height / 16,
      });
    }
  }, [treeDimensions]);

  // Recalculate dimensions and translate whenever the button is pressed
  const startListening = () => {
    setIsListening(true);
    setIsListeningComplete(false);
    setTransforms({});
    setIsTreeVisible(true);

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

          // Prevent duplicates
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

  // Toggle tree visibility
  const toggleTreeVisibility = () => {
    setIsTreeVisible((prev) => !prev);
  };

  return (
    <div className="tree-container" ref={treeContainerRef}>
      <h1 className="tree-title">TF Tree</h1>
      <div className="buttons-row">
        <button
          onClick={startListening}
          disabled={isListening}
          className="start-listening-btn"
        >
          {isListening ? "Listening..." : "Start Listening for 5 Seconds"}
        </button>

        {/* Show/Hide tree visibility button */}
        {isListeningComplete && transforms && (
          <button
            onClick={toggleTreeVisibility}
            className="toggle-tree-btn"
            aria-label="Toggle tree visibility"
          >
            {isTreeVisible ? (
              <EyeOff size={24} /> // Use EyeOff icon when the tree is visible
            ) : (
              <Eye size={24} /> // Use Eye icon when the tree is hidden
            )}
          </button>
        )}
      </div>

      {isListeningComplete && transforms && isTreeVisible && (
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
