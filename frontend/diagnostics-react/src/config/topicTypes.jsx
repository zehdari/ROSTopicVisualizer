import "../styles/graphColors.css";

// Function to get CSS variables (color values) dynamically
const getCSSColor = (varName) => {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(varName)
    .trim();
};

const generateArrayGraphKeys = (dataArray) => {
  return dataArray.map((value, index) => ({
    key: `value${index}`,
    name: `Value ${index + 1}`,
    stroke: getCSSColor(`--color-${(index % 10) + 1}`), // Color cycling based on index
  }));
};

// Use CSS variable names to retrieve color values
export const TOPIC_TYPES = {
  "geometry_msgs/msg/Twist": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      linearX: message.linear?.x || 0,
      linearY: message.linear?.y || 0,
      linearZ: message.linear?.z || 0,
      angularX: message.angular?.x || 0,
      angularY: message.angular?.y || 0,
      angularZ: message.angular?.z || 0,
    }),
    graphKeys: [
      {
        key: "linearX",
        name: "Linear X",
        stroke: getCSSColor("--color-light-blue"),
      },
      {
        key: "linearY",
        name: "Linear Y",
        stroke: getCSSColor("--color-soft-green"),
      },
      {
        key: "linearZ",
        name: "Linear Z",
        stroke: getCSSColor("--color-light-yellow"),
      },
      {
        key: "angularX",
        name: "Angular X",
        stroke: getCSSColor("--color-soft-red"),
      },
      {
        key: "angularY",
        name: "Angular Y",
        stroke: getCSSColor("--color-light-sky-blue"),
      },
      {
        key: "angularZ",
        name: "Angular Z",
        stroke: getCSSColor("--color-purple"),
      },
    ],
  },
  "std_msgs/msg/Bool": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data ? 1 : 0,
    }),
    graphKeys: [
      {
        key: "value",
        name: "Enabled",
        stroke: getCSSColor("--color-light-blue"),
      },
    ],
  },
  "std_msgs/msg/Float64": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },
  "std_msgs/msg/Int32": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },
  "nav_msgs/msg/Odometry": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      positionX: message.pose?.pose?.position?.x || 0,
      positionY: message.pose?.pose?.position?.y || 0,
      positionZ: message.pose?.pose?.position?.z || 0,
    }),
    graphKeys: [
      {
        key: "positionX",
        name: "Position X",
        stroke: getCSSColor("--color-light-blue"),
      },
      {
        key: "positionY",
        name: "Position Y",
        stroke: getCSSColor("--color-soft-green"),
      },
      {
        key: "positionZ",
        name: "Position Z",
        stroke: getCSSColor("--color-light-yellow"),
      },
    ],
  },
  "geometry_msgs/msg/WrenchStamped": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      forceX: message.wrench?.force?.x || 0,
      forceY: message.wrench?.force?.y || 0,
      forceZ: message.wrench?.force?.z || 0,
      torqueX: message.wrench?.torque?.x || 0,
      torqueY: message.wrench?.torque?.y || 0,
      torqueZ: message.wrench?.torque?.z || 0,
    }),
    graphKeys: [
      {
        key: "forceX",
        name: "Force X",
        stroke: getCSSColor("--color-light-blue"),
      },
      {
        key: "forceY",
        name: "Force Y",
        stroke: getCSSColor("--color-soft-green"),
      },
      {
        key: "forceZ",
        name: "Force Z",
        stroke: getCSSColor("--color-light-yellow"),
      },
      {
        key: "torqueX",
        name: "Torque X",
        stroke: getCSSColor("--color-soft-red"),
      },
      {
        key: "torqueY",
        name: "Torque Y",
        stroke: getCSSColor("--color-light-sky-blue"),
      },
      {
        key: "torqueZ",
        name: "Torque Z",
        stroke: getCSSColor("--color-purple"),
      },
    ],
  },
  "geometry_msgs/msg/TwistWithCovarianceStamped": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      linearX: message.twist?.linear?.x || 0,
      linearY: message.twist?.linear?.y || 0,
      linearZ: message.twist?.linear?.z || 0,
      angularX: message.twist?.angular?.x || 0,
      angularY: message.twist?.angular?.y || 0,
      angularZ: message.twist?.angular?.z || 0,
      covariance: message.twist?.covariance || [],
    }),
    graphKeys: [
      {
        key: "linearX",
        name: "Linear X",
        stroke: getCSSColor("--color-light-blue"),
      },
      {
        key: "linearY",
        name: "Linear Y",
        stroke: getCSSColor("--color-soft-green"),
      },
      {
        key: "linearZ",
        name: "Linear Z",
        stroke: getCSSColor("--color-light-yellow"),
      },
      {
        key: "angularX",
        name: "Angular X",
        stroke: getCSSColor("--color-soft-red"),
      },
      {
        key: "angularY",
        name: "Angular Y",
        stroke: getCSSColor("--color-light-sky-blue"),
      },
      {
        key: "angularZ",
        name: "Angular Z",
        stroke: getCSSColor("--color-purple"),
      },
      {
        key: "covariance",
        name: "Covariance",
        stroke: getCSSColor("--color-tomato"),
      },
    ],
  },
  "sensor_msgs/msg/PointCloud2": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      // For plotting, we'll just show the point count
      pointCount: message.width * message.height || 0,
      dataSize: message.data?.length || 0,
    }),
    graphKeys: [
      {
        key: "pointCount",
        name: "Number of Points",
        stroke: getCSSColor("--color-light-blue"),
      },
      {
        key: "dataSize",
        name: "Data Size (bytes)",
        stroke: getCSSColor("--color-soft-green"),
      },
    ],
  },
};
