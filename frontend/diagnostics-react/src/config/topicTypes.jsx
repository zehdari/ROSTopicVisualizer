// topicTypes.jsx
export const TOPIC_TYPES = {
  Twist: {
    package: "geometry_msgs",
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
      { key: "linearX", name: "Linear X", stroke: "#8884d8" },
      { key: "linearY", name: "Linear Y", stroke: "#82ca9d" },
      { key: "linearZ", name: "Linear Z", stroke: "#ffc658" },
      { key: "angularX", name: "Angular X", stroke: "#d84d8c" },
      { key: "angularY", name: "Angular Y", stroke: "#4d8cd8" },
      { key: "angularZ", name: "Angular Z", stroke: "#8c4dd8" },
    ],
  },
  Bool: {
    package: "std_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data ? 1 : 0,
    }),
    graphKeys: [{ key: "value", name: "Enabled", stroke: "#8884d8" }],
  },
  Float64: {
    package: "std_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [{ key: "value", name: "Data", stroke: "#82ca9d" }],
  },
  Odometry: {
    package: "nav_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      positionX: message.pose?.pose?.position?.x || 0,
      positionY: message.pose?.pose?.position?.y || 0,
      positionZ: message.pose?.pose?.position?.z || 0,
    }),
    graphKeys: [
      { key: "positionX", name: "Position X", stroke: "#8884d8" },
      { key: "positionY", name: "Position Y", stroke: "#82ca9d" },
      { key: "positionZ", name: "Position Z", stroke: "#ffc658" },
    ],
  },
  WrenchStamped: {
    package: "geometry_msgs",
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
      { key: "forceX", name: "Force X", stroke: "#8884d8" },
      { key: "forceY", name: "Force Y", stroke: "#82ca9d" },
      { key: "forceZ", name: "Force Z", stroke: "#ffc658" },
      { key: "torqueX", name: "Torque X", stroke: "#d84d8c" },
      { key: "torqueY", name: "Torque Y", stroke: "#4d8cd8" },
      { key: "torqueZ", name: "Torque Z", stroke: "#8c4dd8" },
    ],
  },
  Float32MultiArray: {
    package: "std_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      data: message.data || [],
    }),
    graphKeys: [{ key: "data", name: "Array Data", stroke: "#8884d8" }],
  },
  TwistWithCovarianceStamped: {
    package: "geometry_msgs",
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
      { key: "linearX", name: "Linear X", stroke: "#8884d8" },
      { key: "linearY", name: "Linear Y", stroke: "#82ca9d" },
      { key: "linearZ", name: "Linear Z", stroke: "#ffc658" },
      { key: "angularX", name: "Angular X", stroke: "#d84d8c" },
      { key: "angularY", name: "Angular Y", stroke: "#4d8cd8" },
      { key: "angularZ", name: "Angular Z", stroke: "#8c4dd8" },
      { key: "covariance", name: "Covariance", stroke: "#ff6347" },
    ],
  },
};
