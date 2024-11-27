// topicsConfig.js
export const TOPICS_CONFIG = [
  {
    name: "/turtle1/cmd_vel",
    type: "Twist",
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
  {
    name: "/turtle_enabled",
    type: "Bool",
    package: "std_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data ? 1 : 0,
    }),
    graphKeys: [{ key: "value", name: "Enabled", stroke: "#8884d8" }],
  },
  {
    name: "/sin_wave",
    type: "Float64",
    package: "std_msgs",
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [{ key: "value", name: "Sin Wave", stroke: "#82ca9d" }],
  },
  {
    name: "/odom",
    type: "Odometry",
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
  // Add new topics here
];
