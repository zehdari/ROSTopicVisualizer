import "../styles/graphColors.css";

// Function to get CSS variables (color values) dynamically
const getCSSColor = (varName) => {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(varName)
    .trim();
};

// Use CSS variable names to retrieve color values
export const TOPIC_TYPES = {
  // Primitive types
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

  "std_msgs/msg/Char": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data?.charCodeAt(0) || 0,
    }),
    graphKeys: [
      {
        key: "value",
        name: "ASCII Value",
        stroke: getCSSColor("--color-soft-green"),
      },
    ],
  },

  "std_msgs/msg/Int8": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/Int16": {
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

  "std_msgs/msg/Int64": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: Number(message.data) || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/UInt8": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/UInt16": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/UInt32": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/UInt64": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: Number(message.data) || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
    ],
  },

  "std_msgs/msg/Float32": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
      value: message.data || 0,
    }),
    graphKeys: [
      { key: "value", name: "Data", stroke: getCSSColor("--color-soft-green") },
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

  // Array Types

  "std_msgs/msg/Int8MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  "std_msgs/msg/Int16MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  "std_msgs/msg/Int32MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  "std_msgs/msg/Int64MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = Number(value) || 0;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  // This one is special
  "std_msgs/msg/UInt8MultiArray": {
    parser: (message) => {
      try {
        // Decode base64 string
        const binaryString = atob(message.data);
        const dataArray = new Uint8Array(binaryString.length);

        for (let i = 0; i < binaryString.length; i++) {
          dataArray[i] = binaryString.charCodeAt(i);
        }

        // Create the data point with timestamp
        const parsedData = {
          time: new Date().toLocaleTimeString(),
        };

        // Add each value to the data point
        Array.from(dataArray).forEach((value, index) => {
          parsedData[`value${index}`] = value;
        });

        return parsedData;
      } catch (error) {
        return {
          time: new Date().toLocaleTimeString(),
          value0: 0,
        };
      }
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  "std_msgs/msg/UInt16MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  "std_msgs/msg/UInt32MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },
  "std_msgs/msg/UInt64MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = Number(value) || 0;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },
  "std_msgs/msg/Float32MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },
  "std_msgs/msg/Float64MultiArray": {
    parser: (message) => {
      const dataArray = message.data || [];
      return {
        time: new Date().toLocaleTimeString(),
        ...dataArray.reduce((acc, value, index) => {
          acc[`value${index}`] = value;
          return acc;
        }, {}),
      };
    },
    graphKeys: [],
    isDynamicKeys: true,
  },

  // Other types
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
        stroke: getCSSColor("--color-sky-blue"),
      },
      {
        key: "angularZ",
        name: "Angular Z",
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
        stroke: getCSSColor("--color-sky-blue"),
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
        stroke: getCSSColor("--color-sky-blue"),
      },
      {
        key: "torqueZ",
        name: "Torque Z",
        stroke: getCSSColor("--color-purple"),
      },
    ],
  },

  "sensor_msgs/msg/PointCloud2": {
    parser: (message) => ({
      time: new Date().toLocaleTimeString(),
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
