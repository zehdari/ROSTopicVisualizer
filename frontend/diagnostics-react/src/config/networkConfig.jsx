// config/networkConfig.js
export const NETWORK_CONFIG = {
  HOST: "192.168.1.19",
  PORTS: {
    ROS_BRIDGE: "9090",
    VIDEO_STREAM: "9091",
    FLASK_SERVER: "5001",
  },

  // Helper getters for common URLs
  get ROS_BRIDGE_URL() {
    return `ws://${this.HOST}:${this.PORTS.ROS_BRIDGE}`;
  },
  get FLASK_SERVER_URL() {
    return `http://${this.HOST}:${this.PORTS.FLASK_SERVER}`;
  },
  get VIDEO_STREAM_URL() {
    return `http://${this.HOST}:${this.PORTS.VIDEO_STREAM}`;
  },
};
