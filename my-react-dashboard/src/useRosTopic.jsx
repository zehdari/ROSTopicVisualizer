// useRosTopic.js
import { useState, useEffect } from "react";
import { Ros, Topic } from "roslib";

let ros;

if (!ros) {
  ros = new Ros({
    url: "ws://localhost:9090", // Update with your ROS bridge URL
  });

  ros.on("error", (error) => {
    console.error("Ros error:", error);
  });

  ros.on("close", () => {
    console.log("Ros connection closed.");
  });
}

export const useRosTopic = (topicName, messageType) => {
  const [message, setMessage] = useState(null);

  useEffect(() => {
    if (!ros.isConnected) {
      ros.connect("ws://localhost:9090"); // Ensure connection is established
    }

    const topic = new Topic({
      ros: ros,
      name: topicName,
      messageType: messageType,
    });

    const messageListener = (msg) => {
      setMessage(msg);
    };

    topic.subscribe(messageListener);

    return () => {
      topic.unsubscribe(messageListener);
      // Optionally, you can also unadvertise the topic or handle cleanup
    };
  }, [topicName, messageType]);

  return message;
};
