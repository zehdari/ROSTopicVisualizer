// useRosTopic.js
import { useState, useEffect } from "react";
import { Ros, Topic, Service } from "roslib";
import { NETWORK_CONFIG } from "../config/networkConfig";

let ros;

// Initialize rosbridge connection if it's not already connected
if (!ros) {
  ros = new Ros({
    url: NETWORK_CONFIG.ROS_BRIDGE_URL, // Update with your ROS bridge URL
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
      ros.connect(NETWORK_CONFIG.ROS_BRIDGE_URL); // Ensure connection is established
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
    };
  }, [topicName, messageType]);

  return message;
};

export const useRosService = (serviceName, serviceType) => {
  const [response, setResponse] = useState(null);

  useEffect(() => {
    if (!ros.isConnected) {
      ros.connect(NETWORK_CONFIG.ROS_BRIDGE_URL);
    }

    const service = new Service({
      ros: ros,
      name: serviceName,
      serviceType: serviceType,
    });

    const request = new serviceType.Request();

    service.callService(request, (res) => {
      setResponse(res);
    });
  }, [serviceName, serviceType]);

  return response;
};

export const useRosParam = (paramName) => {
  const [paramValue, setParamValue] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (!ros.isConnected) {
      ros.connect(NETWORK_CONFIG.ROS_BRIDGE_URL);
    }

    const param = new Param({
      ros: ros,
      name: paramName,
    });

    param.get(
      (value) => {
        setParamValue(value);
        setIsLoading(false);
      },
      (err) => {
        setError(err);
        setIsLoading(false);
      }
    );
  }, [paramName]);

  return { value: paramValue, isLoading, error };
};
