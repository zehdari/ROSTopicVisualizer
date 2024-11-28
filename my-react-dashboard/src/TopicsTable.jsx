import React, { useState, useEffect, useRef } from "react";
import { Ros, Service, Topic } from "roslib";
import "./TopicsTable.css";
import SearchBar from "./SearchBar";
import { TOPICS_CONFIG } from "./topicsConfig"; // Import the existing topics configuration
import { IGNORED_TOPICS } from "./topicsConfig"; // Import ignored topics

const TopicsTable = ({ onAddGraph, visibleTopics }) => {
  const [topics, setTopics] = useState([]);
  const [openTopics, setOpenTopics] = useState({});
  const [ros, setRos] = useState(null);
  const [filter, setFilter] = useState("");

  const topicSubscriptions = useRef({});

  useEffect(() => {
    const newRos = new Ros({
      url: "ws://localhost:9090",
    });

    newRos.on("connection", () => {
      console.log("Connected to ROS websocket");

      // Create service clients
      const topicListService = new Service({
        ros: newRos,
        name: "/rosapi/topics",
        serviceType: "rosapi/Topics",
      });

      const topicTypeService = new Service({
        ros: newRos,
        name: "/rosapi/topic_type",
        serviceType: "rosapi/TopicType",
      });

      const request = {};
      topicListService.callService(
        request,
        (response) => {
          // Filter topics based on IGNORED_TOPICS array
          const filteredTopics = response.topics.filter(
            (topicName) => !IGNORED_TOPICS.includes(topicName)
          );

          const topicDetailsPromises = filteredTopics.map((topicName) => {
            return new Promise((resolve) => {
              const typeRequest = { topic: topicName };
              topicTypeService.callService(
                typeRequest,
                (typeResponse) => {
                  resolve({
                    name: topicName,
                    type: typeResponse.type,
                  });
                },
                (error) => {
                  console.error(`Error getting type for ${topicName}:`, error);
                  resolve({
                    name: topicName,
                    type: "Unknown",
                  });
                }
              );
            });
          });

          Promise.all(topicDetailsPromises).then((resolvedTopics) => {
            const sortedTopics = resolvedTopics.sort((a, b) => {
              const namespaceA = a.name.split("/").slice(0, -1).join("/");
              const namespaceB = b.name.split("/").slice(0, -1).join("/");
              return namespaceA.localeCompare(namespaceB);
            });

            setTopics(sortedTopics);
          });
        },
        (error) => {
          console.error("Error fetching topics:", error);
        }
      );

      setRos(newRos);
    });

    newRos.on("error", (error) => {
      console.error("ROS connection error:", error);
    });

    return () => {
      if (newRos) {
        newRos.close();
      }
    };
  }, []);

  const subscribeToTopic = (topic) => {
    if (topicSubscriptions.current[topic.name]) return;

    const topicInstance = new Topic({
      ros: ros,
      name: topic.name,
      messageType: topic.type,
    });

    let messageCount = 0;
    let frequency = 0;
    let lastMessageTime = Date.now(); // Track the time of the last message

    // Start an interval to update the frequency every second
    const frequencyInterval = setInterval(() => {
      const now = Date.now();
      const timeDiff = (now - lastMessageTime) / 1000; // Time difference in seconds

      // If no messages in the last second, set frequency to 0
      if (messageCount === 0 && timeDiff >= 1) {
        frequency = 0;
      } else {
        frequency = messageCount; // Set frequency to the message count in the last second
      }

      // Update the frequency in the state
      setOpenTopics((prevOpen) => ({
        ...prevOpen,
        [topic.name]: {
          ...prevOpen[topic.name],
          frequency: frequency.toFixed(2), // Display frequency with 2 decimal places
        },
      }));

      // Reset message count for the next second
      messageCount = 0;
    }, 1000); // Every 1000ms (1 second)

    // Subscribe to the topic and count the messages
    topicInstance.subscribe((msg) => {
      messageCount++;
      lastMessageTime = Date.now(); // Update the last message time
      setOpenTopics((prevOpen) => ({
        ...prevOpen,
        [topic.name]: {
          ...prevOpen[topic.name],
          latestMessage: JSON.stringify(msg, null, 2),
        },
      }));
    });

    topicSubscriptions.current[topic.name] = {
      topicInstance,
      frequencyInterval,
    };
  };

  const unsubscribeFromTopic = (topicName) => {
    if (!topicSubscriptions.current[topicName]) return;

    const { topicInstance, frequencyInterval } =
      topicSubscriptions.current[topicName];
    topicInstance.unsubscribe();

    // Clear the frequency interval when unsubscribing
    clearInterval(frequencyInterval);

    delete topicSubscriptions.current[topicName];
  };

  const toggleTopicDetails = (topicName) => {
    setOpenTopics((prev) => {
      const isOpen = prev[topicName];
      if (!isOpen) {
        const topic = topics.find((t) => t.name === topicName);
        subscribeToTopic(topic);
      } else {
        unsubscribeFromTopic(topicName);
      }

      return {
        ...prev,
        [topicName]: !isOpen,
      };
    });
  };

  const handleAddGraph = (topic) => {
    // Check if the topic is already configured in TOPICS_CONFIG
    const existingTopicConfig = TOPICS_CONFIG.find(
      (configTopic) => configTopic.name === topic.name
    );

    // If the topic is not in the configuration, you might want to add a default parser or configuration
    if (!existingTopicConfig) {
      console.warn(`No configuration found for topic ${topic.name}`);
      return;
    }

    // Trigger the graph addition in the parent component
    onAddGraph(existingTopicConfig);
  };

  // Filter topics based on the search term
  const filteredTopics = topics.filter((topic) =>
    topic.name.toLowerCase().includes(filter.toLowerCase())
  );

  return (
    <div className="topics-table-container">
      <SearchBar setFilter={setFilter} />
      <div className="topics-table-wrapper">
        <table className="graph-card topics-table">
          <tbody>
            {filteredTopics.map((topic, index) => {
              // Check if the topic is in the configuration
              const isConfigured = TOPICS_CONFIG.some(
                (configTopic) => configTopic.name === topic.name
              );

              return (
                <React.Fragment key={index}>
                  <tr
                    className="hidden-graph-item"
                    onClick={() => toggleTopicDetails(topic.name)}
                  >
                    <td className="topic-name">{topic.name}</td>
                    <td>{topic.type}</td>
                    <td>
                      {isConfigured &&
                        !visibleTopics.some(
                          (visibleTopic) => visibleTopic.name === topic.name
                        ) && (
                          <button
                            onClick={(e) => {
                              e.stopPropagation(); // Prevent row click event
                              handleAddGraph(topic);
                            }}
                            className="add-graph-btn"
                          >
                            +
                          </button>
                        )}
                    </td>
                  </tr>
                  {openTopics[topic.name] && (
                    <TopicDetailsView
                      topic={topic}
                      latestMessage={openTopics[topic.name]?.latestMessage}
                      frequency={openTopics[topic.name]?.frequency}
                    />
                  )}
                </React.Fragment>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
};

const TopicDetailsView = ({ topic, latestMessage, frequency }) => (
  <tr className="topic-details-row">
    <td colSpan="3">
      <div className="graph-card topic-details-content">
        <p>
          <strong>Type:</strong> {topic.type}
        </p>
        <p>
          <strong>Frequency:</strong>{" "}
          {frequency ? `${frequency} Hz` : "Calculating..."}{" "}
          {/* Show 0 Hz if no frequency */}
        </p>
        {latestMessage ? (
          <>
            <p>
              <strong>Latest Message:</strong>
            </p>
            <pre>{latestMessage}</pre>
          </>
        ) : (
          <p>Waiting for message...</p>
        )}
      </div>
    </td>
  </tr>
);

export default TopicsTable;
