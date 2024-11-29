import React, { useState, useEffect, useRef } from "react";
import { Ros, Service, Topic } from "roslib";
import "../styles/TopicsTable.css";
import { RefreshCcw } from "lucide-react";
import SearchBar from "./SearchBar";
import ParamPanel from "./ParamPanel";
import { TOPICS_CONFIG, IGNORED_TOPICS } from "../config/topicsConfig";

const TopicsTable = ({ onAddGraph, visibleTopics }) => {
  const [topicsByNode, setTopicsByNode] = useState({});
  const [openTopics, setOpenTopics] = useState({});
  const [selectedNode, setSelectedNode] = useState("");
  const [ros, setRos] = useState(null);
  const [filter, setFilter] = useState("");
  const [loading, setLoading] = useState(true);

  const topicSubscriptions = useRef({});
  const rosRef = useRef(null);

  const fetchTopicsAndNodes = () => {
    setLoading(true);
    // Clear existing subscriptions
    Object.values(topicSubscriptions.current).forEach(
      ({ topicInstance, frequencyInterval }) => {
        topicInstance.unsubscribe();
        clearInterval(frequencyInterval);
      }
    );
    topicSubscriptions.current = {};
    setOpenTopics({});

    if (rosRef.current) {
      rosRef.current.close();
    }

    const newRos = new Ros({
      url: "ws://192.168.1.19:9090",
    });

    newRos.on("connection", () => {
      console.log("Connected to ROS websocket");

      // Create service clients for rosapi/nodes and rosapi/node_details
      const nodesService = new Service({
        ros: newRos,
        name: "/rosapi/nodes",
        serviceType: "rosapi/Nodes",
      });

      const nodeDetailsService = new Service({
        ros: newRos,
        name: "/rosapi/node_details",
        serviceType: "rosapi/NodeDetails",
      });

      const topicTypeService = new Service({
        ros: newRos,
        name: "/rosapi/topic_type",
        serviceType: "rosapi/TopicType",
      });

      // Fetch the list of nodes
      nodesService.callService(
        {},
        (nodesResponse) => {
          const nodeNames = nodesResponse.nodes;
          console.log("Fetched Nodes:", nodeNames);

          // For each node, fetch its details
          const nodeDetailsPromises = nodeNames.map((nodeName) => {
            return new Promise((resolve) => {
              nodeDetailsService.callService(
                { node: nodeName },
                (detailsResponse) => {
                  resolve({
                    node: nodeName,
                    publishing: detailsResponse.publishing,
                    subscribing: detailsResponse.subscribing,
                    services: detailsResponse.services,
                  });
                },
                (error) => {
                  console.error(
                    `Error fetching details for node ${nodeName}:`,
                    error
                  );
                  resolve(null); // Resolve with null on error
                }
              );
            });
          });

          Promise.all(nodeDetailsPromises).then((nodesDetails) => {
            // Filter out any null responses due to errors
            const validNodesDetails = nodesDetails.filter(
              (detail) => detail !== null
            );

            // Collect all unique publishing topics
            const allPublishingTopics = validNodesDetails
              .flatMap((node) => node.publishing)
              .filter((topic) => !IGNORED_TOPICS.includes(topic));

            // Remove duplicate topics
            const uniquePublishingTopics = Array.from(
              new Set(allPublishingTopics)
            );
            console.log("Unique Publishing Topics:", uniquePublishingTopics);

            // Fetch topic types
            const topicTypePromises = uniquePublishingTopics.map(
              (topicName) => {
                return new Promise((resolve) => {
                  topicTypeService.callService(
                    { topic: topicName },
                    (typeResponse) => {
                      resolve({
                        name: topicName,
                        type: typeResponse.type,
                      });
                    },
                    (error) => {
                      console.error(
                        `Error getting type for ${topicName}:`,
                        error
                      );
                      resolve({
                        name: topicName,
                        type: "Unknown",
                      });
                    }
                  );
                });
              }
            );

            Promise.all(topicTypePromises).then((topicsWithTypes) => {
              // Create a map for quick lookup of topic types
              const topicTypeMap = topicsWithTypes.reduce((acc, topic) => {
                acc[topic.name] = topic.type;
                return acc;
              }, {});
              console.log("Topic Type Map:", topicTypeMap);

              // Group topics by node based on publishing topics
              const grouped = validNodesDetails.reduce((acc, nodeDetail) => {
                const publishingTopics = nodeDetail.publishing
                  .filter((topic) => !IGNORED_TOPICS.includes(topic))
                  .map((topic) => ({
                    name: topic,
                    type: topicTypeMap[topic] || "Unknown",
                  }));

                if (publishingTopics.length > 0) {
                  acc[nodeDetail.node] = publishingTopics;
                }

                return acc;
              }, {});

              console.log("Grouped Topics by Node:", grouped);

              setTopicsByNode(grouped);
              setLoading(false);
            });
          });
        },
        (error) => {
          console.error("Error fetching nodes:", error);
          setLoading(false);
        }
      );

      rosRef.current = newRos;
      setRos(newRos);
    });

    newRos.on("error", (error) => {
      console.error("ROS connection error:", error);
      setLoading(false);
    });
  };

  useEffect(() => {
    fetchTopicsAndNodes();

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
      // Clean up subscriptions
      Object.values(topicSubscriptions.current).forEach(
        ({ topicInstance, frequencyInterval }) => {
          topicInstance.unsubscribe();
          clearInterval(frequencyInterval);
        }
      );
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
        const nodeEntries = Object.entries(topicsByNode);
        let topic;
        for (const [, topics] of nodeEntries) {
          topic = topics.find((t) => t.name === topicName);
          if (topic) break;
        }
        if (topic) {
          subscribeToTopic(topic);
        }
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

  const toggleNodeDetails = (node) => {
    // If clicking the same node, deselect it
    setSelectedNode((prevNode) => (prevNode === node ? "" : node));
  };

  // Enhanced Filtering: Filter topics and nodes based on the search term
  const groupedFilteredTopics = Object.entries(topicsByNode).reduce(
    (acc, [node, topics]) => {
      const lowerCaseFilter = filter.toLowerCase();
      const nodeMatches = node.toLowerCase().includes(lowerCaseFilter);

      if (nodeMatches) {
        // If node name matches, include all its topics
        acc[node] = topics;
      } else {
        // Otherwise, filter topics within the node
        const filteredTopics = topics.filter((topic) =>
          topic.name.toLowerCase().includes(lowerCaseFilter)
        );
        if (filteredTopics.length > 0) {
          acc[node] = filteredTopics;
        }
      }

      return acc;
    },
    {}
  );

  return (
    <div className="topics-table-container">
      <div className="search-and-refresh-container">
        <div className="search-bar-wrapper">
          <SearchBar setFilter={setFilter} />
        </div>
        <button
          onClick={fetchTopicsAndNodes}
          className="refresh-topics-btn"
          disabled={loading}
          aria-label="Refresh topics"
        >
          <RefreshCcw size={16} className="refresh-icon" />
        </button>
      </div>
      {loading ? (
        <div className="loading-indicator">Loading topics...</div>
      ) : (
        <div className="topics-table-wrapper">
          <table className="topics-card topics-table">
            <tbody>
              {Object.entries(groupedFilteredTopics).map(
                ([node, topics], nodeIndex) => (
                  <React.Fragment key={`node-${nodeIndex}`}>
                    {/* Node Row */}
                    <tr
                      className="node-row"
                      onClick={() => toggleNodeDetails(node)}
                      style={{ cursor: "pointer" }}
                    >
                      <td colSpan="3">
                        <strong>{node}</strong>
                      </td>
                    </tr>
                    {selectedNode === node && (
                      <tr className="param-panel-row">
                        <td colSpan="3">
                          <ParamPanel initialSelectedNode={node} ros={ros} />
                        </td>
                      </tr>
                    )}
                    {/* Topic Rows */}
                    {topics.map((topic, topicIndex) => {
                      // Check if the topic is in the configuration
                      const isConfigured = TOPICS_CONFIG.some(
                        (configTopic) => configTopic.name === topic.name
                      );

                      return (
                        <React.Fragment key={`topic-${topicIndex}`}>
                          <tr
                            className="topic-row hidden-graph-item"
                            onClick={() => toggleTopicDetails(topic.name)}
                          >
                            <td className="topic-name">{topic.name}</td>
                            <td>{topic.type}</td>
                            <td>
                              {isConfigured &&
                                !visibleTopics.some(
                                  (visibleTopic) =>
                                    visibleTopic.name === topic.name
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
                            <tr className="topic-details-row">
                              <td colSpan="3">
                                <div className="graph-card topic-details-content">
                                  <p>
                                    <strong>Type:</strong> {topic.type}
                                  </p>
                                  <p>
                                    <strong>Frequency:</strong>{" "}
                                    {openTopics[topic.name]?.frequency
                                      ? `${openTopics[topic.name].frequency} Hz`
                                      : "Calculating..."}
                                  </p>
                                  {openTopics[topic.name]?.latestMessage ? (
                                    <>
                                      <p>
                                        <strong>Latest Message:</strong>
                                      </p>
                                      <pre>
                                        {openTopics[topic.name].latestMessage}
                                      </pre>
                                    </>
                                  ) : (
                                    <p>Waiting for message...</p>
                                  )}
                                </div>
                              </td>
                            </tr>
                          )}
                        </React.Fragment>
                      );
                    })}
                  </React.Fragment>
                )
              )}
              {Object.keys(groupedFilteredTopics).length === 0 && (
                <tr>
                  <td colSpan="3">No topics found.</td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
};

export default TopicsTable;
