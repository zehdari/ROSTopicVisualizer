import React, { useState, useEffect, useRef } from "react";
import { Ros, Service, Topic } from "roslib";
import "../styles/TopicsTable.css";
import NodeRow from "./NodeRow";
import TopicRow from "./TopicRow";
import ParamPanel from "./ParamPanel";
import TopicDetails from "./TopicDetails";
import TableHeader from "./TableHeader";
import { TOPICS_CONFIG, IGNORED_TOPICS } from "../config/topicsConfig";
import { TOPIC_TYPES } from "../config/topicTypes";
import { NETWORK_CONFIG } from "../config/networkConfig";

const TopicsTable = ({
  onAddGraph,
  onAddVideo,
  onAddPointCloud,
  visibleTopics,
  visibleVideos,
  visiblePointClouds,
  isTerminalOpen,
  isTreeOpen,
  onToggleTerminal,
  onToggleTree,
}) => {
  const [topicsByNode, setTopicsByNode] = useState({});
  const [openTopics, setOpenTopics] = useState({});
  const [expandedNodes, setExpandedNodes] = useState({});
  const [openParamNodes, setOpenParamNodes] = useState({});
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
      url: NETWORK_CONFIG.ROS_BRIDGE_URL,
    });

    newRos.on("connection", () => {
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
          const nodeNames = nodesResponse.nodes.sort((a, b) =>
            a.toLowerCase().localeCompare(b.toLowerCase())
          );

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

              setTopicsByNode(grouped);
              setLoading(false);
            });
          });
        },
        (error) => {
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

  const hasExpandedNodes = Object.values(expandedNodes).some((value) => value);

  const toggleAllNodes = () => {
    if (hasExpandedNodes) {
      // Collapse all nodes, close all parameter panels, and close all topic details
      setExpandedNodes({});
      setOpenParamNodes({});

      // Unsubscribe from all topics and clear openTopics
      Object.keys(topicSubscriptions.current).forEach((topicName) => {
        unsubscribeFromTopic(topicName);
      });
      setOpenTopics({});
    } else {
      // Expand all nodes
      const allExpandedNodes = Object.keys(topicsByNode).reduce((acc, node) => {
        acc[node] = true;
        return acc;
      }, {});
      setExpandedNodes(allExpandedNodes);
    }
  };

  const handleAddPointCloud = (topic) => {
    onAddPointCloud({
      name: topic.name,
      type: topic.type,
      timestamp: Date.now(),
    });
  };

  const handleAddGraph = (topic) => {
    // Check if the topic is already in TOPICS_CONFIG
    const existingTopicConfig = TOPICS_CONFIG.find(
      (configTopic) => configTopic.name === topic.name
    );

    if (existingTopicConfig) {
      // If topic is in TOPICS_CONFIG, use its configuration
      onAddGraph(existingTopicConfig);
      return;
    }

    // Check if the topic's type is in TOPIC_TYPES
    const typeConfig = TOPIC_TYPES[topic.type];
    if (typeConfig) {
      // Dynamically create the topic configuration
      const dynamicTopicConfig = {
        name: topic.name,
        type: topic.type,
        ...typeConfig,
        timestamp: Date.now(),
      };
      onAddGraph(dynamicTopicConfig);
    } else {
      console.warn(
        `No configuration or type definition found for topic ${topic.name} (${topic.type})`
      );
    }
  };

  const handleAddVideo = (topic) => {
    onAddVideo({ topic, port: 9091, timestamp: Date.now() }); // Pass the topic and port to the parent component
  };

  const handleParamPanelRowClick = (e, node) => {
    // Only close if clicking the row itself, not its children
    if (e.target === e.currentTarget || e.target.tagName === "TD") {
      e.stopPropagation();
      setOpenParamNodes((prev) => ({
        ...prev,
        [node]: false,
      }));
    }
  };

  const toggleParamPanel = (node, e) => {
    // Prevent the node row toggle event from firing
    e.stopPropagation();

    setOpenParamNodes((prev) => ({
      ...prev,
      [node]: !prev[node],
    }));
  };

  const toggleNodeTopics = (node) => {
    setExpandedNodes((prev) => ({
      ...prev,
      [node]: !prev[node],
    }));
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
      <TableHeader
        loading={loading}
        onRefresh={fetchTopicsAndNodes}
        hasExpandedNodes={hasExpandedNodes}
        onToggleAllNodes={toggleAllNodes}
        onToggleTerminal={onToggleTerminal}
        onToggleTree={onToggleTree}
        isTerminalOpen={isTerminalOpen}
        isTreeOpen={isTreeOpen}
        setFilter={setFilter}
      />

      {loading ? (
        <div className="loading-indicator">Loading topics...</div>
      ) : (
        <div className="topics-table-wrapper">
          <table className="topics-card topics-table">
            <tbody>
              {Object.entries(groupedFilteredTopics).map(
                ([node, topics], nodeIndex) => (
                  <React.Fragment key={`node-${nodeIndex}`}>
                    <NodeRow
                      node={node}
                      onToggleNode={toggleNodeTopics}
                      onToggleParams={toggleParamPanel}
                    />

                    {openParamNodes[node] && (
                      <tr
                        className="param-panel-row"
                        onClick={(e) => handleParamPanelRowClick(e, node)}
                      >
                        <td colSpan="3">
                          <ParamPanel initialSelectedNode={node} ros={ros} />
                        </td>
                      </tr>
                    )}

                    {expandedNodes[node] &&
                      topics.map((topic, topicIndex) => {
                        const isConfigured =
                          TOPICS_CONFIG.some(
                            (configTopic) => configTopic.name === topic.name
                          ) || topic.type in TOPIC_TYPES;

                        return (
                          <React.Fragment key={`topic-${topicIndex}`}>
                            <TopicRow
                              topic={topic}
                              isConfigured={isConfigured}
                              onToggleDetails={toggleTopicDetails}
                              onAddGraph={handleAddGraph}
                              onAddVideo={handleAddVideo}
                              onAddPointCloud={handleAddPointCloud}
                              isVisible={visibleTopics.some(
                                (vt) => vt.name === topic.name
                              )}
                              isVideoVisible={visibleVideos.some(
                                (vv) => vv.topic === topic.name
                              )}
                              isPointCloudVisible={visiblePointClouds.some(
                                (vpc) => vpc.name === topic.name
                              )}
                            />

                            {openTopics[topic.name] && (
                              <TopicDetails
                                topic={topic}
                                topicDetails={openTopics[topic.name]}
                                onToggleDetails={toggleTopicDetails} // Add this line
                              />
                            )}
                          </React.Fragment>
                        );
                      })}
                  </React.Fragment>
                )
              )}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
};

export default TopicsTable;
