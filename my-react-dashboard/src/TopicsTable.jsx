// TopicsTable.jsx
import React, { useState, useEffect, useRef } from "react";
import { Ros, Service, Topic } from "roslib";
import "./TopicsTable.css";
import SearchBar from "./SearchBar"; // Import the new SearchBar component

const TopicsTable = () => {
  const [topics, setTopics] = useState([]);
  const [openTopics, setOpenTopics] = useState({});
  const [ros, setRos] = useState(null);
  const [filter, setFilter] = useState(""); // State for storing the search filter

  const topicSubscriptions = useRef({});
  const ignoredTopics = [
    "/rosout",
    "/client_count",
    "/connected_clients",
    "/parameter_events",
  ];

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
          const filteredTopics = response.topics.filter(
            (topicName) => !ignoredTopics.includes(topicName)
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

    topicInstance.subscribe((msg) => {
      setOpenTopics((prevOpen) => ({
        ...prevOpen,
        [topic.name]: {
          ...prevOpen[topic.name],
          latestMessage: JSON.stringify(msg, null, 2),
        },
      }));
    });

    topicSubscriptions.current[topic.name] = topicInstance;
  };

  const unsubscribeFromTopic = (topicName) => {
    if (!topicSubscriptions.current[topicName]) return;

    const topicInstance = topicSubscriptions.current[topicName];
    topicInstance.unsubscribe();

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

  // Filter topics based on the search term
  const filteredTopics = topics.filter((topic) =>
    topic.name.toLowerCase().includes(filter)
  );

  return (
    <div className="topics-table-container">
      <SearchBar setFilter={setFilter} /> {/* Use the SearchBar component */}
      <div className="topics-table-wrapper">
        <table className="graph-card topics-table">
          <tbody>
            {filteredTopics.map((topic, index) => (
              <React.Fragment key={index}>
                <tr
                  className="hidden-graph-item"
                  onClick={() => toggleTopicDetails(topic.name)}
                >
                  <td className="topic-name">{topic.name}</td>
                  <td>{topic.type}</td>
                </tr>
                {openTopics[topic.name] && (
                  <TopicDetailsView
                    topic={topic}
                    latestMessage={openTopics[topic.name]?.latestMessage}
                  />
                )}
              </React.Fragment>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

const TopicDetailsView = ({ topic, latestMessage }) => (
  <tr className="topic-details-row">
    <td colSpan="3">
      <div className="graph-card topic-details-content">
        <p>
          <strong>Type:</strong> {topic.type}
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
