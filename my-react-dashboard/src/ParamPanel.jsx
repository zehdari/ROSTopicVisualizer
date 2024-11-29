import React, { useState, useEffect, useRef } from "react";
import { Ros, Service } from "roslib";
import ClipLoader from "react-spinners/ClipLoader";
import "./ParamPanel.css";

const ParamPanel = ({ initialSelectedNode = "", ros: externalRos }) => {
  const [nodes, setNodes] = useState([]);
  const [selectedNode, setSelectedNode] = useState(initialSelectedNode);
  const [nodeParams, setNodeParams] = useState([]);
  const [paramValues, setParamValues] = useState({});
  const [error, setError] = useState(null);
  const [message, setMessage] = useState(null);
  const [ros, setRos] = useState(externalRos);
  const [isLoadingParams, setIsLoadingParams] = useState(false);
  const [showSpinner, setShowSpinner] = useState(false);

  const currentNodeRef = useRef(selectedNode);

  useEffect(() => {
    currentNodeRef.current = selectedNode;
  }, [selectedNode]);

  // If no external ROS instance is provided, create one
  useEffect(() => {
    if (!externalRos) {
      const newRos = new Ros({
        url: "ws://localhost:9090",
      });

      newRos.on("connection", () => {
        console.log("Connected to ROS websocket");
        setRos(newRos);

        // Fetch nodes logic (similar to previous implementation)
        const nodesService = new Service({
          ros: newRos,
          name: "/rosapi/nodes",
          serviceType: "rosapi/Nodes",
        });

        nodesService.callService(
          {},
          (response) => {
            const filteredNodes = response.nodes.filter(
              (node) =>
                !node.startsWith("/rosout") &&
                !node.startsWith("/master") &&
                !node.startsWith("/rosapi")
            );

            setNodes(filteredNodes);

            // If an initial node was provided and it's in the list, select it
            if (
              initialSelectedNode &&
              filteredNodes.includes(initialSelectedNode)
            ) {
              setSelectedNode(initialSelectedNode);
            }
          },
          (error) => {
            console.error("Error fetching nodes:", error);
            setError(`Failed to retrieve nodes: ${error}`);
          }
        );
      });

      return () => {
        if (newRos) {
          newRos.close();
        }
      };
    }
  }, [externalRos, initialSelectedNode]);

  useEffect(() => {
    if (!ros || !selectedNode) return;

    let isCancelled = false; // Flag to track cancellation
    setIsLoadingParams(true); // Start loading
    setShowSpinner(false); // Reset spinner visibility

    // Initialize a timer to show the spinner after 500ms
    const spinnerTimer = setTimeout(() => {
      if (!isCancelled) {
        setShowSpinner(true);
      }
    }, 500);

    const listAndFetchParams = async () => {
      // Step 1: List Parameters
      const listParamsService = new Service({
        ros: ros,
        name: `${selectedNode}/list_parameters`,
        serviceType: "rcl_interfaces/ListParameters",
      });

      const listRequest = { prefixes: [] }; // Adjust if needed
      try {
        const listResult = await new Promise((resolve, reject) => {
          listParamsService.callService(listRequest, resolve, reject);
        });

        const paramNames = listResult.result
          ? listResult.result.names
          : listResult.names || [];

        console.log(`Parameters for ${selectedNode}:`, paramNames);

        if (!isCancelled) {
          setNodeParams(Array.isArray(paramNames) ? paramNames : []);
          setParamValues({});
        }

        // Step 2: Fetch Parameter Values
        const allParamValues = {};

        for (const param of paramNames) {
          if (isCancelled) {
            console.log(
              `Fetch operation cancelled. Skipping parameter '${param}'.`
            );
            break;
          }

          try {
            const getParamService = new Service({
              ros: ros,
              name: `${selectedNode}/get_parameters`,
              serviceType: "rcl_interfaces/GetParameters",
            });

            const getRequest = { names: [param] };
            const getResult = await new Promise((resolve, reject) => {
              getParamService.callService(getRequest, resolve, reject);
            });

            console.log(`Result for parameter ${param}:`, getResult);

            const paramValue = getResult?.result
              ? getResult.result.values[0]
              : getResult.values
              ? getResult.values[0]
              : undefined;

            if (!paramValue) {
              console.warn(`Parameter value for '${param}' is undefined.`);
              allParamValues[param] = { type: null, value: "Undefined" };
              continue; // Skip processing this parameter
            }

            // Convert numeric types to strings
            if (paramValue.type === 2) {
              // Integer
              allParamValues[param] = {
                ...paramValue,
                integer_value: paramValue.integer_value.toString(),
              };
            } else if (paramValue.type === 3) {
              // Double
              allParamValues[param] = {
                ...paramValue,
                double_value: Number.isInteger(paramValue.double_value)
                  ? `${paramValue.double_value}.0`
                  : paramValue.double_value.toString(),
              };
            } else {
              allParamValues[param] = paramValue;
            }
          } catch (err) {
            console.error(`Failed to retrieve parameter '${param}':`, err);
            if (!isCancelled) {
              setError(
                `Failed to retrieve parameter '${param}': ${err.message || err}`
              );
            }
          }
        }

        if (!isCancelled) {
          setParamValues(allParamValues);
        } else {
          console.log(
            `Discarded fetched parameters for '${selectedNode}' as the operation was cancelled.`
          );
        }
      } catch (err) {
        console.error(
          `Error fetching parameters for node '${selectedNode}':`,
          err
        );
        if (!isCancelled) {
          setError(
            `Could not retrieve parameters for node '${selectedNode}': ${
              err.message || err
            }`
          );
        }
      } finally {
        // Clear the spinner timer and reset loading states
        clearTimeout(spinnerTimer);
        if (!isCancelled) {
          setIsLoadingParams(false);
          setShowSpinner(false);
        }
      }
    };

    listAndFetchParams();

    return () => {
      // Cleanup function to set the cancellation flag and clear the timer
      isCancelled = true;
      clearTimeout(spinnerTimer);
      setIsLoadingParams(false);
      setShowSpinner(false);
    };
  }, [ros, selectedNode]);

  const renderParamValue = (param, paramValue) => {
    if (!paramValue || paramValue.type === null)
      return <em>No Value Available</em>;

    switch (paramValue.type) {
      case 1: // Bool
        return (
          <label className="custom-checkbox">
            <input
              type="checkbox"
              checked={paramValue.bool_value || false}
              onChange={(e) =>
                handleParamChange(param, "bool", e.target.checked)
              }
            />
            <span className="checkmark"></span>
          </label>
        );
      case 2: // Integer
        return (
          <input
            type="number"
            value={paramValue.integer_value || ""}
            onChange={(e) =>
              handleParamChange(param, "integer", e.target.value)
            }
          />
        );
      case 3: // Double
        return (
          <input
            type="number"
            value={paramValue.double_value || ""}
            step="any"
            onChange={(e) => handleParamChange(param, "double", e.target.value)}
          />
        );
      case 4: // String
        return (
          <input
            type="text"
            value={paramValue.string_value || ""}
            onChange={(e) => handleParamChange(param, "string", e.target.value)}
          />
        );
      case 7: // Integer Array
        return (
          <input
            type="text"
            value={paramValue.integer_array_value.join(",") || ""}
            onChange={(e) =>
              handleParamChange(param, "integer_array", e.target.value)
            }
            placeholder="Enter comma-separated integers"
          />
        );
      case 8: // Double Array
        return (
          <input
            type="text"
            value={paramValue.double_array_value.join(",") || ""}
            onChange={(e) =>
              handleParamChange(param, "double_array", e.target.value)
            }
            placeholder="Enter comma-separated doubles"
          />
        );
      case 9: // String Array
        return (
          <input
            type="text"
            value={paramValue.string_array_value.join(",") || ""}
            onChange={(e) =>
              handleParamChange(param, "string_array", e.target.value)
            }
            placeholder="Enter comma-separated strings"
          />
        );
      default:
        return <span>Unknown Type</span>;
    }
  };

  const handleParamChange = (param, type, newValue) => {
    setParamValues((prevValues) => {
      const updatedValues = { ...prevValues };
      updatedValues[param] = { ...updatedValues[param] };

      // Update the correct field based on type
      switch (type) {
        case "bool":
          updatedValues[param].bool_value = newValue;
          break;
        case "integer":
          updatedValues[param].integer_value = newValue;
          break;
        case "double":
          updatedValues[param].double_value = newValue;
          break;
        case "string":
          updatedValues[param].string_value = newValue;
          break;
        default:
          break;
      }

      return updatedValues;
    });
  };

  const updateParams = async () => {
    if (!ros || !selectedNode) return;

    // Prepare the parameters to be updated
    const updatedParams = Object.keys(paramValues)
      .map((param) => {
        const paramValue = paramValues[param];
        const paramType = paramValue.type;

        if (paramType === null) {
          console.warn(`Skipping parameter '${param}' due to undefined type.`);
          return null;
        }

        let paramUpdate = {
          name: param,
          value: null,
        };

        // Wrap the value in the correct ParameterValue structure
        switch (paramType) {
          case 1: // Bool
            paramUpdate.value = {
              type: 1,
              bool_value: paramValue.bool_value,
            };
            break;
          case 2: // Integer
            paramUpdate.value = {
              type: 2,
              integer_value: parseInt(paramValue.integer_value, 10),
            };
            break;
          case 3: // Double
            paramUpdate.value = {
              type: 3,
              double_value: parseFloat(paramValue.double_value),
            };
            break;
          case 4: // String
            paramUpdate.value = {
              type: 4,
              string_value: paramValue.string_value,
            };
            break;
          case 5: // Byte Array
            paramUpdate.value = {
              type: 5,
              byte_array_value: paramValue.byte_array_value,
            };
            break;
          case 6: // Bool Array
            paramUpdate.value = {
              type: 6,
              bool_array_value: paramValue.bool_array_value,
            };
            break;
          case 7: // Integer Array
            paramUpdate.value = {
              type: 7,
              integer_array_value: paramValue.integer_array_value,
            };
            break;
          case 8: // Double Array
            paramUpdate.value = {
              type: 8,
              double_array_value: paramValue.double_array_value,
            };
            break;
          case 9: // String Array
            paramUpdate.value = {
              type: 9,
              string_array_value: paramValue.string_array_value,
            };
            break;
          default:
            console.warn(`Unsupported parameter type: ${paramType}`);
            return null; // Skip unsupported types
        }

        return paramUpdate;
      })
      .filter((param) => param !== null);

    // Check if there are parameters to update
    if (updatedParams.length === 0) {
      setMessage("No valid parameters to update.");
      return;
    }

    // Log the updated parameters to verify their structure
    console.log("Updated Parameters:", updatedParams);

    // Call the ROS service to update each parameter individually
    for (const param of updatedParams) {
      const setParamService = new Service({
        ros: ros,
        name: `${selectedNode}/set_parameters`,
        serviceType: "rcl_interfaces/SetParameters",
      });

      const request = { parameters: [param] }; // One parameter per request

      try {
        // Log before calling the service
        console.log(
          `Sending request to update parameter: ${param.name}`,
          request
        );

        // Call the ROS service for each parameter
        await new Promise((resolve, reject) => {
          setParamService.callService(request, resolve, reject);
        });

        // Log success response
        console.log(`Parameter '${param.name}' updated successfully`);
      } catch (err) {
        // Log the error if the service call fails for a specific parameter
        console.error(`Failed to update parameter '${param.name}':`, err);
        setError(
          `Failed to update parameter '${param.name}': ${err.message || err}`
        );
      }
    }

    setMessage("All parameters updated successfully!");
  };

  // Clear messages after 5 seconds
  useEffect(() => {
    if (message || error) {
      const timer = setTimeout(() => {
        setMessage(null);
        setError(null);
      }, 5000); // 5 seconds

      return () => clearTimeout(timer);
    }
  }, [message, error]);

  return (
    <div className="param-panel">
      {/* Message Display */}
      <div className="message-container">
        {error && <p className="error-message">{error}</p>}
        {message && <p className="info-message">{message}</p>}
      </div>

      {/* Conditionally Render Parameters and Update Button Only When a Node is Selected */}
      {selectedNode && (
        <>
          {/* Display parameters with input fields for values */}
          <div className="param-values-container">
            {isLoadingParams || showSpinner ? (
              <div className="spinner-container">
                {showSpinner && (
                  <ClipLoader
                    size={35}
                    color={"#123abc"}
                    loading={showSpinner}
                  />
                )}
              </div>
            ) : nodeParams.length > 0 ? (
              <div>
                {nodeParams.map((param) => (
                  <div key={param} className="param-value-item">
                    <strong>{param}: </strong>
                    {renderParamValue(param, paramValues[param])}
                  </div>
                ))}
              </div>
            ) : (
              <p>No parameters available for this node.</p>
            )}
          </div>

          {/* Button to update parameters */}
          {!isLoadingParams &&
            showSpinner === false &&
            nodeParams.length > 0 && (
              <div className="update-button-container">
                <button onClick={updateParams}>Update Parameters</button>
              </div>
            )}
        </>
      )}
    </div>
  );
};

export default ParamPanel;
