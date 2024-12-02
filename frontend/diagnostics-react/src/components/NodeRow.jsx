import React from "react";
import { Settings } from "lucide-react";

const NodeRow = ({ node, onToggleNode, onToggleParams }) => {
  const handleParamsClick = (e) => {
    e.preventDefault(); // Prevent default touch behavior
    e.stopPropagation(); // Stop event from bubbling up
    onToggleParams(node, e);
  };

  const handleRowClick = (e) => {
    e.preventDefault(); // Prevent default touch behavior
    onToggleNode(node);
  };

  return (
    <tr
      className="node-row"
      onClick={handleRowClick}
      style={{ cursor: "pointer" }}
    >
      <td colSpan="2">
        <strong>{node}</strong>
      </td>
      <td className="node-actions">
        <button
          onClick={handleParamsClick}
          className="params-btn"
          aria-label="View Node Parameters"
        >
          <Settings size={16} />
        </button>
      </td>
    </tr>
  );
};

export default NodeRow;
