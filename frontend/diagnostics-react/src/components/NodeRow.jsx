// NodeRow.jsx
import React from "react";
import { Settings } from "lucide-react";

const NodeRow = ({ node, onToggleNode, onToggleParams }) => {
  return (
    <tr
      className="node-row"
      onClick={() => onToggleNode(node)}
      style={{ cursor: "pointer" }}
    >
      <td colSpan="2">
        <strong>{node}</strong>
      </td>
      <td className="node-actions">
        <button
          onClick={(e) => onToggleParams(node, e)}
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
