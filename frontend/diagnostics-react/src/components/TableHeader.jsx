// TableHeader.jsx
import React from "react";
import {
  RefreshCcw,
  Expand,
  Minimize,
  Terminal,
  TreeDeciduous,
} from "lucide-react";
import SearchBar from "./SearchBar";

const TableHeader = ({
  loading,
  onRefresh,
  hasExpandedNodes,
  onToggleAllNodes,
  onToggleTerminal,
  onToggleTree,
  isTerminalOpen,
  isTreeOpen,
  setFilter,
}) => {
  return (
    <div className="search-and-refresh-container">
      <div className="search-bar-wrapper">
        <SearchBar setFilter={setFilter} />
      </div>
      <div className="table-action-buttons">
        <button
          onClick={onRefresh}
          className="refresh-topics-btn"
          disabled={loading}
          aria-label="Refresh topics"
        >
          <RefreshCcw size={16} className="refresh-icon" />
        </button>
        <button
          onClick={onToggleAllNodes}
          className="expand-collapse-btn"
          aria-label={hasExpandedNodes ? "Collapse All" : "Expand All"}
        >
          {hasExpandedNodes ? <Minimize size={16} /> : <Expand size={16} />}
        </button>
        <button
          onClick={onToggleTerminal}
          className="terminal-toggle-btn"
          aria-label={isTerminalOpen ? "Close Terminal" : "Open Terminal"}
        >
          <Terminal size={16} />
        </button>
        <button
          onClick={onToggleTree}
          className="tree-toggle-btn"
          aria-label={isTreeOpen ? "Hide Tree" : "Show Tree"}
        >
          <TreeDeciduous size={16} />
        </button>
      </div>
    </div>
  );
};

export default TableHeader;
