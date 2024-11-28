import React, { useState } from "react";
import "./SearchBar.css";

const SearchBar = ({ setFilter }) => {
  const [searchTerm, setSearchTerm] = useState("");

  const handleInputChange = (event) => {
    const newSearchTerm = event.target.value;
    setSearchTerm(newSearchTerm);
    setFilter(newSearchTerm.toLowerCase()); // Pass the lowercase term to parent
  };

  const handleClearClick = () => {
    setSearchTerm(""); // Clear the input field
    setFilter(""); // Clear the filter in the parent component
  };

  return (
    <div className="search-bar">
      <input
        type="text"
        placeholder="Search for a topic..."
        value={searchTerm}
        onChange={handleInputChange}
      />
      {searchTerm && (
        <button className="clear-btn" onClick={handleClearClick}>
          &#10005; {/* Unicode "X" symbol */}
        </button>
      )}
    </div>
  );
};

export default SearchBar;
