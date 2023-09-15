import React, { useState } from 'react';

function SearchBar() {
    const [searchTerm, setSearchTerm] = useState('');

    const handleSearch = () => {
        // 在此處實現搜索邏輯，例如呼叫 API 或更新狀態
    };

    return (
        <div>
            <input
                type="text"
                value={searchTerm}
                onChange={e => setSearchTerm(e.target.value)}
                placeholder="Search..."
            />
            <button onClick={handleSearch}>Search</button>
        </div>
    );
}

export default SearchBar;
