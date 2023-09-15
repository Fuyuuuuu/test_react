import React from 'react';

function ExportButton() {
    const handleExport = () => {
        // 實現導出邏輯，例如將數據轉換為 Excel
    };

    return (
        <div>
            <button onClick={handleExport}>Export Data</button>
        </div>
    );
}

export default ExportButton;
