import React, { useState, useEffect } from 'react';

function DataView() {
    const [data, setData] = useState([]);

    useEffect(() => {
        // 此處可以加入從 MongoDB 獲取數據的邏輯
    }, []);

    return (
        <div>
            {data.map(item => (
                <div key={item._id}>
                    {/* 顯示 item 的內容 */}
                </div>
            ))}
        </div>
    );
}

export default DataView;
