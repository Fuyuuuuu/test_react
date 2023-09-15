import React, { useState, useEffect } from 'react';

function DashBoard() {
    const [data, setData] = useState({});

    useEffect(() => {
        // Fetch data logic here
    }, []);

    return (
        <div>
            <p>程式狀態: {data.program_status || "未知"}</p>
            <p>總經過的數量: {data.num || 0}</p>
            {/* Other data representation */}
        </div>
    );
}

export default DashBoard;
