import React, { useRef, useEffect } from 'react';

function Stream() {
    const videoRef = useRef(null);

    useEffect(() => {
        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            navigator.mediaDevices.getUserMedia({ video: true })
                .then(stream => {
                    if (videoRef.current) {
                        videoRef.current.srcObject = stream;
                    }
                })
                .catch(err => {
                    console.error("Error accessing webcam: ", err);
                });
        }
    }, []);

    return (
        <div>
            <p>Stream:</p>
            <video ref={videoRef} autoPlay playsInline width="480" height="360"></video>
        </div>
    );
}

export default Stream;
