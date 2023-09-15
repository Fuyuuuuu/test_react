import React from 'react';
import DashBoard from './components/DashBoard';
import Stream from './components/Stream';
import ControlButtons from './components/ControlButtons';

function Home() {
    return (
        <div>
            <ControlButtons />
            <DashBoard />
            <Stream />
        </div>
    );
}

export default Home;
