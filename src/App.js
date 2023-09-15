import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import NavigationBar from './components/NavigationBar';
import Home from './Pages/Home';
import DataPage from './Pages/Data_Page';
import LinePage from './Pages/Line_Page';

function App() {
    return (
        <Router>
            <NavigationBar />
            <Routes>
                <Route path="/" element={<Home />} />
                <Route path="/data" element={<DataPage />} />
                <Route path="/line" element={<LinePage />} />
            </Routes>
        </Router>
    );
}

export default App;
