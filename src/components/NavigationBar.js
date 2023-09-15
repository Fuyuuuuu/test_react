import React from 'react';
import { Link } from 'react-router-dom';

function NavigationBar() {
    return (
        <nav>
            <ul>
                <li><Link to="/">Home</Link></li>
                <li><Link to="/data">Data Page</Link></li>
                <li><Link to="/line">Line Page</Link></li>
            </ul>
        </nav>
    );
}

export default NavigationBar;
