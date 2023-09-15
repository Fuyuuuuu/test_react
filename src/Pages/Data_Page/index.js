import React from 'react';
import SearchBar from './components/SearchBar';
import DataView from './components/DataView';
import ExportButton from './components/ExportButton';

function DataPage() {
    return (
        <div>
            <SearchBar />
            <DataView />
            <ExportButton />
        </div>
    );
}

export default DataPage;
