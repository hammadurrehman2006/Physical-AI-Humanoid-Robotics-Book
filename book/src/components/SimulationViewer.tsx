import React from 'react';

// A component to display simulation content or placeholder
const SimulationViewer = ({ simulation, title = 'Simulation Viewer', description = '' }) => {
  return (
    <div className="simulation-viewer">
      <div className="simulation-header">
        <h3>{title}</h3>
        {description && <p>{description}</p>}
      </div>
      <div className="simulation-content">
        <div className="simulation-placeholder">
          <p>Simulation: {simulation}</p>
          <p>This is where the ROS 2 simulation would be displayed.</p>
          <div className="simulation-controls">
            <button className="btn-play">Play</button>
            <button className="btn-pause">Pause</button>
            <button className="btn-reset">Reset</button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SimulationViewer;