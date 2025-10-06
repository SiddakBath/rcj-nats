# Localization Web Server

A real-time web-based visualization system for the soccer robot's localization data. This server provides a visual interface to monitor the robot's position, sensor readings, and field boundaries.

## Features

- **Real-time Field Visualization**: Shows the soccer field with robot position and orientation
- **Sensor Ray Display**: Visualizes TOF sensor readings as rays from the robot
- **Position Tracking**: Displays current robot coordinates and heading
- **Confidence Monitoring**: Shows localization confidence level
- **Hardware Status**: Displays sensor count and system status
- **Interactive Controls**: Reset position and toggle visualization options

## Requirements

- Python 3.7+
- Flask
- Access to the soccer robot hardware modules (TOF sensors, IMU)
- Web browser for visualization

## Installation

1. Ensure the soccer robot modules are available in the parent directory
2. Install required dependencies:
   ```bash
   pip install flask numpy
   ```

## Usage

### With Real Hardware

```bash
cd localization_server
python localization_web_server.py
```

### Testing Without Hardware

```bash
cd localization_server
python test_localization_server.py
```

The server will start on `http://localhost:5001` (or `http://0.0.0.0:5001` for network access).

## Web Interface

Open your web browser and navigate to the server URL to access:

- **Field View**: Top-down view of the soccer field with robot position
- **Robot Information**: Current position, angle, and confidence
- **System Status**: Hardware availability and sensor counts
- **Controls**: Reset position and toggle visualization options

## API Endpoints

- `GET /api/localization_data` - Get current robot position and sensor data
- `GET /api/sensor_data` - Get detailed sensor information
- `GET /api/field_info` - Get field configuration and boundaries
- `GET /api/status` - Get system status and hardware availability
- `POST /api/reset_position` - Reset robot position to specified coordinates

## Configuration

The server uses configuration from the soccer robot's `config.py` file:

- Field dimensions and wall definitions
- TOF sensor configuration
- Localization parameters
- Update frequencies

## Troubleshooting

1. **No Hardware Detected**: The server will run in simulation mode if hardware is not available
2. **Connection Issues**: Check that the soccer robot modules are properly installed
3. **Visualization Problems**: Ensure your browser supports HTML5 Canvas

## Development

To modify the visualization:

1. Edit `templates/localization_viewer.html` for UI changes
2. Modify `localization_web_server.py` for server logic
3. Use `test_localization_server.py` for testing without hardware

## Integration

This server integrates with the existing soccer robot localization system:

- Uses the `Localizer` class from `soccer.localization`
- Reads configuration from `soccer.config`
- Interfaces with TOF sensors and IMU through the existing modules
