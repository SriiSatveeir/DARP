#!/bin/bash
# Simple script to start VNC for DARP visualization

echo "Starting VNC services..."

# Set display
export DISPLAY=:99

# Kill any existing services
pkill Xvfb 2>/dev/null
pkill x11vnc 2>/dev/null
pkill -f websockify 2>/dev/null
sleep 1

# Start Xvfb
Xvfb :99 -screen 0 1280x1024x24 > /dev/null 2>&1 &
sleep 2

# Start x11vnc
x11vnc -display :99 -nopw -listen 0.0.0.0 -forever -shared > /dev/null 2>&1 &
sleep 1

# Start noVNC
websockify -D --web=/usr/share/novnc/ 6080 localhost:5900 > /dev/null 2>&1

echo "✓ VNC services started!"
echo ""
echo "Next steps:"
echo "1. In VS Code PORTS tab, forward port 6080"
echo "2. Click the globe icon to open VNC in browser"
echo "3. Click 'Connect'"
echo "4. Run: python multiRobotPathPlanner.py -vis"

