#!/bin/bash

# Variables
SERVICE_NAME="rosbridge"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_NAME="run_rosbridge.py"
SCRIPT_DIR="/opt/rosbridge"
PYTHON_PATH="/usr/bin/python3"
USER_NAME=$(whoami)
GROUP_NAME=$(id -gn)

# Ensure script is run with sudo
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root using sudo."
    exit 1
fi

# Check if the service file already exists
if [ -f "$SERVICE_FILE" ]; then
    echo "The service file ${SERVICE_FILE} already exists."
    read -p "Do you want to overwrite it? (y/n): " response
    if [[ "$response" != "y" && "$response" != "Y" ]]; then
        echo "Exiting without making changes."
        exit 0
    fi
    echo "Overwriting the existing service file..."
fi

# Check if the Python script exists
if [ ! -f "./${SCRIPT_NAME}" ]; then
    echo "Error: ${SCRIPT_NAME} not found in the current directory."
    exit 1
fi

# Create the directory for the Python script
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "Creating directory ${SCRIPT_DIR}..."
    mkdir -p "$SCRIPT_DIR"
fi

# Move the Python script to the target directory
echo "Moving ${SCRIPT_NAME} to ${SCRIPT_DIR}..."
mv -f "./${SCRIPT_NAME}" "$SCRIPT_DIR"

# Make the Python script executable
chmod +x "${SCRIPT_DIR}/${SCRIPT_NAME}"

# Write the systemd service file
echo "Writing systemd service file to ${SERVICE_FILE}..."
cat << EOF > "$SERVICE_FILE"
[Unit]
Description=ROS2 Rosbridge Websocket Service
After=network.target

[Service]
Type=simple
ExecStart=${PYTHON_PATH} ${SCRIPT_DIR}/${SCRIPT_NAME}
Restart=always
User=${USER_NAME}
Group=${GROUP_NAME}
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd to apply changes
echo "Reloading systemd daemon..."
systemctl daemon-reload

# Enable the service to start on boot
echo "Enabling ${SERVICE_NAME} service..."
systemctl enable "$SERVICE_NAME"

# Start the service
echo "Starting ${SERVICE_NAME} service..."
systemctl start "$SERVICE_NAME"

# Verify the service
echo "Service status:"
systemctl status "$SERVICE_NAME" --no-pager

echo "Setup complete. The ${SERVICE_NAME} service is now running."
