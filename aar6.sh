#!/bin/bash

# Function to list ACM devices
list_acm_devices() {
    ACM_DEVICES=($(ls /dev/ttyACM* 2>/dev/null))
    if [ ${#ACM_DEVICES[@]} -eq 0 ]; then
        echo "❌ No /dev/ttyACM* devices found."
        return 1
    fi

    if [ ${#ACM_DEVICES[@]} -eq 1 ]; then
        echo "✅ Only one device found: ${ACM_DEVICES[0]}"
        DEVICE="${ACM_DEVICES[0]}"
    else
        echo "Available /dev/ttyACM* devices:"
        for i in "${!ACM_DEVICES[@]}"; do
            echo "  [$i] ${ACM_DEVICES[$i]}"
        done
        # Ask user to select device
        read -p "Select device index to use: " DEVICE_INDEX
        if ! [[ "$DEVICE_INDEX" =~ ^[0-9]+$ ]] || [ "$DEVICE_INDEX" -ge "${#ACM_DEVICES[@]}" ]; then
            echo "❌ Invalid selection."
            exit 1
        fi
        DEVICE="${ACM_DEVICES[$DEVICE_INDEX]}"
    fi

    echo "✅ Selected device: $DEVICE"
    export ROBOT_DEVICE="$DEVICE"
    return 0
}

# Check if user passed "--sim"
if [[ "$1" == "--sim" ]]; then
    echo "⚠️  SIMULATION MODE: Skipping device check."
    DEVICE="/dev/null"
    export ROBOT_DEVICE="$DEVICE"
else
    if ! list_acm_devices; then
        echo "Tip: Run '$0 --sim' to bypass this check for simulation."
        exit 1
    fi
fi

# Allow X11 forwarding
xhost +local:root

# Make init_host.sh executable
if [ -f "Robot_ros2_ws/init_host.sh" ]; then
    echo "🔧 Setting executable permissions for init_host.sh..."
    chmod +x Robot_ros2_ws/init_host.sh
else
    echo "⚠️  WARNING: Robot_ros2_ws/init_host.sh not found!"
fi

# Start Docker Compose
docker compose up -d
echo "🚀 Services started."

# Attach to logs
echo "📜 Attaching to logs (Press Ctrl+C to stop watching logs, containers will keep running)..."
docker compose logs -f
