#!/bin/bash

# Check if can0 is up
# if ! ip link show up | grep -q can0; then
echo "Bringing up can0..."

# Load CAN related modules
sudo modprobe can
sleep 0.1
sudo modprobe can_raw
sleep 0.1
# Set the bitrate for can0
sudo ip link set can0 type can bitrate 500000
sleep 0.5
sudo ip link set up can0
# Bring up can0 interfac
echo "can0 setup complete."
# else
#     echo "can0 is already up."
# fi
