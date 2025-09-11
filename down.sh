#!/bin/bash

# Check if can0 is already down
if ! ip link show up | grep -q can0; then
    echo "can0 is already down."
else
    echo "Bringing down can0..."

    # Bring down can0 interface
    sudo ip link set down can0

    echo "can0 has been brought down."
fi
