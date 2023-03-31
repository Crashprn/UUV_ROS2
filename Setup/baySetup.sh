#!/bin/bash

# server setup
echo "Starting DHCP server..."
sudo systemctl restart isc-dhcp-server
echo -e "\nDHCP server status: $(sudo systemctl is-active isc-dhcp-server)"

# obtain and print IP addresses
bayIP=$(hostname -I)
uuvInfo=($(dhcp-lease-list --parsable 2> /dev/null)) # split information on DHCP leases into array
uuvIP="${uuvInfo[3]}"
echo -e "\nBay Station IP address: $bayIP"
echo -e "UUV IP address:         $uuvIP\n"

# setup ROS on bay station computer
echo "Setting up bay station ROS workspace and starting arduino interface..."
cd ros_ws
# colcon build
. install/setup.bash
cd ..

# ssh into UUV computer
echo -e "\nSSHing into UUV computer..."
ssh $uuvIP
