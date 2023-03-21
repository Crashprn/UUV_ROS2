#!/bin/bash

echo "Starting DHCP server..."
sudo systemctl start isc-dhcp-server

echo -e "\nDHCP server status: $(sudo systemctl is-active isc-dhcp-server)"

bayIP=$(hostname -I)
onboardInfo=$(dhcp-lease-list --parsable 2> /dev/null) # get information on DHCP leases
onboardInfo=($(grep -w "IP" <<< $onboardInfo)) # split line with IP address into array
onboardIP="${onboardInfo[3]}"

echo -e "\nBay Station IP: $bayIP"
echo -e "Onboard IP: $onboardIP\n"
