# UUV Computer Setup
This document explains how to set up the UUV computers. First, both the bay station and UUV computers must be connected 
to power and connected to one another with an ethernet cord. To control the UUV, the controller must be plugged into the 
bay station computer. For other control, a keyboard and monitor must be connected to the bay station computer. The software 
may be set up manually or using a script. Both methods are described below.

## Manual Setup
### DHCP Server Setup
A DHCP server is used for the bay station and UUV computers to communicate with each other. This requires the bay
station computer to assign an IP address to the UUV computer. Below are the steps to configure the server manually. 
I use nano as a text editor, but you can use any you like. `$` indicates the command line.

1. Run `$ ip a`. This gives the server (bay station) IP information. The information under `2:` gives the relevant 
   information. Note the name of the interface, listed `2: <interface>:` and the server IP address, listed
   `inet <IP address>/24`. These values should respectively be `enp2s0` and `192.168.1.202`.
2. Run `$ sudo nano /etc/default/isc-dhcp-server`. Set the interface fields
   equal to the server interface found in the previous step, 
   ```
   INTERFACESv4="enp2s0"
   INTERFACESv6="enp2s0"    
   ```
   Exit the text editor.
3. Run `$ sudo nano /etc/dhcp/dhcpd.conf`. Uncomment the block under
   ```
   # A slightly different configuration for an internal subnet.
   ```
   Modify this block so it matches the values below
   ```
   subnet 192.168.1.0 netmask 255.255.255.0 {
     range 192.168.1.20 192.168.1.254;
     option domain-name-servers server.example.org;
     option domain-name "example.org"
     option subnet-mask 255.255.255.0;
     option routers 192.168.1.1;
     option broadcast-address 192.168.1.255;
     default-lease-time 600;
     max-lease-time 7200;
   }
   ```
   Exit the text editor.
4. Run `$ sudo systemctl restart isc-dhcp-server`. This should cause the server to search for available devices and
   assign them addresses.
5. Run `$ sudo systemctl status isc-dhcp-server` and see that the DHCP server is active.
6. Run `$ dhcp-lease-list`. This should show `uuvcomputer` as a device with an IP lease.
7. Run `$ ssh <UUV IP address>`. Enter the password and you will now be controlling the onboard computer. Use `$ exit` to return to controlling
   the bay station computer.

After completing this process once, you should be able to give the onboard computer an IP address and SSH into it using
only steps 4-7 for future connections.

### ROS Setup
On both the bay station computer and the onboard computer, ROS must be sourced and any nodes must be launched that you desire to use. To 
source ROS, navigate into the root of the ROS workspace, `ros_ws`, and run
`$ . install/setup.bash`

Then, you may launch a node from the root of the workspace by running
`$ ros2 launch <package name> <launch file>`

The launch file must be placed in `ros_ws/src/<node>/launch`. In this case, the node we want to launch is the `arduino_interface` node. To
launch the node, from the root of the ROS workspace, run
`$ ros2 launch nodes.py arduino_interface.launch.py`


## Setup Scripts
Both the bay station computer and UUV computer have a script to perform their setup.

### `baySetup.sh`
This script sets up the bay station computer. To run the script, run `$ ./baySetup.sh` from the home directory of the bay station computer.

The script assigns an IP address to the onboard computer, prints out the IP addresses of both computers, sources ROS on the bay station
computer, launches the `arduino_interface` node, and SSHes into the onboard computer. The user will be prompted for the computer password
twice, once for starting the DHCP server and once for SSHing into the onboard computer. If the script does not print out anything for the 
UUV IP address, the script must be ran again.

### `uuvSetup.sh'
This script sets up the onboard computer. To run the script, run `$ ./uuvSetup.sh` from the home directory of the UUV computer after SSHing into it.

This script prints out the IP address of the UUV computer, sources ROS on the UUV computer, and launches the `arduino_interface` node.
