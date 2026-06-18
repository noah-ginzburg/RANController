# Configuring Fast DDS for communication with ROS 2 topics over multiple machines

## Introduction
This guide explains how to configure Fast DDS Discovery Server to allow ROS topics to be visible across machines in a local network. Useful in scenarios where multiple devices publish and subscribe to one/multiple topics across a network.

To make ROS 2 topics visible across machines:
<ul>
<li>A central Discovery Server must be launched on one machine. Called here the Fast DDS-server machine</li>
<li>Each machine must point to this server using the <em>ROS_DISCOVERY_SERVER</em> variable.</li>
</ul>
A setup script can simplify the configuration for each machine.

## Setup Instructions
### Step 1: Create the Setup Scripts

Let's create the script ```setup-ros2-discovery.sh```(also included in the repository). You’ll need two variants of the this script: one for the server machine and  another for all other machines. The only difference between them is the IP address assigned to ```ROS_DISCOVERY_SERVER```. The script contents are as follows:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<DISCOVERY_SERVER_IP>
export ROS_SUPER_CLIENT=True
```

- On the **Fast DDS‐server machine** (the host you want to run the discovery server on):
    - Use `127.0.0.1:11811` (localhost) for `<DISCOVERY_SERVER_IP>`.
    - Keep this copy on the server machine only.
    - Get the IP address of this machine for the next steps. You could use `ifconfig`, `ipconfig` or `ip a` to find it.

- On the **other (client) machines**, have a client-variant of the script:
    - Replace `<DISCOVERY_SERVER_IP>` with the Fast DDS‐server machine's IP address (e.g. 192.168.1.42).

Note: If any topics are created by the server machine itself, they will be visible across the network given that the setup script (host variant) was sourced before starting the node.

As an **example**, suppose the IP address of the Fast DDS-server machine is `192.168.1.42`, then:
- On the server machine the script looks like:
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    export ROS_SUPER_CLIENT=True
    ```
- On client machine the script looks like:
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER=192.168.1.42:11811
    export ROS_SUPER_CLIENT=True
    ```

### Step 2: Start the Fast DDS Discovery Server

On the **server machine**, open a new terminal and:
```bash
source setup-ros2-discovery.sh # loads the server variant setup script
fastdds discovery --server-id 0 # starts the discovery service
```

**Note**: Any time you open a new terminal, re-run the source command before running any ROS 2 commands.

### Step 3: Source the Setup Script on Clients

On each **client machine**, in every terminal you use for ROS:
```bash
source setup-ros2-discovery.sh # client variant (points to server IP)
```

### Step 4: Launch ROS 2 Nodes

With the server running and all terminals sourced, you can run your ROS 2 nodes or launch files (e.g., your publishers and subscribers).

Now:
- You should be able to create, publish and subscribe to topics across the network.
- `ros2 topic list` should show all the topics on the network.

## Troubleshooting

1. If topics are not visible, try restarting the ROS 2 daemon:
    ```bash
    ros2 daemon stop
    ```
    Then try listing topics again.

2. Make sure the firewall is not blocking port <em>11811</em> on the server machine, if so, use another port.

3. Ensure the setup script is sourced in the terminal before using ROS 2 commands.

<!-- ## Custom Interface

If using custom message types (e.g., from a Vicon system), ensure the corresponding ROS 2 packages are built and sourced on all machines. For example:

```bash
colcon build
source install/setup.bash
```

Failing to source custom interfaces may lead to type errors. -->

## Conclusion
This setup allows ROS 2 nodes on different machines to communicate through a central Fast DDS Discovery Server by making ROS topics discoverable.

