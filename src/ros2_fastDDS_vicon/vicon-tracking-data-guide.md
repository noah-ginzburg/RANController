# Receiving Vicon Motion Capture Data via ROS 2 and Fast DDS

## Introduction
This guide explains how to receive live motion‑capture data from a Vicon system in ROS 2 over a network using Fast DDS. The setup involves two machines:

- The <strong>Vicon PC</strong>, which is running the Vicon System and is configured to publish motion capture data.</li>
- The <strong>host machine</strong>, which receives this data using the [ros2-vicon-receiver](https://github.com/einstein07/ros2-vicon-receiver) package and creates ROS topics that allows other machines to consume the tracking data.

It is assumed that **ROS 2 is installed on all machines except for the Vicon PC**

## Prerequisites

### Running Fast DDS server
Ensure the Fast DDS Discovery Server is running, as explained in the Fast DDS setup guide. All participating machines should source the setup script pointing to the discovery server. The Vicon PC would be a Windows PC and doesn't need to source this script.

## Setup Instructions

### Step 1: Clone and Build the Vicon Receiver Package
On the machine hosting the Fast DDS server, in a seprate terminal, clone and build the ROS 2 Vicon receiver package based on the version of ROS 2 installed. It is compatible with ROS 2 Dashing, Foxy, Galactic and Humble. Here is the example with ROS 2 Humble:
```bash
cd ~/
git clone -b humble https://github.com/einstein07/ros2-vicon-receiver.git
cd ros2-vicon-receiver
colcon build
```

### Step 2: Configure the IP Address
Find the IP address of the Vicon PC by running <em>ipconfig</em> (on Windows), and update the Vicon receiver launch file on the host machine accordingly:

- Open: `~/ros2-vicon-receiver/vicon_receiver/launch/client.launch.py`.
- Replace the placeholder IP in the `hostname` variable with the IP address of the PC running the Vicon system. (e.g., `hostname = '192.168.3.3'`)


### Step 3: Launch the Vicon Receiver
Source the setup script (created during the [Fast DDS discovery guide](/fast-dds-guide.md)), the Vicon receiver build, and launch the Vicon receiver:
```bash
source ~/setup-ros2-discovery.sh
source install/setup.bash
ros2 launch vicon_receiver client.launch.py
```
(Here, we suppose that `setup-ros2-discovery.sh` is in the home directory)

### Step 4: Verify Topics
Open a new terminal, source the Fast DDS setup script, and list topics:
```bash
source ~/setup-ros2-discovery.sh
ros2 topic list
```
You should now see topics being published by the Vicon system for each subject.

**Note**: Ensure that setup script is sourced and pointing to the Discovery Server in all terminals before running ROS commands.

## Example Subscriber Node

Here is an example of a python node that receives Vicon Data:
```python
import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        vicon_topic = "vicon/default/data"
        
        self.vicon_subscription =  self.create_subscription(
            PositionList,
            vicon_topic,
            self.vicon_callback,
            10
        )
        self.my_id = "id0"
        self.my_position = None
    def vicon_callback(self, msg):
        for i in range(msg.n):
            if msg.positions[i].subject_name == self.my_id:
                print('Id = %s' % self.my_id)
                self.my_position = msg.positions[i]
                break
        print(self.my_position)
        print('done')

```

## Troubleshooting

### Custom Message Type
The messages received through the Vicon system are of a custom type. If you run into type erorrs, you may need to have the support for this in your workspace using the [ros2_interfaces](https://github.com/einstein07/vicon_interfaces) package.

Suppose your ROS 2 workspace folder is called `ros2_ws`, then:

```bash
cd ~/ros2_ws/src
git clone https://github.com/einstein07/vicon_interfaces.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Conclusion
With this configuration, Vicon motion capture data can be received by ROS 2 nodes using Fast DDS. This enables integration of Vicon tracking into robotic systems or real-time monitoring applications with ROS 2.