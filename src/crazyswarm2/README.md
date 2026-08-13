* Local CI: [![ROS 2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)
* Rolling Dev CI : [![Build Status](https://build.ros2.org/job/Rdev__crazyflie__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__crazyflie__ubuntu_noble_amd64/)
* Jazzy Dev CI: [![Build Status](https://build.ros2.org/job/Jdev__crazyswarm2__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__crazyswarm2__ubuntu_noble_amd64/)
* Humble Dev CI: [![Build Status](https://build.ros2.org/job/Hdev__crazyswarm2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__crazyswarm2__ubuntu_jammy_amd64/)


# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots, with the necessary configurations to make the setup easier here at SML.

The documentation is available here: https://imrclab.github.io/crazyswarm2/. Apart from help in our lab, you can ask for help in their [Discussion](https://github.com/IMRCLab/crazyswarm2/discussions) forum.

## Information

At SML, we are soon going to have 12 Crazyflies 2.1+ with LED ring deck. This repository contains all setup files necessary for using them.

## Installation

Follow the instructions of the [Crazyswarm2 library](https://imrclab.github.io/crazyswarm2/installation.html) up until step 2:

```
sudo apt install libboost-program-options-dev libusb-1.0-0-dev
pip3 install rowan nicegui
sudo apt-get install ros-$ROS_DISTRO-motion-capture-tracking
pip3 install cflib transforms3d
sudo apt-get install ros-$ROS_DISTRO-tf-transformations
```

If you have problems with installing the motion_capture_tracking package, you can install it from source in your workspace (see [motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking))

Create a ros workspace and clone this repo
```
mkdir ~/crazyswarm_ws && cd ~/crazyswarm_ws
mkdir src && cd ./src

git clone git@github.com:KTH-SML/SML-crazyswarm2.git --recursive

# optionally
git clone git@github.com:IMRCLab/motion_capture_tracking.git
```

Build your ROS workspace
```
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Set up the Crazyradio:

For the crazyradio, you need to setup usb rules in order to communicate with the Crazyflie. Find the instructions for that here in [Bitcraze’s USB permission guide](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/) for Linux.

Set up software-in-the-loop simulation (optional): 

Check step 7 in the [installation instructions](https://imrclab.github.io/crazyswarm2/installation.html).

## Usage

For communicating with the Crazyflies, you can use
- The Crazywarm2 library for autonomous flying (that's what you cloned here)
- The CFClient with joystick (Attention: This requires some training. Also take care to not have a Crazyflie connected when choosing the appropriate keymapping - The drone might start flying before it should)
- The Crazyflie Android/iOS app - as the joystick solution, this is hard to control and only good for debugging

You need one Crazyradio 2.0 antenna with up-to-date firmware connected to your computer. Currently (Aug 2025), both radios are flashed with recent firmware.

### Crazyswarm library

For general use, refer to the [official documentation](https://imrclab.github.io/crazyswarm2/index.html). 

To test the setup, do the following:
1. Start the Crazyswarm ROS2 node
   ```
   ros2 launch crazyflie launch.py backend:=cflib topics.poses.qos.mode:=sensor
   ```
2. Start the hello_world script
   ```
   ros2 run crazyflie_examples hello_all
   ```
   
TODO: 
- [ ] Have Bjarnes code as an example?

## Debugging

1. One (or multiple) crazyflie repeatedly flips directly after start
	- Probably the drone is not well balanced: Try to place the battery and cable so that the battery is well-balanced. You can verify good balancing by holding the crazyflie on the two notches on the side.
	- This is always the first thing you should check
	- If you are unsure, you can verify that this is the issue by logging a custom topic through the `crazyflies.yaml` and see if the rpy-control input gets into saturation (Update: the custom topic `low_level_control` now is included by default in the  `crazyflies.yaml`)
	- Be aware of additional payload: This requires higher thrust, therefore the thrusters end up sooner in saturation, and possibly crash.
 	- Even the mocap markers are heavy enough to bring the crazyflies out of balance. The current marker-setup is thus near-symmetric and requires to (1) use `librigidbodytracker` (see section "Positioning") and (2) place the crazyflies at predefined initial positions. According to the crazyswarm doc, the payload could be further reduced by using a single marker-setup. In this case, however, the yaw is unknown at startup, and it has not been tried yet.
2. All crazyflies are crashing at the same time
	- Log the positions of one (or multiple) crazyflies. If the positions experience high drift (in range of meters), restart the MoCap system. Make sure the 'Crazyflie' project is started. 
3. ... (feel free to add...)

## Positioning

The library supports both position estimates from Qualisys directly, and own estimates using `librigidbodytracker`. 

The default configuration is using `librigidbodytracker`. If you don't have special reasons, the simplest is to go along with `librigidbodytracker`.

#### librigidbodytracker

`librigidbodytracker` is used currently, as it supports multiple crazyflies with the same marker setup. Each crazyflie has a unique number on one of the arms. 
- Select the crazyflies that you are using in `crazyflies.yaml` by setting `enabled: true`.
- Select the starting positions for each crazyflie also in `crazyflies.yaml`.
The crazyflie must start within a couple of centimeters from the specified starting location, otherwise, they will not be found.

#### Qualisys

To use Qualisys positioning directly, the following needs to be changed:
1. In `crazyflies.yaml`, choose
   ```
   robot_types:
  	cf21:
      motion_capture:
	    tracking: "vendor" # instead of "librigidbodytracker"
   ```
2. In Qualisys, go to settings. Under `Processing` > `Real-time Action`, check `Calculate 6DOF`. Confirm with `Apply`.
3. _Optional_: Depending on your maker setup, you might need to add it to Qualisys
4. Make sure that the name of the frame in Qualisys is the same as the drone name in `crazyflies.yaml`.
5. As before, select the crazyflies that you are using in `crazyflies.yaml` by setting `enabled: true`.
From there on, it works as before. You don't need to start the drone from a specified starting position any more, but each drone needs a unique marker arrangement.

#### Overview of capabilities librigidbodytracker vs Qualisys

Default: `librigidbodytracker`

Pro:
- Supports multiple crazyflies with the same marker setup
- Already set up

Contra:
- Crazyflies need to start at specified starting locations

Alternative: Use Qualisys directly

Pro:
- Crazyflies can start anywhere

Contra:
- Needs unique marker setups (limited space and payload of crazyflies)
- Thus limited to one or few drones
- (+ you will need to set new marker setups)

## Radio channels

The standard address for each crazyflie is 0xE7E7E7E7E7, it can be changed in the [CFClient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client). Currently, they are set to

| Crazyflie No. | Radio channel |
|---------------|---------------|
| cf01          | 0xE7E7E7E701  |
| cf02          | 0xE7E7E7E702  |
| cf03          | 0xE7E7E7E703  |

etc. up until

| Crazyflie No. | Radio channel |
|---------------|---------------|
| cf12          | 0xE7E7E7E712  |

(Note: "10" to 12" are written as decimal numbers in the radio channel, not in hexadecimal - we have 12 crazyflies, not x12)

## Charging

The Crazyflie can directly be plugged into a USB port for charging, or use the USB charging stations. 
If we decide to buy additional batteries later, there are separate chargers for these available.

You can see the carge status by the blinking blue light: The longer the blink, the higher the charge percentage.
