# ESP32 + Husarnet + Rosserial

This project shows you how to use ESP32 together with Robot Operating System (ROS). ROS is a leading middleware for developing software for autonomous robots. ROS runs on Linux however by using rosserial you can integrated with ROS microcontroller based systems. A single ROS project can be distributed among multiple computers as long as they are in the same LAN network. That limitation is fixed thanks to using Husarnet.

In this project I will show you how to use run rosserial on ESP32 to connect that popular and affordable Wi-Fi microcontroller with the ROS powered system over the internet.

# Setup husarnet on your computer
To make husarnet working on your computer is very simple to do so you have to install it and change `.bashrc` file.

1. Install husarnet 
```
curl https://install.husarnet.com/install.sh | sudo bash
```

Right after installing Husarnet, you should link the device to your Husarnet Dashboard account.

```
sudo husarnet websetup
```

You will need to login/sign-up and then you will be presented with the following dialog:

![add_husarnet_device](https://user-images.githubusercontent.com/29305346/62351992-55c6c800-b507-11e9-9568-86df90eba6d1.png)

You can create a new network or join existing one (if you already have one before). All devices should be connected to some Husarnet network - by default, only devices in the same network are able to communicate.

## Setting up the environment

Add these lines to .bashrc (or .zshrc if you use zsh) of the user who will use ROS:

```
source /opt/ros/kinetic/setup.bash

export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
```

Sourcing the /opt/ros/kinetic/setup.bash enables all ROS tools. 

# Preparing ESP32 firmware

In this tutorial it is assumed that you already have ROS installed on your computer running Ubuntu 16.04 and ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu ) or Ubuntu 18.04 with ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu).

If you have ROS already installed you can follow next instructions
 

## Arduino IDE

Open Arduino IDE, and follow these instruction steps:

### Install Husarnet IDF for ESP32:
In arduion IDE:

1. Open File -> Preferences
2. In a field Additional Board Manager URLs add this link: 

`https://files.husarion.com/arduino/package_esp32_husarnet_index.json`

![setup_arduion_ide](https://user-images.githubusercontent.com/29305346/62209500-1169e980-b39a-11e9-9b5c-4f564cc44ee6.png)


### Install ESP32 dev boards

- Open Tools -> Board: "..." -> Boards Manager ...
- Search for esp32-husarnet
- Click Install button

![install_board](https://user-images.githubusercontent.com/29305346/62209622-6574ce00-b39a-11e9-9f58-a4ffe2d618ae.png)

### Select ESP32 dev board:

- Open Tools -> Board "..."
- Select ESP32 Dev Module under ESP32 Arduino (Husarnet) section

![select_board](https://user-images.githubusercontent.com/29305346/62209768-cf8d7300-b39a-11e9-8f52-45c2496b3030.png)

## Setup Rosserial

Once we have it working we need to setup rosserial to works with Husarnet - this will enable us to use ESP32 microcontrollers for sending information to devices even tough they are not in the same network.
To make it works we need to remove current version and install the one made for husarnet

```
$ sudo apt-get remove ros-kinetic-rosserial*
```
Then go to my github and clone to workspace repository from `ipv6-husarnet branch` [ipv6-husarnet](https://github.com/adamkrawczyk/rosserial/tree/ipv6-husarnet) or use following command:

```bash
$ cd ~/ros_workspace/src
$ git clone --single-branch --branch ipv6-husarnet https://github.com/adamkrawczyk/rosserial.git
```


Build this with `$ catkin_make install` . This should work so now we are able to setup arduino IDE to do that follow official instructions:

# Install ros_lib into the Arduino Environment

In the steps below, <sketchbook> is the directory where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory.

If only you have ros_lib you must delete libraries/ros_lib in order to regenerate as its existence causes an error. 

```
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
 ```

After restarting your IDE you should have it working.

# Main code

If we have husarnet set up we can create new sketch for our arduino project:

1. In your sketchbook create new directory called esp32_rosserial_demo
2. In this folder create file called esp32_rosserial_demo.ino
3. Paste code from git repository:

[esp32_rosserial_demo](https://github.com/adamkrawczyk/esp32_rosserial_demo)

Paste this code to file we have just created.

Now you have to make this code customized. To make it working you have to change some lines. But before we need to have Husarnet join code. This is also pretty simple. Just stick to this steps:

### Get Husarnet join code

1. Register at https://app.husarnet.com/
2. Click Create network button, name it (eg. mynet), and click Create button
3. Click Add element button and go to the join code tab
4. Copy your join code

![husarnet_join_code](https://user-images.githubusercontent.com/29305346/62353388-d9ce7f00-b50a-11e9-9359-df9eb8dbfaf9.png)

### Customize code

Now you have to customize our esp code.  

In lines `16-17` change host names to appropriate
```
const char* hostNameESP = "****"; 
const char* hostNameComputer = "*****";
```

Next step is to change husarnet join code in line `25`
```
const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/******";
```

Finally change lines `27-35`
```
#define NUM_NETWORKS 2 //number of Wi-Fi network credentials saved
const char* ssidTab[NUM_NETWORKS] = {
  "****",         //wifi name
  "*****",
};
const char* passwordTab[NUM_NETWORKS] = {
  "******",      //wifi password   
  "******",
};
```

At the end we have to create launch for esp enabling us to read data it sends, but this will be done at computer:


## Rosrun 
We can run this with following command on host:

```
rosrun rosserial_python serial_node.py tcp 11411
```

## Roslaunch

In this file paste following launch:
Only what you have to take care of is to set appropirate port for each device I suggest to use 11411, 11412 and so on remember to it the same as in esp code.

```xml
<launch >

<node pkg="rosserial_python" type="serial_node.py" name="esp_client1" respawn="true">
<param name="port" value="tcp"/>
<param name="tcp_port" value="11411"/>
</node>

</launch>
```

If you need more clients just add it.

# Summary

At this stage you should see in the terminal messages sent by ESP32 to your ROS computer:

After command `rosrun rosserial_python serial_node.py tcp 11411`

![result1](https://user-images.githubusercontent.com/29305346/62276514-842f9f00-b444-11e9-9c49-d5881f24eb2b.png)

If you type `rostopic list` and `rostopic echo /esp_husarnet` the following data should appear

![result2](https://user-images.githubusercontent.com/29305346/62276516-84c83580-b444-11e9-8eb0-69dfb02d4da2.png)



Of course that is a simple demo showing you how to integrate  microcontrollers and ROS computers over the internet. Now you know how to use affordable Wi-Fi microcontrollers providing for example sensor data to the the robotic system based on ROS.
