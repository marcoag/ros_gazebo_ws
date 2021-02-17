# Workshop: Introduction to ROS and Gazebo

This workshop has been initially designed for the [Computer Science Seminars](https://cs.aston.ac.uk/seminars/) at [Aston University](https://www.aston.ac.uk). 
It introduces some basics on how to use [ROS](https://www.ros.org) and [Gazebo](http://gazebosim.org) for robotics research and development.
This is intended to serve as a guide to follow up along with the online presentation of the workshop.
Alternatively you could try to follow these steps on your own and play around for some self learning. 
The content of this workshop is based on the [ROS tutorials](https://index.ros.org/doc/ros2/Tutorials/) as well as the [Gazebo tutorials](http://gazebosim.org/tutorials).
It is heavily recommended to check them out in order to expand the knowledge on the tools beyond what is covered here.

If you run into problems or have any questions you could visit [ROS answers](https://answers.ros.org) or [Gazebo answers](https://answers.gazebosim.org) at any time to find a fast and optimal answer.

## How to use this workshop repo:

The recommended way to follow this workshop is to **fork this repo**. That way you can add your notes to this file and commit your own files for later reviews.

## Abstract:

ROS is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviour across a wide variety of robotic platforms. Gazebo is a robotics simulator that allows to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. In this workshop, Dr Gutierrez will guide you through a step by step practical session on how to develop robotics software with ROS and how Gazebo can help you test your robotics software so it is ready for the real world once you move it to a real robot.

Given that this is a hands-on seminar, attendants are advised where possible to install ROS2 and Gazebo prior to attending the seminar.

Preferably: Ubuntu (20.04) + ROS Foxy (ros-foxy-desktop) + Gazebo 11

ROS Foxy (desktop package): https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

Gazebo 11: http://gazebosim.org/tutorials?tut=install_ubuntu

Alternatively: Ubuntu (18.04/20.04) + ROS Dashing (ros-dashing-desktop) + Gazebo 11

ROS Dashing (desktop pacakge): https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

Gazebo 11: http://gazebosim.org/tutorials?tut=install_ubuntu 

**Pre-installed virtual machine (recommended):** https://drive.google.com/file/d/1gT7C0TBfkQi0dvYoDceRagPC7X-FIRt_/view?usp=sharing

## Prerequisites:

* You are somehow familiar with GNU/Linux based environments and the bash terminal
* You have basic knowledge of Python  or some other form of object oriented programing langugage
* You have a computer with the required software installed or the provided virtual machine running

## 1. ROS Overview

You can find the this overview [here](https://docs.google.com/presentation/d/1MWuh-OQoFF2c5AxdfburZOnkCEHG8cknO582183rrLE/edit?usp=sharing) (slides 1-13).

## 2. Quick review of the install steps

The ROS install instructions can be found [here](https://index.ros.org/doc/ros2/Installation/#installationguide).

For the purpose of this workshop please follow the Desktop Install (ROS, RViz, demos, tutorials):

```
sudo apt install ros-foxy-desktop
```

Once installed set up your environment by sourcing the file:

```
source /opt/ros/foxy/setup.bash
```

You can make this automated for every bash you open with:

```
echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
```

You can now check the version of your ROS distribution:
```
$ env|grep ROS
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=foxy
```

In order to install further ROS packages:

```
sudo apt install ros-<distro>-<package> command 
```

## 3. Understanding interfaces

ROS applications typically communicate through interfaces of one of three types: messages, services and actions. ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

* msg: .msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
* srv: .srv files describe a service. They are composed of two parts: a request and a response. The request and response are message declarations.
* action: .action files describe actions. They are composed of three parts: a goal, a result, and feedback. Each part is a message declaration itself.

## 4. Pub/sub, services and actions

Let's run our first two nodes:
```
ros2 run turtlesim turtlesim_node
```

````
ros2 run turtlesim turtle_teleop_key
````

### 4.1 Publish and subscribe

Let's have a look at what's going on. For this you can run:
```
rqt
```
And select Plugins > Introspection > Nodes Graph. Or you can just run:
```
rqt_graph
```
Now let's check our nodes and topics through the CLI tools:
```
ros2 node list
ros2 topic list
ros2 topic list -t
```

We what's going on in a topic:
```
ros2 topic echo /turtle1/cmd_vel
```
Note: /_ros2cli_<pid> is the node created by the echo

To check the info of a topic:

```
ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscriber count: 2
```

And to check the interface:
```
ros2 interface show geometry_msgs/msg/Twist

geometry_msgs/msg/Twist

# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
    Vector3  angular
```

We can also publish using the ROS CLI tools:
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Let's add some frequency:
```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

You can also check the frequency of messages on a certain topic:

```
ros2 topic hz /turtle1/cmd_vel
```
Note that the average rate is 1hz as we set in the ROS CLI command.

### 4.2 Services

You can check services from the ROS CLI with:
```
ros2 service list
```

Let's check rqt service caller plugin:
```
rqt
```
Select Plugins > Services > Service Caller

We can check the type of a service:
```
ros2 service type /clear
std_srvs/srv/Empty
```
We can also check the type of each service:
```
ros2 service list -t
```

Or which services use a certain srv interface:
```
ros2 service find std_srvs/srv/Empty
```

We can check the details of an srv interface:
```
ros2 interface show std_srvs/srv/Empty
```

Or when checking on one with a bit more fields:
```
ros2 interface show turtlesim/srv/Spawn

float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

And again we can also call services using the ROS CLI tools:
```
ros2 service call /clear std_srvs/srv/Empty
```

Another example:
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

### 4.3 Actions

We can check available actions with:
```
ros2 action list
/turtle1/rotate_absolute
```

Same as previously you can check the interface of each action:
```
ros2 action list -t

/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

A summary of the information related to an action:
```
ros2 action info /turtle1/rotate_absolute

Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

Description of the action interface:
```
ros2 interface show turtlesim/action/RotateAbsolute.action

# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
The first section of this message, above the ---, is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

You can sue the ROS CLI commands to send an action to a node:
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```

To see the feedback of this goal, add --feedback to the last command you ran. 
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

### 4.4 Extra CLI commands

Checking the info of a node:
```
ros2 node info
```

Checking ROS health status:
```
ros2 doctor --report
```

Record and play topics:
```
ros2 bag
```

Launch several nodes with some specific configs:
```
ros2 launch
```

## 5. Develeloping with ROS

When working with ROS we use workspaces. A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

You also have the option of sourcing an “overlay” – a secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay”. Your underlay must contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.

If a certain ROS software doesn't have a binary package or you want to use the source code version you would have to add it to your workspace and build it as we will do with these packages.

### 5.1 Get the workspace ready:

In ROS2 it is usually a common practice to use `colcon` as the tool to build your packages.
It is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`.
You can install it from the oficial usual Ubuntu repositories:
```
sudo apt-get install python3-colcon-common-extensions
```

Let's create our workspace:
```
mkdir ~/ws/src
cd ~/ws/src
```

In order to create a python package:
```
ros2 pkg create --build-type ament_python py_pubsub
```

### 5.2 Writing a publisher:

Let's edit our publisher file:
```
cd py_pubsub/py_pubsub
vim publisher_member_function.py
```

Add this to it:
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Add the dependencies to your `package.xml`:
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Add the following line within the console_scripts brackets of the entry_points field of your `setup.py`:
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

Now you can build your workspace:

```
cd ..
colcon build 
```

Then you can source the workspace and run the publisher:

```
source install/setup.bash
ros2 run py_pubsub talker
```

### 5.3 Writing a subscriber:

Edit your subscriber file:
```
cd src/py_pubsub/py_pubsub/
vim listener_member_function.py
```

Add this to the file:
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Reopen `setup.py` and add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

Now we can build again and run our subscriber too:

```
cd ~/ws/
colcon build
ros2 run py_pubsub talker
```

## 6. ROS resources and community

* [ros.org](https://www.ros.org)
* [index.ros.org](https://index.ros.org)
* [answers.ros.org](https://answers.ros.org)
* [discourse.ros.org](https://discourse.ros.org)

## 7. Gazebo Overview

You can find the this overview [here](https://docs.google.com/presentation/d/1MWuh-OQoFF2c5AxdfburZOnkCEHG8cknO582183rrLE/edit?usp=sharing) (slides 14-19).

## 8. Quick Review of install steps

You can find the Gazebo installation instructions [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

For the purpose of this workshop the latest version (gazebo 11) is recommended. For this you can just follow the one-liner default installation:
```
curl -sSL http://get.gazebosim.org | sh
```

## 9. Gazebo basics and GUI

You can run gazebo by:
```
gazebo
```

Gazebo has a server and a client that can be run independently. You can try running the server with:
```
gzserver
```
And the client:
```
gzclient
```
At this point you should see the Gazebo user interface. You restart the gzclient application as often as you want, and even run multiple interfaces.

### 9.1 Worlds

The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects. This file is formatted using [SDF (Simulation Description Format)](http://gazebosim.org/sdf.html), and typically has a .world extension.

There are several demo worlds installed at:
```
ls /usr/share/gazebo-11/worlds/
```
You can load them in gazebo like this:
```
gazebo worlds/shapes_layers.world
```

### 9.2 Models and Actors

#### Models

Gazebo is able to dynamically load models into simulation either programmatically or through the GUI. Models exist on your computer, after they have been downloaded or created by you. 

You can find models at the [Gazebo Model database](https://github.com/osrf/gazebo_models) as well as on [ignition fuel](https://app.ignitionrobotics.org/fuel).

The model database is a GitHub repository found [here](https://github.com/osrf/gazebo_models).

More info on Gazebo's model directory structure, and the necessary files within a model directory can be found [here](http://gazebosim.org/tutorials?tut=model_structure).

The SDF format can also be used to define models as described in [this tutorial](http://gazebosim.org/tutorials?tut=build_model).

#### Actors

In Gazebo, an animated model is called an actor. Actors extend common models, adding animation capabilities.
There are two types of animations which can be used separately or combined together:

* Skeleton animation, which is relative motion between links in one model.
* Motion along a trajectory, which carries all of the actor's links around the world, as one group.
* Both types of motions can be combined to achieve a skeleton animation which moves in the world.

For example loads an box moving in a square trajectory again and again:
```
gazebo worlds/animated_box.world
```
The trajectory goes through four points in the world ([-1, -1, 1], [-1, 1, 1], [1, 1, 1] and [1, -1, 1]) and takes 1 s in between them.


We can also create our own with `vim walk.world` and let's add:

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <actor name="actor">
      <skin>
        <filename>walk.dae</filename>
      </skin>
    </actor>
  </world>
</sdf>
```

Now we can test it out:
```
gazebo walk.world
```
The actor in the example above is really simple, all it loads is a COLLADA file described within the <skin> tag.
The animation tag goes alongside the skin tag, and it takes a name parameter.

You can also try with the `moonwalk.dae` animation.

For example, the files walk.dae and moonwalk.dae are compatible so they can be mixed with each other:

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <actor name="actor">
      <skin>
        <filename>walk.dae</filename>
      </skin>
      <animation name="animation">
        <filename>moonwalk.dae</filename>
      </animation>
    </actor>
  </world>
</sdf>
```

Now let's combine trajectories and static animations:

```
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <actor name="actor">
      <skin>
        <filename>walk.dae</filename>
      </skin>
      <animation name="animation">
        <filename>walk.dae</filename>
      </animation>
      <script>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0</time>
            <pose>0 2 0 0 0 -1.57</pose>
          </waypoint>
          <waypoint>
            <time>2</time>
            <pose>0 -2 0 0 0 -1.57</pose>
          </waypoint>
          <waypoint>
            <time>2.5</time>
            <pose>0 -2 0 0 0 1.57</pose>
          </waypoint>
          <waypoint>
            <time>7</time>
            <pose>0 2 0 0 0 1.57</pose>
          </waypoint>
          <waypoint>
            <time>7.5</time>
            <pose>0 2 0 0 0 -1.57</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
  </world>
</sdf>
```

The actor's leg are not moving because the animation name has to match the trajectory type:

```
     <animation name="walking">
        <filename>walk.dae</filename>
      </animation>
```

It's still not looking good, we need to synchronize the animation with the trajectory. 
To do this e can enable that by setting <interpolate_x> to true inside <animation>:

```
     <animation name="walking">
        <filename>walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
```

### 9.3 Connect from your source code

Let's create our little listener project directory:
```
mkdir ~/listener/
cd ~/listener
```

Now edit an empty `listener.cc` file and add:

```
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstWorldStatisticsPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo and run the transport system
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
```

We edit `CMakeLists.txt` file and add this:

```
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(listener listener.cc)
target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)
```
Then just build like you will do with any `cmake` based project:

```
mkdir build
cd build
cmake ..
make
```

You can go ahead and execute the resulting binary to checkout the results:

```
./listener
```

### 9.4 Plugins

Plugins provide a simple and convenient mechanism to interface with Gazebo. Plugins can either be loaded on the command line, or specified in an SDF file (see the SDF format).

There are currently 6 types of plugins

1. World
2. Model
3. Sensor
4. System
5. Visual
6. GUI

For more depth on this you can check the [Gazebo plugins tutorial](http://gazebosim.org/tutorials/?tut=plugins_hello_world).

Now we can check how a plugin is loaded. You might need to install the ros-gazebo plugins:
```
sudo apt-get install ros-foxy-gazebo-plugins
```

You can find the plugin load instruction on the SDF world file:
```
vim /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

Let's run the ros differential drive demo:
```
gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

## 10. ROS and Gazebo

You can publish to the `/demo/cmd_demo` topic to start moving the differential model:
```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```
Also rotating it:
```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{angular: {z: 0.1}}' -1
```
You can inspect the topics from the ROS CLI:
```
ros2 topic echo /demo/odom_demo
```
Or inspect the tf:
```
ros2 run tf2_ros tf2_echo odom_demo chassis
```
`tf` is the ROS transform library, which lets the user keep track of multiple coordinate frames over time.

You can do this fir the right wheel:
```
ros2 run tf2_ros tf2_echo chassis right_wheel
```
Or the left wheel:
```
ros2 run tf2_ros tf2_echo chassis left_wheel
```

## 11. Gazebo resources and community

* [gazebosim.org](http://gazebosim.org/)
* [ignitionrobotics.org](https://ignitionrobotics.org/)
* [answers.gazebosim.org](https://answers.gazebosim.org)
* [community.gazebosim.org](https://community.gazebosim.org/)

## 12. How to join the community and contribute

A summary of ROS and Gazebo resources and community links can be found on the [slides](https://docs.google.com/presentation/d/1MWuh-OQoFF2c5AxdfburZOnkCEHG8cknO582183rrLE/edit?usp=sharing) (21-23).

### ROS contributions:
* https://index.ros.org/doc/ros2/Contributing/

### Gazebo Contributions:
* http://gazebosim.org/tutorials?tut=contrib_code&cat=development
