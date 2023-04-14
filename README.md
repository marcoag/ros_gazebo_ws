# Workshop run at:

- 15-04-2023: [ROS Meetup Singapore](https://www.meetup.com/singapore-ros-meetup/events/292489542/) as part of the [FOSSAsia Summit](https://eventyay.com/e/7cfe0771) [robotics track](https://eventyay.com/e/7cfe0771/schedule?track=Robotics%3A). Branch: https://github.com/marcoag/ros_gazebo_ws/tree/main
- 17-02-2022: [Computer Science Seminars](https://cs.aston.ac.uk/seminars/) at [Aston University](https://www.aston.ac.uk). Branch: https://github.com/marcoag/ros_gazebo_ws/tree/aston_university

# Workshop: Introduction to ROS and Gazebo

It introduces some basics on how to use [ROS](https://www.ros.org) and [Gazebo](http://gazebosim.org) for robotics research and development.
This is intended to serve as a guide to follow up along with the online presentation of the workshop.
Alternatively you could try to follow these steps on your own and play around for some self learning. 
The content of this workshop is havily based on the [ROS tutorials](https://index.ros.org/doc/ros2/Tutorials/) as well as the [Gazebo tutorials](http://gazebosim.org/tutorials).
It is recommended to check these official resources out in order to expand the knowledge on the tools beyond what is covered here.

If you run into problems or have any questions you could visit [ROS answers](https://answers.ros.org) or [Gazebo answers](https://answers.gazebosim.org) at any time to find an answer from the community. Please look for previous questions and verify your issue is not already answered before posting yours.

## How to use this workshop repo:

The recommended way to follow this workshop is to **fork this repo**. That way you can add your notes to this file and commit your own files for later reviews.

Feel free to send Pull Requests with fixes and improvements!

Give a star, follow, like or whatever applies if you find this interesting.

## Abstract:

ROS is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviour across a wide variety of robotic platforms. Gazebo is a robotics simulator that allows to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. In this workshop, we will guide you through a step by step practical session on how to develop robotics software with ROS and how Gazebo can help you test your robotics software so it is ready for the real world once you move it to a real robot.

Given that this is a hands-on seminar, attendants are advised where possible to install ROS2 and Gazebo prior to attending the seminar.

Preferably: Ubuntu (22.04) + ROS Humble (ros-humble-desktop) + Gazebo Garden

ROS 2 Humble (desktop package): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Gazebo Garden: https://gazebosim.org/docs/garden/install_ubuntu

**Pre-installed virtual machine (recommended):** https://drive.google.com/file/d/1gT7C0TBfkQi0dvYoDceRagPC7X-FIRt_/view?usp=sharing

## Prerequisites:

* You are somehow familiar with GNU/Linux based environments and the bash terminal.
* You have basic knowledge of Python or some other form of object oriented programing langugage.
* You have a computer with the required software installed or the provided virtual machine running.

## 1. ROS Overview

You can find the this overview [here](https://docs.google.com/presentation/d/1lqLCteaQou5E9WmedOlYZsTQ5_XGaaEVayGw0RX8LxE/edit?usp=sharing) (slides 1-16).

## 2. Quick review of the install steps

The ROS install instructions can be found [here](https://index.ros.org/doc/ros2/Installation/#installationguide).

For the purpose of this workshop please follow the Desktop Install (ROS, RViz, demos, tutorials):

```
sudo apt install ros-humble-desktop
```

Once installed set up your environment by sourcing the file:

```
source /opt/ros/humble/setup.bash
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
ROS_DISTRO=humble
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
Publisher count: 0
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
Action clients: 0
Action servers: 0ros2 
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

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings and lists. In ROS 2, each node maintains its own parameters. All parameters are dynamically reconfigurable, and built off of ROS 2 services. For more on this check the [ROS 2 paramaters tutorial](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/).
```
ros2 param list
````

## 5. Develeloping with ROS

When working with ROS we use workspaces. A workspace is a directory containing the ROS 2 packages you will develop. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace (with the binaries) in the terminal you plan to work in. This makes ROS 2’s installed packages available for you to use in that terminal.

You also have the option of sourcing an “overlay” – a secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay”. Your underlay must contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.

If a certain ROS software doesn't have a binary package or you want to use the source code version you would have to add it to your workspace and build it as we will do with these packages.

### 5.1 Get the workspace ready:

In ROS2 it is usually a common practice to use `colcon` as the tool to help build your packages.
It is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`.
You can install it from the oficial usual Ubuntu repositories:
```
sudo apt-get install python3-colcon-common-extensions
```

Let's create our workspace:
```
mkdir -p ~/ws/src
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
                'listener = py_pubsub.listener_member_function:main',
        ],
},
```

Now we can build again and run our subscriber too:

```
cd ~/ws/
colcon build
source  install/setup.bash 
ros2 run py_pubsub talker
```

## 6. ROS resources and community

* [ros.org](https://www.ros.org)
* [index.ros.org](https://index.ros.org)
* [answers.ros.org](https://answers.ros.org)
* [discourse.ros.org](https://discourse.ros.org)
