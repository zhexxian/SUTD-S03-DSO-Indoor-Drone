import os
import rospkg
import rospy
import tf

from crazyflie_driver.msg import Command, int_array, Pose2D, equation, equation_array
from geometry_msgs.msg import Pose, PoseArray

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QSlider
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import QtGui, QtCore

class Publisher(Plugin):
    
    def __init__(self, context):
        super(Publisher, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Publisher')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_publisherpkg'), 'resource', 'Publisher.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PublisherUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # set name of topic to publish to
        self.topicName = "/human_command"
        # initialize ROS message
        self.msg = Command()
        # start commandNum as 0 and increase by 1 every time a command is called
        self.msg.CommandNum = 0

        self._widget.publishButton1.clicked.connect(self.publishMsg1)
        self._widget.publishButton2.clicked.connect(self.publishMsg2)
        self._widget.publishButton3.clicked.connect(self.publishMsg3)
        self._widget.publishButton4.clicked.connect(self.publishMsg4)
        self._widget.publishButton5.clicked.connect(self.publishMsg5)
        self._widget.publishButton6.clicked.connect(self.publishMsg6)
        self._widget.publishButton7.clicked.connect(self.publishMsg7)
        self._widget.publishButton8.clicked.connect(self.publishMsg8)           

    # publishMsg method publishes a ROS message with user-defined parameters (upon button click):
    # int8 CommandNum
    # int8[] CommandType
    # int8 SimulationType
    # geometry_msgs/PoseArray InitPose
    # geometry_msgs/PoseArray DirectGoal
    # float32[] Time
    # int_array[] Squad
    # Pose2D[] SquadPos
    # float32[] Gap
    # int8[] Circle
    # int8[] DirectGoalCommandProperties
    # float32[] RotationRate
    # equation[] SquadEquation
    # float32[] theta_start
    # float32[] theta_end
    def publishMsg1(self):
        # for CommandType parameter
        # INITIATE_SQUADS = 0
        # CHANGE_SQUAD_COMMAND = 1
        # DIRECT_GOAL_COMMAND = 2
        # KILL_SQUAD = 3
        # CONGO_LINE = 4
        # STOP_CONGO = 5
        # EQUATION = 6
        # RESET_SIMULATION = 7
        # RUN_SIMULATION = 8
        self.msg.CommandType = [2,2,2,2]

        # for Time parameter
        self.msg.Time = [1.5,1.5,1.5,1.5]

        # for SimulationType parameter
        self.msg.SimulationType = 1

        # for DirectGoal parameter
        self.dirGoalArr = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 2.5
        self.pose1.position.y = 1.45
        self.pose1.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose1)
        self.pose2 = Pose()
        self.pose2.position.x = 3.5
        self.pose2.position.y = 1.45
        self.pose2.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose2)
        self.pose3 = Pose()
        self.pose3.position.x = 2
        self.pose3.position.y = 2.316
        self.pose3.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose3)
        self.pose4 = Pose()
        self.pose4.position.x = 2
        self.pose4.position.y = 0.584
        self.pose4.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose4)

        self.msg.DirectGoal = self.dirGoalArr
        self.msg.DirectGoal.header.stamp = rospy.Time.now()

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [0,0,0,0]

        # for RotationRate parameter
        self.msg.RotationRate = [0,0,0,0]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published command 1 - send direct goal'

    def publishMsg2(self):
        # for CommandType parameter
        self.msg.CommandType = [2,6,6,6]

        # for Time parameter
        self.msg.Time = [15,15,15,15]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for DirectGoal parameter
        self.dirGoalArr = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 2.5
        self.pose1.position.y = 1.45
        self.pose1.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose1)
        self.pose2 = Pose()
        self.pose2.position.x = 3.5
        self.pose2.position.y = 1.45
        self.pose2.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose2)
        self.pose3 = Pose()
        self.pose3.position.x = 2
        self.pose3.position.y = 2.316
        self.pose3.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose3)
        self.pose4 = Pose()
        self.pose4.position.x = 2
        self.pose4.position.y = 0.584
        self.pose4.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose4)

        self.msg.DirectGoal = self.dirGoalArr
        self.msg.DirectGoal.header.stamp = rospy.Time.now()

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [0,0,0,0]

        # for RotationRate parameter
        self.msg.RotationRate = [0,0,0,0]

        # for SquadEquation parameter
        self.eqn1 = equation()
        self.eqn1x = equation_array()
        self.eqn1x.finalpoint, self.eqn1x.offset, self.eqn1x.sine, self.eqn1x.cosine, self.eqn1x.finalcosine, self.eqn1x.sine = 2.5,2.5,0,1,1,0
        self.eqn1y = equation_array()
        self.eqn1y.finalpoint, self.eqn1y.offset, self.eqn1y.sine, self.eqn1y.cosine, self.eqn1y.finalcosine, self.eqn1y.sine = 1.45,1.45,1,0,0,1
        self.eqn1z = equation_array()
        self.eqn1z.finalpoint, self.eqn1z.offset, self.eqn1z.sine, self.eqn1z.cosine, self.eqn1z.finalcosine, self.eqn1z.sine = 1.5,1.5,0,0,0,0

        self.eqn2 = equation()
        self.eqn2x = equation_array()
        self.eqn2x.finalpoint, self.eqn2x.offset, self.eqn2x.sine, self.eqn2x.cosine, self.eqn2x.finalcosine, self.eqn2x.sine = 2.5,2.5,0,1,1,0
        self.eqn2y = equation_array()
        self.eqn2y.finalpoint, self.eqn2y.offset, self.eqn2y.sine, self.eqn2y.cosine, self.eqn2y.finalcosine, self.eqn2y.sine = 1.45,1.45,1,0,0,1
        self.eqn2z = equation_array()
        self.eqn2z.finalpoint, self.eqn2z.offset, self.eqn2z.sine, self.eqn2z.cosine, self.eqn2z.finalcosine, self.eqn2z.sine = 1.5,1.5,0,0,0,0

        self.eqn3 = equation()
        self.eqn3x = equation_array()
        self.eqn3x.finalpoint, self.eqn3x.offset, self.eqn3x.sine, self.eqn3x.cosine, self.eqn3x.finalcosine, self.eqn3x.sine = 2.5,2.5,0,1,1,0
        self.eqn3y = equation_array()
        self.eqn3y.finalpoint, self.eqn3y.offset, self.eqn3y.sine, self.eqn3y.cosine, self.eqn3y.finalcosine, self.eqn3y.sine = 1.45,1.45,1,0,0,1
        self.eqn3z = equation_array()
        self.eqn3z.finalpoint, self.eqn3z.offset, self.eqn3z.sine, self.eqn3z.cosine, self.eqn3z.finalcosine, self.eqn3z.sine = 1.5,1.5,0,0,0,0

        self.eqn4 = equation()
        self.eqn4x = equation_array()
        self.eqn4x.finalpoint, self.eqn4x.offset, self.eqn4x.sine, self.eqn4x.cosine, self.eqn4x.finalcosine, self.eqn4x.sine = 2.5,2.5,0,1,1,0
        self.eqn4y = equation_array()
        self.eqn4y.finalpoint, self.eqn4y.offset, self.eqn4y.sine, self.eqn4y.cosine, self.eqn4y.finalcosine, self.eqn4y.sine = 1.45,1.45,1,0,0,1
        self.eqn4z = equation_array()
        self.eqn4z.finalpoint, self.eqn4z.offset, self.eqn4z.sine, self.eqn4z.cosine, self.eqn4z.finalcosine, self.eqn4z.sine = 1.5,1.5,0,0,0,0
        
        self.eqn1.x, self.eqn1.y, self.eqn1.z = self.eqn1x, self.eqn1y, self.eqn1z
        self.eqn2.x, self.eqn2.y, self.eqn2.z = self.eqn2x, self.eqn2y, self.eqn2z
        self.eqn3.x, self.eqn3.y, self.eqn3.z = self.eqn3x, self.eqn3y, self.eqn3z
        self.eqn4.x, self.eqn4.y, self.eqn4.z = self.eqn4x, self.eqn4y, self.eqn4z
        self.msg.SquadEquation = [self.eqn1, self.eqn2, self.eqn3, self.eqn4]

        # for theta parameters
        self.msg.theta_start = [0,0,120,240]
        self.msg.theta_end = [0,720,840,960]

        # for InitPose parameter
        # self.initPoseArr = PoseArray()
        # self.pose3 = Pose()
        # self.pose3.position.x = 0
        # self.pose3.position.y = 1.5
        # self.pose3.position.z = 1.5
        # self.dirGoalArr.poses.append(self.pose3)
        # self.msg.InitPose = self.initPoseArr
        # self.msg.InitPose.header.stamp = rospy.Time.now()

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published comamand 2 - equation'

    def publishMsg3(self):
        # for CommandType parameter
        self.msg.CommandType = [1]

        # for Time parameter
        self.msg.Time = [3]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for Squad parameter
        self.squad1 = int_array()
        self.squad1.members = [0,1,2,3]
        self.msg.Squad = [self.squad1]

        # for SquadPos parameter
        self.pose2d_s1 = Pose2D()
        self.squadArr1 = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 2.5
        self.pose1.position.y = 1.45
        self.pose1.position.z = 1.5
        self.squadArr1.poses.append(self.pose1)
        self.pose2 = Pose()
        self.pose2.position.x = 1
        self.pose2.position.y = 0
        self.pose2.position.z = 0
        self.squadArr1.poses.append(self.pose2)
        self.pose3 = Pose()
        self.pose3.position.x = -0.5
        self.pose3.position.y = 0.866
        self.pose3.position.z = 0
        self.squadArr1.poses.append(self.pose3)
        self.pose4 = Pose()
        self.pose4.position.x = -0.5
        self.pose4.position.y = -0.866
        self.pose4.position.z = 0
        self.squadArr1.poses.append(self.pose4)
        self.pose2d_s1.membersRelativePos = self.squadArr1

        self.msg.SquadPos = [self.pose2d_s1]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published comamand 3 - squad change'

    def publishMsg4(self):
        # for CommandType parameter
        self.msg.CommandType = [2]

        # for Time parameter
        self.msg.Time = [8]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for DirectGoal parameter
        self.dirGoalArr = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 5
        self.pose1.position.y = 2
        self.pose1.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose1)

        self.msg.DirectGoal = self.dirGoalArr
        self.msg.DirectGoal.header.stamp = rospy.Time.now()

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [3]

        # for RotationRate parameter
        self.msg.RotationRate = [3]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published command 4 - send direct goal'

    def publishMsg5(self):
        # for CommandType parameter
        self.msg.CommandType = [2]

        # for Time parameter
        self.msg.Time = [2]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for DirectGoal parameter
        self.dirGoalArr = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 6
        self.pose1.position.y = 2
        self.pose1.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose1)

        self.msg.DirectGoal = self.dirGoalArr
        self.msg.DirectGoal.header.stamp = rospy.Time.now()

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [3]

        # for RotationRate parameter
        self.msg.RotationRate = [3]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published command 5 - send direct goal'


    def publishMsg6(self):
        # for CommandType parameter
        self.msg.CommandType = [6]

        # for Time parameter
        self.msg.Time = [20]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [3]

        # for RotationRate parameter
        self.msg.RotationRate = [5]

        # for SquadEquation parameter
        self.eqn1 = equation()
        self.eqn1x = equation_array()
        self.eqn1x.finalpoint, self.eqn1x.offset, self.eqn1x.sine, self.eqn1x.cosine, self.eqn1x.finalcosine, self.eqn1x.sine = 1,5,0,0,0,0
        self.eqn1y = equation_array()
        self.eqn1y.finalpoint, self.eqn1y.offset, self.eqn1y.sine, self.eqn1y.cosine, self.eqn1y.finalcosine, self.eqn1y.sine = 1.25,1.5,0,0.75,0.75,0
        self.eqn1z = equation_array()
        self.eqn1z.finalpoint, self.eqn1z.offset, self.eqn1z.sine, self.eqn1z.cosine, self.eqn1z.finalcosine, self.eqn1z.sine = 1.25,1.25,0.75,0,0,0.75
        
        self.eqn1.x, self.eqn1.y, self.eqn1.z = self.eqn1x, self.eqn1y, self.eqn1z
        self.msg.SquadEquation = [self.eqn1]

        # for theta parameters
        self.msg.theta_start = [0]
        self.msg.theta_end = [1080]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published comamand 6 - equation'

    def publishMsg7(self):
        # for CommandType parameter
        self.msg.CommandType = [7]

        # for Time parameter
        self.msg.Time = [5]

        # for SimulationType parameter
        self.msg.SimulationType = 0

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published command 7 - reset simulation'

    def publishMsg8(self):
        # for CommandType parameter
        self.msg.CommandType = [8]

        # for SimulationType parameter
        self.msg.SimulationType = 0

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published command 8 - run simulation'

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
