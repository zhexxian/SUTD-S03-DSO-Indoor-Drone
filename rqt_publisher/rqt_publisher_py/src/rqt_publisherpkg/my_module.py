import os
import rospkg
import rospy
import tf

from crazyflie_driver.msg import Command, int_array, Pose2D
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
        self.msg.CommandType = [1,1]

        # for Time parameter
        self.msg.Time = [3.0,3.0]

        # for SimulationType parameter
        self.msg.SimulationType = 1

        # for Gap parameter
        # self.msg.Gap = []

        # for Circle parameter
        # self.msg.Circle = []

        # for SquadEquation parameter
        # self.msg.SquadEquation = []

        # for theta parameters
        # self.msg.theta_start = []
        # self.msg.theta_end = []

        # for InitPose parameter
        # self.initPoseArr = PoseArray()
        # self.pose3 = Pose()
        # self.pose3.position.x = 0
        # self.pose3.position.y = 1.5
        # self.pose3.position.z = 1.5
        # self.dirGoalArr.poses.append(self.pose3)
        # self.msg.InitPose = self.initPoseArr
        # self.msg.InitPose.header.stamp = rospy.Time.now()

        # for Squad parameter
        self.squad1 = int_array()
        self.squad1.members = [0,3,4]
        self.squad2 = int_array()
        self.squad2.members = [1,2]
        self.msg.Squad = [self.squad1,self.squad2]

        # for SquadPos parameter
        self.pose2d_s1 = Pose2D()
        self.squadArr1 = PoseArray()
        self.pose4 = Pose()
        self.pose4.position.x = 4
        self.pose4.position.y = 1.5
        self.pose4.position.z = 1.5
        self.squadArr1.poses.append(self.pose4)
        self.pose5 = Pose()
        self.pose5.position.x = -0.5
        self.pose5.position.y = 0.5
        self.pose5.position.z = 0
        self.squadArr1.poses.append(self.pose5)
        self.pose6 = Pose()
        self.pose6.position.x = -0.5
        self.pose6.position.y = -0.5
        self.pose6.position.z = 0
        self.squadArr1.poses.append(self.pose6)
        self.pose2d_s1.membersRelativePos = self.squadArr1

        self.pose2d_s2 = Pose2D()
        self.squadArr2 = PoseArray()
        self.pose7 = Pose()
        self.pose7.position.x = 2
        self.pose7.position.y = 1.5
        self.pose7.position.z = 1.5
        self.squadArr2.poses.append(self.pose7)
        self.pose8 = Pose()
        self.pose8.position.x = -1
        self.pose8.position.y = -1
        self.pose8.position.z = 0
        self.squadArr2.poses.append(self.pose8)
        self.pose2d_s2.membersRelativePos = self.squadArr2

        self.msg.SquadPos = [self.pose2d_s1,self.pose2d_s2]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published squad change message!'

    def publishMsg2(self):
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
        self.msg.CommandType = [2,2]

        # for Time parameter
        self.msg.Time = [5.0,5.0]

        # for SimulationType parameter
        self.msg.SimulationType = 2

        # for DirectGoal parameter
        self.dirGoalArr = PoseArray()
        self.pose1 = Pose()
        self.pose1.position.x = 3
        self.pose1.position.y = 1.5
        self.pose1.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose1)
        self.pose2 = Pose()
        self.pose2.position.x = 5
        self.pose2.position.y = 1.5
        self.pose2.position.z = 1.5
        self.dirGoalArr.poses.append(self.pose2)

        self.msg.DirectGoal = self.dirGoalArr
        self.msg.DirectGoal.header.stamp = rospy.Time.now()

        # for DirectGoalCommandProperties parameter
        self.msg.DirectGoalCommandProperties = [3,3]

        # for RotationRate parameter
        self.msg.RotationRate = [0,0]

        # initialize publisher and publish configured message
        pub = rospy.Publisher(self.topicName, Command, queue_size=1)
        pub.publish(self.msg)

        self.msg.CommandNum += 1 # increase commandNum to keep track of commands and to debug duplicate commands if any

        print 'published send direct goal message!'

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
