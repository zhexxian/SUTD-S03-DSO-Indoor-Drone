import os
import rospkg
import rospy
import tf
from std_msgs.msg import Float32

from crazyflie_driver.msg import Command, int_array, Pose2D, equation, equation_array
from geometry_msgs.msg import Pose, PoseArray

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import QtGui, QtCore

class battery(Plugin):
    
    def __init__(self, context):
        super(battery, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Battery')
        rp = rospkg.RosPack()

        UPDATE_RATE = 30

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

        self._container = QWidget()
        self._container.setObjectName('BatteryUI')
        self._layout = QVBoxLayout()
        self._container.setLayout(self._layout)
        context.add_widget(self._container)

        if context.serial_number() > 1:
            self._container.setWindowTitle(self._container.windowTitle() + (' (%d)' % context.serial_number()))

        # find the number of crazyflies connected
        self.crazyflieNum = 0
        self.checked = []
        allParams = rospy.get_param_names()
        for topic in allParams:
            # print topic
            if "crazyflie" in topic and topic[0:11] not in self.checked:
                self.checked.append(topic[0:11])
                self.crazyflieNum += 1
        print self.checked
        print self.crazyflieNum

        # keep track of battery levels
        self.battArr = []

        # set name of topic to publish to
        batteryStr = "/battery"
        crazyflieStr = "/crazyflie"
        for i in range(self.crazyflieNum):
            self.battArr.append(0.0)
            topicName = crazyflieStr+str(i)+batteryStr
            print topicName
            rospy.Subscriber(topicName, Float32, self.callback, i)
            self.battLabel=QLabel("Battery for crazyflie" + str(i))

            setattr(self, "batteryBar"+str(i), QProgressBar()) # equivalent to self.batteryBari = QProgressBar()
            progressBar = getattr(self, "batteryBar%d" % i) # equivalent to progressBar = self.batteryBari

            self._layout.addWidget(self.battLabel)
            self._layout.addWidget(progressBar)

        rospy.Timer(rospy.Duration(1.0/UPDATE_RATE), self.updateDisplay)

    def callback(self, data, flieNum):
        self.battArr[flieNum] = float(data.data)
        # print "battery voltage for crazyflie" + str(flieNum) + ": " + str(data.data)

    def updateDisplay(self, event):
        # print self.battArr
        for k in range(len(self.battArr)):
            currentBatt = self.battArr[k]
            updatedVal = self.normalizeBattery(currentBatt)

            progressBar = getattr(self, "batteryBar%d" % k)
            progressBar.setValue(updatedVal)
            # self._container.progressBar.setValue(updatedVal)

    def normalizeBattery(self, curr):
        fullLevel = 4.0 - 2.164
        currLevel = curr - 2.164
        return (currLevel / fullLevel) * 100

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
