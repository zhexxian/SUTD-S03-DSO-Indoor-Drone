import os
import rospkg
import rospy
import tf
from geometry_msgs.msg import PoseStamped

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QSlider
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class Formations(Plugin):

    def __init__(self, context):
        super(Formations, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Formations')
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
        ui_file = os.path.join(rp.get_path('rqt_formationspkg'), 'resource', 'Formations.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('FormationsUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # The following line passes the horizontalSlider's value to a function lineMove
        # Idea is the set slider value min: X value of start of table, max slider value: X value of end of table
        # When sliding, the X value of the line changes according to the slider value
        # Ref: http://pyqt.sourceforge.net/Docs/PyQt4/qframe.html#setFrameShadow
        # http://stackoverflow.com/questions/5671354/how-to-programmatically-make-a-horizontal-line-in-qt
        # http://pyqt.sourceforge.net/Docs/PyQt4/qslider.html#details
        # http://stackoverflow.com/questions/26624713/get-the-values-from-the-vertical-slider-in-pyqt4-and-save-that-value-into-one-va
        ### INCOMPLETE ###

        ### For RVIZ ###
        worldFrame = rospy.get_param("~worldFrame", "/world")
        name = "goal"
        coordinate = [0,0,0]
        rate = rospy.Rate(30) # reference from launch file args
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        ##################

        self._widget.horizontalSlider.valueChanged[int].connect(self.lineMove)
        self._widget.generateButton.clicked.connect(self.generateClicked)
        self._widget.playButton.clicked.connect(lambda: self.playClicked(msg, name)) # ref: http://eli.thegreenplace.net/2011/04/25/passing-extra-arguments-to-pyqt-slot

    ### INCOMPLETE ###
    def lineMove(self, valueOfSlider):
        #if valueOfSlider 
        vline = QFrame()
        vline.setFrameShape(QFrame.VLine)
        vline.setFrameShadow(QFrame.Sunken)
        self.add_widget(vline, 1, 0, 1, 2)

    def generateClicked(self):
        formationList = []
        rowList = []
        maxRows = self._widget.tableWidget.rowCount()
        maxCols = self._widget.tableWidget.columnCount()
        # print ("maxRows: " + str(maxRows) + " maxCols: " + str(maxCols))
        # generate a list using the values in qtablewidget
        for row in range(0, maxRows):
            for col in range(0, maxCols):
                try:
                    curItemText = self._widget.tableWidget.item(row, col).text()
                # handle empty cells by setting them to empty strings
                except AttributeError:
                    if self._widget.tableWidget.item(row, col) is None:
                        curItemText = ""
                rowList.append(str(curItemText)) # implicit str to convert unicode str to ascii str
            # copy to newRowList so we do not lose the data in formationList when we delete rowList
            newRowList = list(rowList)
            formationList.append(newRowList)
            del rowList[:]
        print (formationList)

    def Forwardx(self, curr_x, curr_y, curr_z):
        return (curr_x+0.1, curr_y, curr_z)

    def playClicked(self, msg, name):
        # TODO: change to some kind of toggle to show that play button was clicked
        pub = rospy.Publisher(name, PoseStamped, queue_size=1)
        coordinate = self.Forwardx(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        msg.pose.position.x = coordinate[0]
        msg.pose.position.y = coordinate[1]
        msg.pose.position.z = coordinate[2]
        pub.publish(msg)

        # Send position setpoints 30 times per seconds
        self.timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE),
                                 self.send_setpoint)
        # call shutdown() to stop the timer from firing

        # # need to modify this to escape the loop sometimes so that the GUI can be refreshed
        # while not rospy.is_shutdown():
        #     #print ("wtf")
        #     # modify to time segments instead of infinitely moving
        #     # need
        #     msg.header.seq += 1

        #     if msg.header.seq % 2 == 0:
        #         coordinate = (msg.pose.position.x+0.01, msg.pose.position.y, msg.pose.position.z)
        #         msg.pose.position.x = coordinate[0]
        #         msg.pose.position.y = coordinate[1]
        #         msg.pose.position.z = coordinate[2]
        #     msg.header.seq = 0
        #     msg.header.stamp = rospy.Time.now()
        #     pub.publish(msg)
        #     rate.sleep()




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
