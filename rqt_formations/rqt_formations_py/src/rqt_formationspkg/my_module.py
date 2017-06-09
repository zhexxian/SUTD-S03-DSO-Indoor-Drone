import os
import rospkg
import rospy
import tf
import pyqtgraph as pg
from geometry_msgs.msg import PoseStamped

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QSlider
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import QtGui, QtCore


#  put all the movements in the move.py python script? now i importing Forwardx from move.py
from move import Forwardx

UPDATE_RATE = 30
# made formationList global so can have executeCommands function use it
formationList = []

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
        ui_file = os.path.join(rp.get_path('rqt_formationspkg'), 'resource', 'Formations3.ui')
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
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.name = "goal"
        self.coordinate = [0,0,0]
        # rate = rospy.Rate(30) # reference from launch file args
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.worldFrame
        self.msg.pose.position.x = 0
        self.msg.pose.position.y = 0
        self.msg.pose.position.z = 0
        self.quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.pose.orientation.x = self.quaternion[0]
        self.msg.pose.orientation.y = self.quaternion[1]
        self.msg.pose.orientation.z = self.quaternion[2]
        self.msg.pose.orientation.w = self.quaternion[3]
        ##################

        self._widget.horizontalSlider.valueChanged[int].connect(self.lineMove)
        self._widget.generateButton.clicked.connect(self.generateClicked)
        self._widget.playButton.setCheckable(True)
        # set playButton to a toggle button for play/pause functionality
        self._widget.playButton.toggled.connect(self.playClicked) 
        # ref: http://eli.thegreenplace.net/2011/04/25/passing-extra-arguments-to-pyqt-slot
        # button1.clicked.connect(lambda: self.on_button(1))
        # squadron table should show other members' relative positions to the leader
        #squadronMembersCount = 2 # hardcoded for now but it should be fetching the members count from squadron creator
        self._widget.FormationTree.itemClicked.connect(self.handleItemClicked)
        self._widget.tableWidget.viewport().installEventFilter(self)
        self._widget.squadronTableWidget.itemChanged.connect(self.handleSquadronItemChanged)

        

        # initiate relative positions of squadron members (incoming list of squadron members and their relative pos to squadron leader)


    # https://stackoverflow.com/questions/25925765/pyqt-get-the-current-qtreewidget-item
    # https://stackoverflow.com/questions/8323537/qt-after-drop-event  viewport() was the problem, previously did not use viewport() and drop event was not firing
    # 
    def eventFilter(self, object, event):
        if (object is self._widget.tableWidget.viewport()):
            if (event.type() == QtCore.QEvent.DragEnter):
                getSelected = self._widget.FormationTree.selectedItems()
                #print getSelected[0].text(0)
                if getSelected[0].text(0):
                    event.accept()   # must accept the dragEnterEvent or else the dropEvent can't occur !!!
                else:
                    event.ignore()
            elif (event.type() == QtCore.QEvent.Drop):
                getSelected = self._widget.FormationTree.selectedItems()
                getTimeSpan = float(self._widget.squadronTableWidget.item(0, 3).text()) # 4th cell is the time
                print "time span is: " + str(getTimeSpan)
                # get the row, col https://stackoverflow.com/questions/10524178/qtablewidget-drag-and-drop-a-button-into-a-cell-in-the-qtablewidget
                position = event.pos()
                # print position
                row = self._widget.tableWidget.rowAt(position.y())
                column = self._widget.tableWidget.columnAt(position.x())
                self._widget.tableWidget.setSpan(row, column, 1, getTimeSpan) # https://stackoverflow.com/questions/28017657/how-to-remove-span-from-qtablewidget-in-pyqt4-python-2-7
                                                                              # QTableView.setSpan (self, int row, int column, int rowSpan, int columnSpan)

                # get row that has things being added
                # retrieve the items in the row
                # shift the items by getTimeSpan
                # need to set the span of the shifted rows <-- how to check the span?
                # insert drop by shifting data in cells by getTimeSpan
                print getSelected[0].text(0)

            # insert a wheel event to zoom the grid
            elif (event.type() == QtCore.QEvent.Wheel):
                horizontalSectionSize = self._widget.tableWidget.horizontalHeader().defaultSectionSize()
                verticalSectionSize = self._widget.tableWidget.verticalHeader().defaultSectionSize()
                if event.delta() > 0:
                    horizontalSectionSize += 6
                    verticalSectionSize += 2
                    self._widget.tableWidget.horizontalHeader().setDefaultSectionSize(horizontalSectionSize)
                    self._widget.tableWidget.verticalHeader().setDefaultSectionSize(verticalSectionSize)
                    self._widget.tableWidget.setColumnWidth(0, 100)
                    self._widget.tableWidget.setRowHeight(0, 100)
                    self._widget.tableWidget.setColumnWidth(0, horizontalSectionSize)
                    self._widget.tableWidget.setRowHeight(0, verticalSectionSize)
                    #self._widget.tableWidget.verticalHeader().setDefaultSectionSize(verticalSectionSize)
                    
                if event.delta() < 0 and verticalSectionSize > 3:
                    horizontalSectionSize -= 6
                    verticalSectionSize -= 2
                    self._widget.tableWidget.horizontalHeader().setDefaultSectionSize(horizontalSectionSize)
                    self._widget.tableWidget.verticalHeader().setDefaultSectionSize(verticalSectionSize)
                    self._widget.tableWidget.setColumnWidth(0, 100)
                    self._widget.tableWidget.setRowHeight(0, 100)
                    self._widget.tableWidget.setColumnWidth(0, horizontalSectionSize)
                    self._widget.tableWidget.setRowHeight(0, verticalSectionSize)
                 # tableView->horizontalHeader()->setFixedHeight(100);

                 # setMinimumHeight() and setMaximumHeight()?



            return False # lets the event continue to the edit
        return False

    # Fixed detecting own changes created by function creating infinite event loops https://stackoverflow.com/questions/29420007/detect-item-changed-qttablewidget
    # TODO: Detect how many members are in a squadron (from squadron maker) and edit relative positions according to the file generated by squadron maker
    def handleSquadronItemChanged(self, item):
        # Relative member positions
        # squadron member 2
        self._widget.squadronTableWidget.blockSignals(True)
        row = self._widget.squadronTableWidget.row(item)
        # print "row: " + str(row)
        col = self._widget.squadronTableWidget.column(item)
        # print "col: " + str(col)
        if row == 0: # if x  y, z for the leader has been changed, update the members
            #try:
                leaderPos = float(self._widget.squadronTableWidget.item(0, col).text())
                if col != 3:
                    member2RelativePos = 1.5
                    member2Pos = leaderPos + member2RelativePos
                    item = QTableWidgetItem()
                    item.setText(str(member2Pos))
                    item.setFlags(QtCore.Qt.ItemIsEnabled)  # make item not clickable/editable https://riverbankcomputing.com/pipermail/pyqt/2009-March/022160.html
                    self._widget.squadronTableWidget.setItem(1, col, item)
                elif col == 3:
                    item = QTableWidgetItem()
                    item.setText(str(leaderPos))
                    item.setFlags(QtCore.Qt.ItemIsEnabled)
                    self._widget.squadronTableWidget.setItem(1, col, item)
            #except Exception:
            #    pass
        self._widget.squadronTableWidget.blockSignals(False)

    def handleItemClicked(self, item):
        # TODO:
        # parse the squadron file which includes squadron numbers and their members and relative values
        squadronMembersCount = 2
        # traverse the tree to find the parent regardless of how deep the item that was selected is
        while (item.parent() is not None):
            item = item.parent()
        print (item.text(0))
        if item.text(0) == "Squadron 1":
            self._widget.squadronTableWidget.setRowCount(squadronMembersCount)
        elif item.text(0) == "Squadron 2":
            self._widget.squadronTableWidget.setRowCount(squadronMembersCount)
        elif item.text(0) == "Squadron 3":
            self._widget.squadronTableWidget.setRowCount(squadronMembersCount)
        elif item.text(0) == "Squadron 4":
            self._widget.squadronTableWidget.setRowCount(squadronMembersCount)
        else:
            self._widget.squadronTableWidget.setRowCount(1)
            self._widget.squadronTableWidget.clearContents()

    ### TODO: INCOMPLETE ###
    def lineMove(self, valueOfSlider):
        #if valueOfSlider 
        vline = QFrame()
        vline.setFrameShape(QFrame.VLine)
        vline.setFrameShadow(QFrame.Sunken)
        self.add_widget(vline, 1, 0, 1, 2)

    ### Fetches a list of the formations timeline when the generate button is clicked ###
    # TODO: Append a end tag to signify the end of the sequences of formations (for time synchronization)
    def generateClicked(self):
        del formationList[:] # everytime generateClicked is executed, clear the current formationList to generate a new one (previously it saves the previous entries so the list kept growing)
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

    # TODO: take in commands from formationList and execute them (right now it is hardcoded to only execute one Forwardx command)
    def executeCommands(self, event):
        print ('executing commands!')
        numDrones = len(formationList)
        for droneForm in formationList:
            for formation in droneForm:
                if formation != '': 
                    print (formation)
                    # TODO: process commands for individual drones here

        pub = rospy.Publisher(self.name, PoseStamped, queue_size=1)
        coordinate = Forwardx(self.msg.pose.position.x, self.msg.pose.position.y, self.msg.pose.position.z)
        self.msg.pose.position.x = coordinate[0]
        self.msg.pose.position.y = coordinate[1]
        self.msg.pose.position.z = coordinate[2]
        pub.publish(self.msg)

    # sets the play button to become a toggle button. A timer is used to call the executeCommands function periodically to animate the formations in RVIZ
    def playClicked(self):
        if (self._widget.playButton.isChecked()):
            print ('playButton checked')
            # Send position setpoints 10 times per second
            self.timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE), self.executeCommands) # got rospy.Timer to work inside the rqt plugin, after verifying that rospy.Timer and shutdown() on it works but in a separate python script

        else:
            print ('playButton unchecked')
            # call shutdown() to stop the timer from firings
            self.timer.shutdown()

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
