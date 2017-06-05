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

class Squadron(Plugin):
    
    def __init__(self, context):
        super(Squadron, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Squadron')
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
        ui_file = os.path.join(rp.get_path('rqt_squadronpkg'), 'resource', 'Squadron.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SquadronUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._widget.toggle1.setCheckable(True)
        self._widget.toggle2.setCheckable(True)
        self._widget.toggle3.setCheckable(True)
        self._widget.toggle1.toggled.connect(lambda:self.btnstate(self._widget.toggle1))
        self._widget.toggle2.toggled.connect(lambda:self.btnstate(self._widget.toggle2))
        self._widget.toggle3.toggled.connect(lambda:self.btnstate(self._widget.toggle3))

        self._widget.saveConfig.clicked.connect(self.saveSquadConfig)
        self._widget.delSquad.clicked.connect(self.delSelectedSquad)
        self._widget.createFile.clicked.connect(self.generateFile)

    def btnstate(self, b):
        if b == self._widget.toggle1:
            if b.isChecked() == True:
                b.setText("Toggle +ve")
                self._widget.xcoord.setInputMask('-0.0;')
            else:
                b.setText("Toggle -ve")
                self._widget.xcoord.setInputMask('0.0;')
        if b == self._widget.toggle2:
            if b.isChecked() == True:
                b.setText("Toggle +ve")
                self._widget.ycoord.setInputMask('-0.0;')
            else:
                b.setText("Toggle -ve")
                self._widget.ycoord.setInputMask('0.0;')
        if b == self._widget.toggle3:
            if b.isChecked() == True:
                b.setText("Toggle +ve")
                self._widget.zcoord.setInputMask('-0.0;')
            else:
                b.setText("Toggle -ve")
                self._widget.zcoord.setInputMask('0.0;')       

    def saveSquadConfig(self):
        self.leaderName = str(self._widget.selectLeader.currentText())
        # self.leaderNum = str(self._widget.selectLeader.currentText())[-1]
        print ('leader name: ' + self.leaderName)
        self.leaderX = float(self._widget.xcoord.text())
        self.leaderY = float(self._widget.ycoord.text())
        self.leaderZ = float(self._widget.zcoord.text())
        print ('leader coordinates: ' + str(self.leaderX), str(self.leaderY), str(self.leaderZ))

        rootItem = self.leaderName+';('+str(self.leaderX)+','+str(self.leaderY)+','+str(self.leaderZ)+')'
        squadItemRoot = QTreeWidgetItem([rootItem])

        model = self._widget.relativeTable.model()
        data = []
        for row in range(model.rowCount()):
            data.append([])
            for column in range(model.columnCount()):
                index = model.index(row, column)
                data[row].append(str(model.data(index)))
        # print(data)
        indexVal = [-1.0,-0.75,-0.50,-0.25,0,0.25,0.50,0.75,1.0]
        for rowData in data:
            for colData in rowData:
                if colData == 'None' or colData == 'Leader':
                    pass
                else:
                    self.droneName = colData
                    print ('drone to be added to squadron:' + colData)
                    colIndex = rowData.index(colData)
                    # print ('column index:' + colIndex)
                    rowIndex = data.index(rowData)
                    # print ('row index:' + rowIndex)
                    self.droneX = indexVal[colIndex]
                    self.droneY = indexVal[rowIndex]
                    print ('drone coordinates: ' + str(self.droneX), str(self.droneY))

                    droneItem = self.droneName+';('+str(self.droneX)+','+str(self.droneY)+',0)'
                    leaderChild = QTreeWidgetItem([droneItem])
                    squadItemRoot.addChild(leaderChild)
        self._widget.squadTree.addTopLevelItem(squadItemRoot)

    def delSelectedSquad(self):
        root = self._widget.squadTree.invisibleRootItem()
        for item in self._widget.squadTree.selectedItems():
            (item.parent() or root).removeChild(item)

    # TODO: decide on a file format and write the squadron contents to it
    def generateFile(self):
        fileList = []
        squadList = []

        root = self._widget.squadTree.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            entry = root.child(i)
            leader = entry.text(0).encode("ascii").split(';')
            ldrCoords = leader[1].split(',')
            ldrNum = leader[0][-1]
            ldrX = ldrCoords[0][1:]
            ldrY = ldrCoords[1]
            ldrZ = ldrCoords[2][:-1]
            print ('leader: ' + str(leader))
            squadList.append([ldrNum, ldrX, ldrY, ldrZ])
            child_count2 = entry.childCount()
            for j in range(child_count2):
                otherDrone = root.child(i).child(j).text(0).encode("ascii").split(';')
                droneCoords = otherDrone[1].split(',')
                droneNum = otherDrone[0][-1]
                droneX = droneCoords[0][1:]
                droneY = ldrCoords[1]
                droneZ = ldrCoords[2][:-1]
                print ('follower: ' + str(otherDrone))
                squadList.append([droneNum, droneX, droneY, droneZ])
            fileList.append(squadList)
            squadList = []
        print fileList

        print os.getcwd()
        os.chdir(os.path.expanduser('~'))
        os.chdir('crazyflie_ws/src/rqt_squadron/rqt_squadron_py/src/rqt_squadronpkg') # change accordingly to the name of your catkin workspace
        print os.getcwd()

        squadronFile = open('squadronPlan.txt','w') 
        squadronFile.write(str(fileList))
        print('written squadron file')
        squadronFile.close() 

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
