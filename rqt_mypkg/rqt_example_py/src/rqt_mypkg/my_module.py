import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem

class Param(Plugin):

    def __init__(self, context):
        super(Param, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Param')
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
        ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'Param.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ParamUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        #print rospy.get_param("/")
        allParams = rospy.get_param("/")
        
        # Populate the QTreeWidget
        # Ref: http://pyqt.sourceforge.net/Docs/PyQt4/qtreewidget.html
        #      http://pyqt.sourceforge.net/Docs/PyQt4/qtreewidgetitem.html
        stringsListKey = []
        stringsListParam = []
        stringsListParam2 = []
        for key in allParams:
            #print "key: " + key
            stringsListKey.append(key)
            widgetItemRoot = QTreeWidgetItem(stringsListKey)
            paramParent = allParams.get(key)
            #print "paramParent: " + str(paramParent)
            if (type(paramParent) is dict):
                for param in paramParent:
                    try:
                        paramValue = paramParent.get(param)
                        # print 'param: ' + param
                        # print 'paramValue: ' + str(paramValue)
                        if (type(paramValue) is dict):
                            stringsListParam.append(str(param))
                            widgetItem = QTreeWidgetItem(stringsListParam)
                            for paramKey in paramValue:
                                paramValue2 = paramValue.get(paramKey)
                                # print 'paramKey: ' + paramKey
                                # print "paramValue2: " + str(paramValue2)
                                stringsListParam2.append(str(paramKey))
                                stringsListParam2.append(str(paramValue2))
                                widgetItemChild = QTreeWidgetItem(stringsListParam2)
                                widgetItem.addChild(widgetItemChild)
                                del stringsListParam2[:]
                        else:
                            stringsListParam.append(str(param))
                            stringsListParam.append(str(paramValue))
                            widgetItem = QTreeWidgetItem(stringsListParam)
                    except AttributeError:
                        # print 'paramAtribError: ' + key
                        # print 'paramAtribValue: ' + str(paramParent)
                        widgetItemRoot.setText(1, str(paramParent))
                        break            
                    widgetItemRoot.addChild(widgetItem)
                    del stringsListParam[:]
                self._widget.paramTree.addTopLevelItem(widgetItemRoot)
                del stringsListKey[:]
      

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
