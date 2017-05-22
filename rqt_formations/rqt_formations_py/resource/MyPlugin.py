# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MyPlugin.ui'
#
# Created: Mon Mar 20 16:04:46 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(881, 573)
        self.verticalLayout = QtGui.QVBoxLayout(Form)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.paramTree = QtGui.QTreeWidget(Form)
        self.paramTree.setEnabled(True)
        self.paramTree.setObjectName(_fromUtf8("paramTree"))
        item_0 = QtGui.QTreeWidgetItem(self.paramTree)
        item_1 = QtGui.QTreeWidgetItem(item_0)
        item_0 = QtGui.QTreeWidgetItem(self.paramTree)
        item_1 = QtGui.QTreeWidgetItem(item_0)
        self.verticalLayout.addWidget(self.paramTree)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.paramTree.setSortingEnabled(True)
        self.paramTree.headerItem().setText(0, _translate("Form", "Name", None))
        self.paramTree.headerItem().setText(1, _translate("Form", "Type", None))
        self.paramTree.headerItem().setText(2, _translate("Form", "Value", None))
        __sortingEnabled = self.paramTree.isSortingEnabled()
        self.paramTree.setSortingEnabled(False)
        self.paramTree.topLevelItem(0).setText(0, _translate("Form", "parent", None))
        self.paramTree.topLevelItem(0).child(0).setText(0, _translate("Form", "Child", None))
        self.paramTree.topLevelItem(1).setText(0, _translate("Form", " NewParent", None))
        self.paramTree.topLevelItem(1).child(0).setText(0, _translate("Form", "NewChild", None))
        self.paramTree.setSortingEnabled(__sortingEnabled)


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

