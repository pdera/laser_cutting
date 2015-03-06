# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'joystick.ui'
#
# Created: Tue Sep 30 02:32:18 2014
#      by: PyQt4 UI code generator 4.11.2
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
        Form.resize(720, 394)
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(30, 70, 81, 16))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(30, 140, 81, 16))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(30, 210, 81, 16))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.pushButton_Close = QtGui.QPushButton(Form)
        self.pushButton_Close.setGeometry(QtCore.QRect(520, 310, 131, 41))
        self.pushButton_Close.setObjectName(_fromUtf8("pushButton_Close"))
        self.layoutWidget = QtGui.QWidget(Form)
        self.layoutWidget.setGeometry(QtCore.QRect(30, 100, 627, 30))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setMargin(0)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.lineEdit_position_X = QtGui.QLineEdit(self.layoutWidget)
        self.lineEdit_position_X.setObjectName(_fromUtf8("lineEdit_position_X"))
        self.horizontalLayout.addWidget(self.lineEdit_position_X)
        self.pushButton_X_move_n = QtGui.QPushButton(self.layoutWidget)
        self.pushButton_X_move_n.setObjectName(_fromUtf8("pushButton_X_move_n"))
        self.horizontalLayout.addWidget(self.pushButton_X_move_n)
        self.lineEdit_step_X = QtGui.QLineEdit(self.layoutWidget)
        self.lineEdit_step_X.setObjectName(_fromUtf8("lineEdit_step_X"))
        self.horizontalLayout.addWidget(self.lineEdit_step_X)
        self.pushButton_X_move_p = QtGui.QPushButton(self.layoutWidget)
        self.pushButton_X_move_p.setObjectName(_fromUtf8("pushButton_X_move_p"))
        self.horizontalLayout.addWidget(self.pushButton_X_move_p)
        self.lineEdit_target_X = QtGui.QLineEdit(self.layoutWidget)
        self.lineEdit_target_X.setObjectName(_fromUtf8("lineEdit_target_X"))
        self.horizontalLayout.addWidget(self.lineEdit_target_X)
        self.layoutWidget1 = QtGui.QWidget(Form)
        self.layoutWidget1.setGeometry(QtCore.QRect(30, 170, 627, 30))
        self.layoutWidget1.setObjectName(_fromUtf8("layoutWidget1"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_2.setMargin(0)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.lineEdit_position_Y = QtGui.QLineEdit(self.layoutWidget1)
        self.lineEdit_position_Y.setObjectName(_fromUtf8("lineEdit_position_Y"))
        self.horizontalLayout_2.addWidget(self.lineEdit_position_Y)
        self.pushButton_Y_move_n = QtGui.QPushButton(self.layoutWidget1)
        self.pushButton_Y_move_n.setObjectName(_fromUtf8("pushButton_Y_move_n"))
        self.horizontalLayout_2.addWidget(self.pushButton_Y_move_n)
        self.lineEdit_step_Y = QtGui.QLineEdit(self.layoutWidget1)
        self.lineEdit_step_Y.setObjectName(_fromUtf8("lineEdit_step_Y"))
        self.horizontalLayout_2.addWidget(self.lineEdit_step_Y)
        self.pushButton_Y_move_p = QtGui.QPushButton(self.layoutWidget1)
        self.pushButton_Y_move_p.setObjectName(_fromUtf8("pushButton_Y_move_p"))
        self.horizontalLayout_2.addWidget(self.pushButton_Y_move_p)
        self.lineEdit_target_Y = QtGui.QLineEdit(self.layoutWidget1)
        self.lineEdit_target_Y.setObjectName(_fromUtf8("lineEdit_target_Y"))
        self.horizontalLayout_2.addWidget(self.lineEdit_target_Y)
        self.layoutWidget2 = QtGui.QWidget(Form)
        self.layoutWidget2.setGeometry(QtCore.QRect(30, 240, 627, 30))
        self.layoutWidget2.setObjectName(_fromUtf8("layoutWidget2"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_3.setMargin(0)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.lineEdit_position_Z = QtGui.QLineEdit(self.layoutWidget2)
        self.lineEdit_position_Z.setObjectName(_fromUtf8("lineEdit_position_Z"))
        self.horizontalLayout_3.addWidget(self.lineEdit_position_Z)
        self.pushButton_Z_move_n = QtGui.QPushButton(self.layoutWidget2)
        self.pushButton_Z_move_n.setObjectName(_fromUtf8("pushButton_Z_move_n"))
        self.horizontalLayout_3.addWidget(self.pushButton_Z_move_n)
        self.lineEdit_step_Z = QtGui.QLineEdit(self.layoutWidget2)
        self.lineEdit_step_Z.setObjectName(_fromUtf8("lineEdit_step_Z"))
        self.horizontalLayout_3.addWidget(self.lineEdit_step_Z)
        self.pushButton_Z_move_p = QtGui.QPushButton(self.layoutWidget2)
        self.pushButton_Z_move_p.setObjectName(_fromUtf8("pushButton_Z_move_p"))
        self.horizontalLayout_3.addWidget(self.pushButton_Z_move_p)
        self.lineEdit_target_Z = QtGui.QLineEdit(self.layoutWidget2)
        self.lineEdit_target_Z.setObjectName(_fromUtf8("lineEdit_target_Z"))
        self.horizontalLayout_3.addWidget(self.lineEdit_target_Z)
        self.pushButton_Connect = QtGui.QPushButton(Form)
        self.pushButton_Connect.setGeometry(QtCore.QRect(520, 20, 131, 41))
        self.pushButton_Connect.setObjectName(_fromUtf8("pushButton_Connect"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.pushButton_X_move_p, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_X.update)
        QtCore.QObject.connect(self.pushButton_Y_move_p, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_Y.update)
        QtCore.QObject.connect(self.pushButton_Z_move_p, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_Z.update)
        QtCore.QObject.connect(self.pushButton_X_move_n, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_X.update)
        QtCore.QObject.connect(self.pushButton_Y_move_n, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_Y.update)
        QtCore.QObject.connect(self.pushButton_Z_move_n, QtCore.SIGNAL(_fromUtf8("clicked()")), self.lineEdit_target_Z.update)
        QtCore.QObject.connect(self.pushButton_Close, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.close)
        QtCore.QMetaObject.connectSlotsByName(Form)
        Form.setTabOrder(self.lineEdit_step_X, self.lineEdit_step_Y)
        Form.setTabOrder(self.lineEdit_step_Y, self.lineEdit_step_Z)
        Form.setTabOrder(self.lineEdit_step_Z, self.pushButton_X_move_p)
        Form.setTabOrder(self.pushButton_X_move_p, self.lineEdit_position_X)
        Form.setTabOrder(self.lineEdit_position_X, self.lineEdit_target_X)
        Form.setTabOrder(self.lineEdit_target_X, self.lineEdit_position_Y)
        Form.setTabOrder(self.lineEdit_position_Y, self.pushButton_Y_move_n)
        Form.setTabOrder(self.pushButton_Y_move_n, self.pushButton_Close)
        Form.setTabOrder(self.pushButton_Close, self.pushButton_Y_move_p)
        Form.setTabOrder(self.pushButton_Y_move_p, self.lineEdit_target_Y)
        Form.setTabOrder(self.lineEdit_target_Y, self.lineEdit_position_Z)
        Form.setTabOrder(self.lineEdit_position_Z, self.pushButton_Z_move_n)
        Form.setTabOrder(self.pushButton_Z_move_n, self.pushButton_X_move_n)
        Form.setTabOrder(self.pushButton_X_move_n, self.pushButton_Z_move_p)
        Form.setTabOrder(self.pushButton_Z_move_p, self.lineEdit_target_Z)
        Form.setTabOrder(self.lineEdit_target_Z, self.pushButton_Connect)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.label.setText(_translate("Form", "X-position", None))
        self.label_2.setText(_translate("Form", "Y-position", None))
        self.label_3.setText(_translate("Form", "Z-position", None))
        self.pushButton_Close.setText(_translate("Form", "Close", None))
        self.pushButton_X_move_n.setText(_translate("Form", "<-", None))
        self.pushButton_X_move_p.setText(_translate("Form", "->", None))
        self.pushButton_Y_move_n.setText(_translate("Form", "<-", None))
        self.pushButton_Y_move_p.setText(_translate("Form", "->", None))
        self.pushButton_Z_move_n.setText(_translate("Form", "<-", None))
        self.pushButton_Z_move_p.setText(_translate("Form", "->", None))
        self.pushButton_Connect.setText(_translate("Form", "Connect", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

