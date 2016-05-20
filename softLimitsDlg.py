from PyQt4 import QtCore,QtGui, uic


class softLimitsDlg (QtGui.QDialogButtonBox) :


    def __init__(self) :
        QtGui.QDialogButtonBox.__init__(self)
        self.ui = uic.loadUi ("softLimitsDlg.ui")
        self.ui.connect.accepted.connect (self.accept)
        self.xmin = 0.
        self.ymin = 0.
        self.zmin = 0.
        self.xmax = 0.
        self.ymax = 0.
        self.zmax== 0.

    def setVals (self, Xlims, Ylims, Zlims) :
        str = "%f"%Xlims[0]
        self.ui.x_minLE.setText(str)
        str = "%f"%Xlims[1]
        self.ui.x_maxLE.setText(str)
        str = "%f"%Ylims[0]
        self.ui.y_minLE.setText(str)
        str = "%f"%Ylims[1]
        self.ui.y_maxLE.setText(str)
        str = "%f"%Zlims[0]
        self.ui.z_minLE.setText(str)
        str = "%f"%Zlims[1]
        self.ui.z_maxLE.setText(str)




    def accept (self) :
        self.xmin = self.ui.x_minLE.text().toFloat()[0]
        self.xmax = self.ui.x_maxLE.text().toFloat()[0]
        self.ymin = self.ui.y_minLE.text().toFloat()[0]
        self.ymax = self.ui.y_maxLE.text().toFloat()[0]
        self.zmin = self.ui.z_minLE.text().toFloat()[0]
        self.zmax = self.ui.z_maxLE.text().toFloat()[0]

        QtGui.QDialogButtonBox.accept()
