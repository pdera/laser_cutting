from PyQt4 import QtCore,QtGui
import sys
import re
import time
import serial
import joystick
class hwl(QtGui.QDialog,joystick.Ui_Form):
    """
    joystickL is inherited from both QtGui.QDialog and hw.Ui_Dialog
    """
    def __init__(self,parent=None):
        """
            Initialization of the class. Call the __init__ for the super classes
        """
        super(hwl,self).__init__(parent)
        self.setupUi(self)
        self.connectActions()
    def main(self):
        self.ser = serial.Serial()
        self.show()
        self.lineEdit_position_X.setEnabled(False)
        self.lineEdit_position_Y.setEnabled(False)
        self.lineEdit_position_Z.setEnabled(False)
        self.lineEdit_step_X.setText('0.01')
        self.lineEdit_step_Y.setText('0.01')
        self.lineEdit_step_Z.setText('0.01')
    def connectActions(self):
        """
        Connect the user interface controls to the logic
        """
        self.pushButton_Connect.clicked.connect(self.connect)
        self.pushButton_X_move_n.clicked.connect(self.move_X_n)
        self.pushButton_X_move_p.clicked.connect(self.move_X_p)
        self.pushButton_Y_move_n.clicked.connect(self.move_Y_n)
        self.pushButton_Y_move_p.clicked.connect(self.move_Y_p)
        self.pushButton_Z_move_n.clicked.connect(self.move_Z_n)
        self.pushButton_Z_move_p.clicked.connect(self.move_Z_p)
    def read_and_update_position(self, ser):
            if self.ser.isOpen():
                self.ser.write("TP?\r".encode('ascii'))
                ln=self.ser.readline()
                positions=self.get_positions(ln)
                self.display_positions(positions)
            else:
                self.showMessage (self, 'Establish connection with the controller first')
    def wait_for_end_of_motion(self, ser, motor):
            if self.ser.isOpen():
                ln=0
                count=0
                while (ln != 1) and (count < 20): 
                    string=str(motor)+'MD?\r'
                    self.ser.write(string.encode('ascii'))
                    ln=int(self.ser.readline())
                    count=count+1
                    time.sleep(0.1)              
                return int(ln)
            else:
                self.showMessage (self, 'Establish connection with the controller first')

    def move_one_motor(self, ser, motor, target):
            if self.ser.isOpen():
                string=str(motor)+'PA '+str(target)+'\r'
                self.ser.write(string.encode('ascii'))
                self.wait_for_end_of_motion(ser, motor)
                self.read_and_update_position(ser)
            else:
                self.showMessage (self, 'Establish connection with the controller first')
    
    def get_positions(self, ln):
            positions=[0.00,0.00,0.00]
            li=[0,1,2]
            ln1=ln
            s=0
            e=0
            for i in li:
                p = re.search(",",ln1)
                e=p.start()-1
                positions[i]=float(ln1[s:e])
                ln1=ln1[p.start()+1:]
            return positions
    def display_positions(self, positions):
            self.lineEdit_position_X.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_position_Y.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_position_Z.setText("{:10.4f}".format(positions[2]))            
    def connect(self, ser):
        if self.pushButton_Connect.text() == 'Connect':
            self.ser.baudrate = 19200
            self.ser.port = 0
            self.ser.bytesize=8
            self.ser.parity='N'
            self.ser.stopbits=1
            self.ser.timeout=0.5
            self.ser.open()
            self.read_and_update_position(ser)
            self.pushButton_Connect.setText('Disconnect')
        else:
            self.pushButton_Connect.setText('Connect')
            self.ser.close()
    #--------------------------------------
    def move_X_n(self):
        Pos  = self.lineEdit_position_X.text()
        Step = self.lineEdit_step_X.text()
        target=float(Pos)-float(Step)
        self.move_one_motor(self.ser, 1, target)
    #--------------------------------------
    def move_X_p(self):
        Pos  = self.lineEdit_position_X.text()
        Step = self.lineEdit_step_X.text()
        target=float(Pos)+float(Step)
        self.move_one_motor(self.ser, 1, target)
    #--------------------------------------
    def move_Y_n(self):
        Pos  = self.lineEdit_position_Y.text()
        Step = self.lineEdit_step_Y.text()
        target=float(Pos)-float(Step)
        self.move_one_motor(self.ser, 2, target)
       #--------------------------------------
    def move_Y_p(self):
        Pos  = self.lineEdit_position_Y.text()
        Step = self.lineEdit_step_Y.text()
        target=float(Pos)+float(Step)
        self.move_one_motor(self.ser, 2, target)
     #--------------------------------------
    def move_Z_n(self):
        Pos  = self.lineEdit_position_Z.text()
        Step = self.lineEdit_step_Z.text()
        target=float(Pos)-float(Step)
        self.move_one_motor(self.ser, 3, target)
    #--------------------------------------
    def move_Z_p(self):
        Pos  = self.lineEdit_position_X.text()
        Step = self.lineEdit_step_X.text()
        target=float(Pos)+float(Step)
        self.move_one_motor(self.ser, 3, target)
    #--------------------------------------
       
if __name__=='__main__':
    app = QtGui.QApplication(sys.argv)
    hwl1 = hwl()
    hwl1.main()
    sys.exit(app.exec_())
