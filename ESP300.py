from PyQt4 import QtCore,QtGui, uic
import sys
import re
import time
import serial
import math
import joystick
import numpy as np
from array import *

class ESP300(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.ui = uic.loadUi ("uiESP300.ui", self)

        self.ser = serial.Serial()
        
        self.ui.lineEdit_position_X.setEnabled(False)
        self.ui.lineEdit_position_Y.setEnabled(False)
        self.ui.lineEdit_position_Z.setEnabled(False)

        self.ui.lineEdit_CircleX.setEnabled(False)
        self.ui.lineEdit_CircleY.setEnabled(False)
        self.ui.lineEdit_CircleZ.setEnabled(False)
    
        self.ui.lineEdit_step_X.setText('0.01')
        self.ui.lineEdit_step_Y.setText('0.01')
        self.ui.lineEdit_step_Z.setText('0.01')
        self.ui.progressBar_Circle.setValue(0)
        self.pushButton_Joystick.setText('Joystick OFF')
        """
        Connect the user interface controls to the logic
        """
        self.pushButton_Connect.clicked.connect(self.connect)
        self.pushButton_TraceCircle.clicked.connect(self.TraceCircle)
        self.pushButton_TraceLine.clicked.connect(self.TraceLine)
        self.pushButton_ReadCenter.clicked.connect(self.ReadCenter)
        self.pushButton_DefineStartLine.clicked.connect(self.DefineStartLine)
        self.pushButton_DefineEndLine.clicked.connect(self.DefineEndLine)
        self.pushButton_ReadCurrentRect.clicked.connect(self.ReadCurrentRect)

        self.pushButton_Joystick.clicked.connect(self.joystick_on)
        self.pushButton_X_move_n.clicked.connect(self.move_X_n)
        self.pushButton_X_move_p.clicked.connect(self.move_X_p)
        self.pushButton_Y_move_n.clicked.connect(self.move_Y_n)
        self.pushButton_Y_move_p.clicked.connect(self.move_Y_p)
        self.pushButton_Z_move_n.clicked.connect(self.move_Z_n)
        self.pushButton_Z_move_p.clicked.connect(self.move_Z_p)

    def TraceCircle (self):
        print "Trace circle"
        # need to check if the center is already defined
        Y       = float(self.lineEdit_CircleY.text())
        Z       = float(self.lineEdit_CircleZ.text())
        radius  = float(self.lineEdit_CircleRadius.text())
        delay   = float(self.lineEdit_CircleDelay.text())
        nsteps  = float(self.lineEdit_CircleSegments.text())
        traj=self.generate_circle_trajectory ([Y,Z], radius, nsteps)
        for i in range(0, int(nsteps)):
            print 'progress', float(i)/int(nsteps)*100.0,'%'
            self.move_one_motor(self.ser, 2, traj[i,0])
            self.move_one_motor(self.ser, 3, traj[i,1])
            time.sleep(delay)
            self.ui.progressBar_Circle.setValue(100*(i+1)/(nsteps+1))
        self.ui.showMessage (self, 'Circle tracing complete')


    def TraceLine (self) :
        startY = self.ui.lineEdit_LineStartY.text().toFloat()[0]
        startZ = self.ui.lineEdit_LineStartZ.text().toFloat()[0]
        endY =  self.ui.lineEdit_LineEndY.text().toFloat()[0]
        endZ =  self.ui.lineEdit_LineEndZ.text().toFloat()[0]
        nsteps = self.ui.lineEdit_LineSegments.text().toInt()[0]
        zytraj = self.generate_line_trajectory([startZ,startY],[endZ,endY],nsteps)
        delay = self.ui.lineEdit_LineDelay.text().toFloat()[0]

        for i in range(nsteps):
            #move motor 2
            self.move_one_motor(self.ser, 3, zytraj[i,0])
            #move motor 3
            self.move_one_motor(self.ser, 2, zytraj[i,1])
            print "motor 2 : %5.3f   motor 3: %5.3f" % (zytraj [i,0], zytraj[i,1])
            self.ui.progressBar_Circle.setValue(100.*(i+1)/nsteps)
            time.sleep(delay)


        infostring = "Line tracing complete"
        self.showMessage (infostring)



    def ReadCenter (self):
        print "Read center"
        if self.ser.isOpen():  
            pos=self.read_position(self.ser)
            self.display_circle_center(pos)
        else:
            self.showMessage ('Establish connection with the controller first')

    def DefineStartLine (self):
        print "Read current loc and place at start YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_line_start(pos)
        else:
            self.showMessage ('Establish connection with the controller first')

    def DefineEndLine (self):
        print "Read current loc and place at end YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_line_end(pos)
        else:
            self.showMessage ('Establish connection with the controller first')

    def ReadCurrentRect (self):
        print "Read current loc and place at upper left YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_rect_upperLeft(pos)
        else:
            self.showMessage ('Establish connection with the controller first')


    def Joystick (self):
        print "Joystick"

    def generate_circle_trajectory (self, center, radius, npoint):
        traj=np.zeros((npoint+1,2),dtype=np.float16)
        step=2.0*math.pi/npoint
        for i in range(0, int(npoint)):
            x=center[0]+radius*math.sin(i*step)
            y=center[1]+radius*math.cos(i*step)
            traj[i,0]=x
            traj[i,1]=y
        return traj

    def generate_line_trajectory (self, xy0, xy1,  steps) :
        ydist = xy1[1] - xy0[1]
        xdist = xy1[0] - xy0[0]
        dist = math.sqrt(xdist * xdist + ydist * ydist)
        dinc = dist / float(steps-1)
        xinc = xdist / float(steps-1)
        yinc = ydist / float(steps-1)
        xytraj = np.zeros ((steps,2),dtype=np.float32)
        for i in range (steps) :
            xytraj [i,0] = xy0[0] + xinc * i
            xytraj [i,1] = xy0[1] + yinc * i
        return xytraj

        

    def read_position(self, ser):
        if self.ser.isOpen():
            self.ser.write("TP?\r".encode('ascii'))
            ln=self.ser.readline()
            positions=self.get_positions(ln)
        else:
            positions=[0.0,0.0,0.0]
        return positions
    
    def read_and_update_position(self, ser):
            if self.ser.isOpen():
                positions=self.read_position(self.ser)
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

    def joystick_on(self, ser):
            if self.ser.isOpen():
                dt = 'S20'
                delay=0.0
                if self.pushButton_Joystick.text() == 'Joystick OFF':
                    self.pushButton_Joystick.setText('Joystick ON')
                    st=np.zeros(13, dtype=dt)
                    st[0]='BO0\r'
                    st[1]='1BP11,10,12\r'
                    st[2]='2BP9,8,12\r'
                    st[3]='3BP14,13,12\r'
                    st[4]='1BQ1\r'
                    st[5]='2BQ1\r'
                    st[6]='3BQ1\r'
                    st[7]='1TJ3\r'
                    st[8]='2TJ3\r'
                    st[9]='3TJ3\r'
                    st[10]='1MO\r'
                    st[11]='2MO\r'
                    st[12]='3MO\r'
                    for i in range(13):
                        print st[i][:]
                        self.ser.write(st[i].encode('ascii'))
                        time.sleep(delay)
                else:
                    self.pushButton_Joystick.setText('Joystick OFF')
                    st=np.zeros(6, dtype=dt)
                    st[0]='1BQ0\r'
                    st[1]='2BQ0\r'
                    st[2]='3BQ0\r'
                    st[3]='1TJ1\r'
                    st[4]='2TJ1\r'
                    st[5]='3TJ1\r'
                    for i in range(6):
                        print st[i].encode('ascii')
                        self.ser.write(st[i].encode('ascii'))
                        time.sleep(delay)
                    self.read_and_update_position(self.ser)
            else : self.showMessage (self, 'Establish connection with the controller first')


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

    def display_circle_center(self, positions):
            self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_CircleY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_CircleZ.setText("{:10.4f}".format(positions[2]))

    def display_rect_upperLeft (self, positions):
            #self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_BoxStartY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_BoxStartZ.setText("{:10.4f}".format(positions[2]))

    def display_line_start (self, positions):
            #self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_LineStartY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_LineStartZ.setText("{:10.4f}".format(positions[2]))

    def display_line_end (self, positions):
            #self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_LineEndY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_LineEndZ.setText("{:10.4f}".format(positions[2]))
            
    def connect(self, ser):
        if self.pushButton_Connect.text() == 'Connect':
            self.ser.baudrate = 19200
            self.ser.port = 0
            self.ser.bytesize=8
            self.ser.parity='N'
            self.ser.stopbits=1
            self.ser.timeout=0.5
            self.ser.open()
            self.read_and_update_position(self.ser)
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
        Pos  = self.lineEdit_position_Z.text()
        Step = self.lineEdit_step_Z.text()
        target=float(Pos)+float(Step)
        self.move_one_motor(self.ser, 3, target)
    #------------------------------------

    def showMessage (self, infostring) :
        QtGui.QMessageBox.information (self, "LaserCutting : Info", infostring)


if __name__=='__main__':
    app = QtGui.QApplication(sys.argv)
    ESP300 = ESP300()
    ESP300.show()
    sys.exit(app.exec_())
