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
        
        self.ui.lineEdit_position_X.setEnabled(True)
        self.ui.lineEdit_position_Y.setEnabled(True)
        self.ui.lineEdit_position_Z.setEnabled(True)

        self.ui.lineEdit_CircleX.setEnabled(True)
        self.ui.lineEdit_CircleY.setEnabled(True)
        self.ui.lineEdit_CircleZ.setEnabled(True)
    
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
        self.pushButton_TraceRect.clicked.connect(self.TraceRect)
        self.pushButton_DefineStartLine.clicked.connect(self.DefineStartLine)
        self.pushButton_DefineEndLine.clicked.connect(self.DefineEndLine)
        self.pushButton_DefineULRect.clicked.connect(self.DefineULRect)
        self.pushButton_DefineLRRect.clicked.connect(self.DefineLRRect)
        self.pushButton_ReadCenter.clicked.connect (self.DefineCenter)

        self.pushButton_Joystick.clicked.connect(self.joystick_on)
        self.pushButton_X_move_n.clicked.connect(self.move_X_n)
        self.pushButton_X_move_p.clicked.connect(self.move_X_p)
        self.pushButton_Y_move_n.clicked.connect(self.move_Y_n)
        self.pushButton_Y_move_p.clicked.connect(self.move_Y_p)
        self.pushButton_Z_move_n.clicked.connect(self.move_Z_n)
        self.pushButton_Z_move_p.clicked.connect(self.move_Z_p)
        #self.circleSpeedSlider.valueChanged.connect(self.csSliderVal)
        self.abortCircleButton.clicked.connect (self.abortCuttingCircle)
        self.cutCircle = True
        self.circleSpeed = .1
        self.lineEdit_CircleSpeed.setText ('0.1')
        self.MoveToTargetButton.clicked.connect (self.move_to_target)

    def abortCuttingCircle (self) :
        self.cutCircle = False

    def csSliderVal (self, value):

        str = QtCore.QString ("%1").arg(value)
        self.lineEdit_CircleSpeed.setText (str)
        self.circleSpeed = value


    def TraceCircle (self):
        print "Trace circle"
        # need to check if the center is already defined
        Y       = float(self.lineEdit_CircleY.text())
        Z       = float(self.lineEdit_CircleZ.text())
        radius  = float(self.lineEdit_CircleRadius.text())
        npasses = int(self.lineEdit_numCirclePasses.text())
        self.circleSpeed = float(self.lineEdit_CircleSpeed.text())
        #nsteps  = float(self.lineEdit_CircleSegments.text())
        #traj = self.generate_circle_trajectory ([Y,Z], radius, nsteps)
        self.cutCircle = True ;


        #alternative is to use the HC and HL commands to create a group
        # and generate the arc....
        #move to a point out on the radius of the circle

        y0 = Y - radius
        z0 = Z



        print Y,Z
        print y0,z0
        if self.ser.isOpen():

            #establish group
            string='1HN2,3\r'
            self.ser.write(string.encode('ascii'))
            #set vectorial velocity, acceleration and deceleration
            string = '1HV%f\r'%(self.circleSpeed)
            print string
            self.ser.write(string.encode('ascii'))
            string = '1HA1\r'
            self.ser.write(string.encode('ascii'))
            string = '1HD1\r'
            self.ser.write(string.encode('ascii'))
            #enable group
            string = '1HO\r'
            self.ser.write(string.encode('ascii'))
            #move to top of circle
            string = '1HL%f,%f\r'%(y0, z0)
            self.ser.write(string.encode('ascii'))
            time.sleep (.1)
            string = '1HW\r'
            self.ser.write(string.encode('ascii'))

            arcstring = '1HC%f,%f,360\r'%(Y,Z)
            totalDegrees = 360.*npasses
            for ipass in range (npasses) :
                
                self.ser.write(arcstring.encode('ascii'))
                #time.sleep (2.)

                
                movingFlag = 0
                count = 0

                while (movingFlag !=1) and (count < 10000)  :
                    self.ser.flush()
                    string = '1HS?\r'
                    self.ser.write(string.encode('ascii'))
                    time.sleep (.2)
                    ln = self.ser.readline()
                    if (len (ln)) >0 :
                        print ln
                        movingFlag=int(ln)
                    QtCore.QCoreApplication.processEvents()
                    time.sleep (.05)
                    count = count +1
                    self.ser.flush()
                    string = '1HP?\r'
                    self.ser.write(string.encode('ascii'))
                    #time.sleep (.2)
                    ln = self.ser.readline()
                    print 'curpos is : ', ln
                    curpos = self.get_positionsYZ (ln)
                    ydiff = curpos[0]- Y
                    zdiff = curpos[1]- Z
                    angle = math.degrees(math.atan2 (zdiff,ydiff)) + 180.
                    #print 'angle : ',angle
                    if (angle < 0) :
                        angle += 360.
                    cumangle = ipass * 360 + angle
                    frac = cumangle / totalDegrees
                    #print 'angle is : ', cumangle, '  frac is : ', frac
                    self.ui.progressBar_Circle.setValue(frac * 100)

                    if (self.cutCircle == False) :
                        string = '1HS\r'
                        self.ser.write(string.encode('ascii'))
                        string = '1HW\r'
                        self.ser.write(string.encode('ascii'))
                        print 'trying to abort'
                        frac = 0
                        break

                time.sleep(.1)

            #delete group
            string = '1HX\r'
            self.ser.write(string.encode('ascii'))
            self.ser.flush ()

            
            self.read_and_update_position(self.ser)

        else:
            self.showMessage ('Establish connection with the controller first')



       
        if self.cutCircle :
            self.ui.showMessage ('Circle tracing complete')
        else :
            self.ui.showMessage ('Circle tracing aborted!')

    def TraceRect (self) :
        # get the vertices of the coordinates
        Y_ul = self.ui.lineEdit_BoxStartY.text().toFloat()[0]
        Z_ul = self.ui.lineEdit_BoxStartZ.text().toFloat()[0]
        Y_lr = self.ui.lineEdit_BoxEndY.text().toFloat()[0]
        Z_lr = self.ui.lineEdit_BoxEndZ.text().toFloat()[0]
        Y_ur = Y_ul
        Z_ur = Z_lr
        Y_ll = Y_lr
        Z_ll = Z_ul
        # get the dist of each segment
        seg0start = [Z_ul, Y_ul]
        seg0end = [Z_ur, Y_ur]
        seg0dist = math.sqrt (math.pow(Z_ur - Z_ul,2)+math.pow(Y_ur - Y_ul,2))
        seg1start = [Z_ur, Y_ur]
        seg1end = [Z_lr, Y_lr]
        seg1dist = math.sqrt (math.pow(Z_lr - Z_ur,2)+math.pow(Y_lr - Y_ur,2))
        seg2start = [Z_lr, Y_lr]
        seg2end = [Z_ll, Y_ll]
        seg2dist = math.sqrt (math.pow(Z_ll - Z_lr,2)+math.pow(Y_ll - Y_lr,2))
        seg3start = seg2end
        seg3end = seg0start
        seg3dist = math.sqrt (math.pow(Z_ul - Z_ll,2)+math.pow(Y_ul - Y_ll,2))
        totdist = seg0dist + seg1dist + seg2dist + seg3dist
        if totdist <= 0 :
            self.ui.showMessage ("0 distance defined")
            return
        frac0 = float(seg0dist) / totdist
        frac1 = float(seg1dist) / totdist
        frac2 = frac0
        frac3 = frac1
        nsteps = self.ui.lineEdit_RectSegments.text().toInt()[0]
        delay = self.ui.lineEdit_RectDelay.text().toFloat()[0]
        npasses = self.ui.comboBox_LinePasses.currentIndex() + 1
        steps0 = int(nsteps * frac0)
        steps1 = int(nsteps * frac1)
        steps2 = int(nsteps * frac2)
        steps3 = int(nsteps * frac3)
        xytraj_0 = self.generate_line_trajectory(seg0start, seg0end,steps0)
        xytraj_1 = self.generate_line_trajectory(seg1start, seg1end,steps1)
        xytraj_2 = self.generate_line_trajectory(seg2start, seg2end,steps2)
        xytraj_3 = self.generate_line_trajectory(seg3start, seg3end,steps3)
        trajs = [xytraj_0, xytraj_1, xytraj_2, xytraj_3 ]
        fracs = [frac0, frac1, frac2, frac3]
        stepseg=[steps0, steps1, steps2, steps3]

        fraction = 1. / float(npasses ) * 100

        for iter in range (npasses) :
            totaldone = fraction * iter
            for iseg in range (4) :
                curtraj = trajs[iseg]

                for i in range (stepseg[iseg]) :
                    #move motor 2
                    self.move_one_motor(self.ser, 3, curtraj[i,0])
                    #move motor 3
                    self.move_one_motor(self.ser, 2, curtraj[i,1])
                    print "motor 2 : %5.3f   motor 3: %5.3f" % (curtraj [i,0], curtraj[i,1])
                    progress = fracs[iseg] * fraction * float(i+1)/stepseg[iseg] + totaldone
                    self.ui.progressBar_Circle.setValue(progress)
                    time.sleep(delay)

                totaldone += fraction * fracs[iseg]
            curtraj = trajs[0]
            #move to origin
            self.move_one_motor(self.ser, 3, curtraj[0,0])
            self.move_one_motor(self.ser, 2, curtraj[0,1])

            print "motor 2 : %5.3f   motor 3: %5.3f" % (curtraj [0,0], curtraj[0,1])

        infostring = "Rect tracing complete"
        self.ui.progressBar_Circle.setValue(100)
        self.showMessage (infostring)

    def TraceLine (self) :
        startY = self.ui.lineEdit_LineStartY.text().toFloat()[0]
        startZ = self.ui.lineEdit_LineStartZ.text().toFloat()[0]
        endY =  self.ui.lineEdit_LineEndY.text().toFloat()[0]
        endZ =  self.ui.lineEdit_LineEndZ.text().toFloat()[0]
        nsteps = self.ui.lineEdit_LineSegments.text().toInt()[0]
        zytraj = self.generate_line_trajectory([startZ,startY],[endZ,endY],nsteps)
        delay = self.ui.lineEdit_LineDelay.text().toFloat()[0]
        npasses = self.ui.comboBox_LinePasses.currentIndex() + 1

        fraction = 1. / float(npasses) * 100
        for iter in range (npasses) :
            totaldone = fraction * iter
            if iter % 2 == 0 :
                for i in range(nsteps):
                    #move motor 2
                    self.move_one_motor(self.ser, 3, zytraj[i,0])
                    #move motor 3
                    self.move_one_motor(self.ser, 2, zytraj[i,1])
                    #print "motor 2 : %5.3f   motor 3: %5.3f" % (zytraj [i,0], zytraj[i,1])
                    progress = fraction * float(i+1)/nsteps + totaldone
                    self.ui.progressBar_Circle.setValue(progress)
                    time.sleep(delay)
            else :
                for i in range(nsteps-1,-1,-1):
                    #move motor 2
                    self.move_one_motor(self.ser, 3, zytraj[i,0])
                    #move motor 3
                    self.move_one_motor(self.ser, 2, zytraj[i,1])
                    #print "motor 2 : %5.3f   motor 3: %5.3f" % (zytraj [i,0], zytraj[i,1])
                    progress = fraction * float(nsteps-i-1)/nsteps + totaldone
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

    def DefineULRect (self):
        print "Read current loc and place at upper left YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_rect_upperLeft(pos)
        else:
            self.showMessage ('Establish connection with the controller first')

    def DefineCenter (self):
        print "Read current loc and place at center YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_circle_center (pos)
        else:
            self.showMessage ('Establish connection with the controller first')

    def DefineLRRect (self):
        print "Read current loc and place at lower right YZ"
        if self.ser.isOpen():
            pos=self.read_position(self.ser)
            self.display_rect_lowerRight(pos)
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

        xytraj = np.zeros ((steps,2),dtype=np.float32)
        if (steps < 1) :
            return xytraj
        ydist = xy1[1] - xy0[1]
        xdist = xy1[0] - xy0[0]
        dist = math.sqrt(xdist * xdist + ydist * ydist)
        dinc = dist / float(steps)
        xinc = xdist / float(steps)
        yinc = ydist / float(steps)

        for i in range (steps) :
            xytraj [i,0] = xy0[0] + xinc * i
            xytraj [i,1] = xy0[1] + yinc * i
        return xytraj

        

    def read_position(self, ser):
        if self.ser.isOpen():
            self.ser.write("TP?\r".encode('ascii'))
            ln=self.ser.readline()
            print 'read_position is : ', ln
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
    def read_and_update_targetposition(self, ser):
            if self.ser.isOpen():
                positions=self.read_position(self.ser)
                self.display_targetpositions(positions)
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
            else : self.showMessage ('Establish connection with the controller first')


    def move_one_motor(self, ser, motor, target):
            if self.ser.isOpen():
                string=str(motor)+'PA '+str(target)+'\r'
                self.ser.write(string.encode('ascii'))
                self.wait_for_end_of_motion(ser, motor)
                self.read_and_update_position(ser)
            else:
                self.showMessage ('Establish connection with the controller first')
    
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

    def get_positionsYZ(self, ln):
            positions=[0.00,0.00]
            li=[0,1]
            ln1=ln
            s=0
            e=0
            p = re.search (",", ln1)
            e = p.start() -1
            positions[0]=float(ln1[0:e])
            positions[1]=float(ln1[p.start()+1:])
            return positions
        
    def display_positions(self, positions):
            self.lineEdit_position_X.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_position_Y.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_position_Z.setText("{:10.4f}".format(positions[2]))

    def display_targetpositions(self, positions):
            self.lineEdit_target_X.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_target_Y.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_target_Z.setText("{:10.4f}".format(positions[2]))

    def display_positionsYZ(self, positions):
            print 'update positions ', positions[0], positions[1]
            self.lineEdit_position_Y.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_position_Z.setText("{:10.4f}".format(positions[1]))

    def display_circle_center(self, positions):
            self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_CircleY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_CircleZ.setText("{:10.4f}".format(positions[2]))

    def display_rect_upperLeft (self, positions):
            #self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_BoxStartY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_BoxStartZ.setText("{:10.4f}".format(positions[2]))

    def display_rect_lowerRight (self, positions):
            #self.lineEdit_CircleX.setText("{:10.4f}".format(positions[0]))
            self.lineEdit_BoxEndY.setText("{:10.4f}".format(positions[1]))
            self.lineEdit_BoxEndZ.setText("{:10.4f}".format(positions[2]))

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
            self.read_and_update_targetposition (self.ser)
            self.pushButton_Connect.setText('Disconnect')
        else:
            self.pushButton_Connect.setText('Connect')
            self.ser.close()

    def move_to_target (self) :
        pos = self.read_position (self.ser)
        xtarget = self.lineEdit_target_X.text().toFloat()[0]
        #magmove = xtarget - pos[0]
        self.move_one_motor (self.ser, 1, xtarget)
        ytarget = self.lineEdit_target_Y.text().toFloat()[0]
        #magmove = ytarget - pos[1]
        self.move_one_motor (self.ser, 2, ytarget)
        ztarget = self.lineEdit_target_Z.text().toFloat()[0]
        magmove = ztarget - pos[2]
        self.move_one_motor (self.ser, 3, ztarget)
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
