from enum import Enum
import numpy as np
import math

def cosd(ang):
    return math.cos(ang/180.0*math.pi)
def acosd(val):
    return math.acos(val)*180.0/math.pi
def sind(ang):
    return math.sin(ang/180.0*math.pi)
def atan2d(y,x):
    return math.atan2(y,x)*180.0/math.pi

#set print precision
np.set_printoptions(precision=4,floatmode='fixed',suppress=True)

class CFrame:
    # matrix = np.eye(4)

    def __init__(self,mat = np.eye(4)):
        self.matrix = mat

    def __str__(self):
        return f"frame is:\n{self.matrix}"
    def __mul__(self, other):
        # if type(other) in (int, float):
        #     return foo(self.data * other)
        # else:
        #     return foo(self.data * other.data)
        frame = self.copy()
        if type(other) is CFrame :
            frame.matrix = (frame.matrix @ other.matrix)
        return frame

    def copy(self):
        frame = CFrame(mat=np.copy(self.matrix))
        return frame
    
    def loc(self,pt = np.zeros(3)):
        return self.matrix[0:3,3] + ((self.matrix[0:3,0:3]) @ pt)

    def rot(self):
        return self.matrix[0:3,0:3]

    def rotX(self,ang):
        self.matrix = (self.matrix @ CFrame.rX(ang))
        return self

    def rotY(self,ang):
        self.matrix = (self.matrix @ CFrame.rY(ang))
        return self

    def rotZ(self,ang):
        self.matrix = (self.matrix @ CFrame.rZ(ang))
        return self

    def transX(self,off):
        self.matrix = (self.matrix @ CFrame.tX(off))
        return self

    def transY(self,off):
        self.matrix = (self.matrix @ CFrame.tY(off))
        return self

    def transZ(self,off):
        self.matrix = (self.matrix @ CFrame.tZ(off))
        return self

    def inv(self):
        rot = np.array(self.matrix[0:3,0:3],dtype='f')
        rot = rot.transpose()
        loc = np.array(self.matrix[0:3,3],dtype='f')
        loc = -(rot @ loc)

        self.matrix[0:3,0:3] = rot
        self.matrix[0:3,3] = loc

        return self

    @staticmethod
    def rX(ang):
        val = np.array([[1,0,0,0]\
              ,[0,cosd((ang)),-sind((ang)),0]\
              ,[0,sind((ang)), cosd((ang)),0]\
              ,[0,0,0,1]],dtype='f')
        #self.matrix = (self.matrix @ val)
        return val

    @staticmethod
    def rY(ang):
            val = np.array([[ cosd((ang)),0,sind((ang)),0]\
                  ,[0,1,0,0]\
                  ,[-sind((ang)),0,cosd((ang)),0]\
                  ,[0,0,0,1]],dtype='f')
            #self.matrix = (self.matrix @ val)
            return val

    @staticmethod
    def rZ(ang):
        val = np.array([[ cosd((ang)),-sind((ang)),0,0]\
                ,[ sind((ang)), cosd((ang)),0,0]\
                ,[0,0,1,0]\
                ,[0,0,0,1]],dtype='f')
        #self.matrix = (self.matrix @ val)
        return val
        
    @staticmethod
    def tX(offset):
        val = np.eye(4,dtype='f')
        val[0,3] = offset
        return val

    @staticmethod
    def tY(offset):
        val = np.eye(4,dtype='f')
        val[1,3] = offset
        return val
        
    @staticmethod
    def tZ(offset):
        val = np.eye(4,dtype='f')
        val[2,3] = offset
        return val

class CInverseKinematics:
        
    #DH parameters d - distance along Z
    # d
    #DH parameters theta - rotation about Z
    # th
    #DH parameters r - distance along X
    # r
    #DH parameters a - rotation about X
    # a

    #joint variable vector
    # q

    #local coord frame of TCP
    # frameTool

    def __init__(self):
        r2 = math.sqrt((369**2) + (33.5**2))
        th2const = -atan2d(33.5,369)
        
        self.th = np.array([-90, -th2const,180+90+th2const, 0,  0,  0],dtype='f')
        self.d = np.array([130, 0,      0, 265+135-120,  0, 66],dtype='f')
        self.r = np.array([  0,r2,      0,           0,  0,  0],dtype='f')
        self.a = np.array([ 90, 0,     90,         -90, 90,  0],dtype='f')

        self.q = np.array([ 0, 0, 0, 0, 0, 0],dtype='f')
        self.qside = np.array([ 1, 1, -1, 1, 1, 1],dtype='f')

        self.frameTool = CFrame().transZ(86).rotZ(-90).rotX(90)

        self.frameEndEffector =  CFrame()

    def __str__(self):
        return f"position of ee is: {self.frameEndEffector.loc()}, orientation is: {self.frameEndEffector.ori()}\nq is: {self.q}"

    def setEndEffectorConfiguration(self,xee,yee,zee,psiee,thetaee,phiee):
        #print("claculating kinematics")
        self.frameEndEffector = CFrame().transX(xee).transY(yee).transZ(zee).rotZ(psiee).rotX(thetaee).rotZ(phiee)
        self.calculate()

    def calculate(self):
        invFrameTool = self.frameTool.copy()
        invFrameTool.inv()

        F6 = self.frameEndEffector * invFrameTool
        frameD = F6.copy()
        
        frameD.transZ(-self.d[5])

        locD = frameD.loc()

        q1 = atan2d(-locD[0],locD[1])

        F1  = CFrame().transZ(self.d[0]).rotZ(self.qside[0]*q1).rotZ(self.th[0]).transX(self.r[0]).rotX(self.a[0])

        invF1 = F1.copy().inv()
        locDreltoB = invF1.loc(locD)

        distDB = math.sqrt((locDreltoB[0]**2)+(locDreltoB[1]**2))
        oriDB = atan2d(locDreltoB[1],locDreltoB[0])
        if oriDB < 0 :
            oriDB = oriDB + 360

        cosq2 = (self.r[1]**2) + (distDB ** 2) - (self.d[3] ** 2)
        sinq2 = math.sqrt(((2.0*self.r[1]*distDB)**2) - (cosq2 ** 2))

        q2 = oriDB - atan2d(sinq2,cosq2) - self.th[1]

        F2 = F1.copy().transZ(self.d[1]).rotZ(self.qside[1]*q2).rotZ(self.th[1]).transX(self.r[1]).rotX(self.a[1])

        frameC = F2.copy().rotZ(self.th[1]).rotZ(180)

        locDreltoC = frameC.inv().loc(locD)

        q3 = -atan2d(locDreltoC[1],locDreltoC[0])

        F3 = F2.copy().transZ(self.d[2]).rotZ(self.qside[2]*q3).rotZ(self.th[2]).transX(self.r[2]).rotX(self.a[2])
        # F3inv  = F2inv.t_z(d(3)).r_z(-q3).r_z(th(3)).t_x(r(3)).r_x(a(3));

        # % Rotation matrix from frame 3 to 6

        # % [ cos(q4)*cos(q5)*cos(q6) - sin(q4)*sin(q6), - cos(q6)*sin(q4) - cos(q4)*cos(q5)*sin(q6), cos(q4)*sin(q5), 0]
        # % [ cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4),   cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6), sin(q4)*sin(q5), 0]
        # % [                          -cos(q6)*sin(q5),                             sin(q5)*sin(q6),         cos(q5), 0]
        # % [                                         0,                                           0,               0, 1]

        # % R3*R36=R6 ->  R36 = (R3')*R6

        R30 = F3.rot().transpose()
        R06 = F6.rot()
        R36 = (R30 @ R06)

        # print(R36)

        q5 = acosd(R36[2,2])
        q4 = atan2d(R36[1,2],R36[0,2])
        q6 = atan2d(R36[2,1],-R36[2,0])

        #q4 = q4 + 360 if q4 < 0 else q4
        #q6 = q6 + 360 if q6 < 0 else q6

        self.q = np.array([q1,q2,q3,q4,q5,q6],dtype='f')