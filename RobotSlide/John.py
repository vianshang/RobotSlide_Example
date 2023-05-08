"""!
@file John.py
@brief This file can simulation the robot motor angle
"""
import pybullet as p
import pybullet_data
import time
class SlideBars():
    def __init__(self,Id):
        self.Id=Id
        self.motorNames=[]
        self.motorIndices=[]
        self.motorLowerLimits=[]
        self.motorUpperLimits=[]
        self.slideIds=[]

        self.numJoints=p.getNumJoints(self.Id)
        print(self.numJoints)

    def add_slidebars(self):
        for i in range(self.numJoints):
            jointInfo=p.getJointInfo(self.Id,i)
            print("jointInfo",jointInfo)
            jointName=jointInfo[1].decode('ascii')
            qIndex = jointInfo[3]
            # lowerLimits=jointInfo[8]
            lowerLimits= 1
            upperLimits=jointInfo[9]
            if qIndex > -1:
                self.motorNames.append(jointName)
                self.motorIndices.append(i)
                self.motorLowerLimits.append(lowerLimits)
                self.motorUpperLimits.append(upperLimits)

        for i in range(len(self.motorIndices)): 
            if self.motorLowerLimits[i]<=self.motorUpperLimits[i]:  
                slideId=p.addUserDebugParameter(self.motorNames[i],self.motorLowerLimits[i],self.motorUpperLimits[i],0)
            else: 
                slideId=p.addUserDebugParameter(self.motorNames[i],self.motorUpperLimits[i],self.motorLowerLimits[i],0)
            self.slideIds.append(slideId)

        return self.motorIndices

    def get_slidebars_values(self):
        slidesValues=[]
        for i in self.slideIds:
            value=p.readUserDebugParameter(i)
            slidesValues.append(value)
        return slidesValues
    

'''Connect to pybullet engine'''
# Connect to pybullet GUI
physicsClient = p.connect(p.GUI)
# Set Acceleration of Gravity
p.setGravity(0, 0, -10)
# Connect to pybullet_data path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane
planeId = p.loadURDF("plane.urdf")
# Set the initial position
startPos = [0.1, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
# Observation Camera 
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,            # 距離單位(m)
    cameraYaw=0, cameraPitch=0, # 正視：0,-15 / 側視：90,-15  角度單位(度)
    cameraTargetPosition=(0, 0, 0.55)
        )

'''
# Load slope
boxHalfLength = 1.5
boxHalfWidth = 1.5
boxHalfHeight = 0.001
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
# mass = 1
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [0.05,0,0.01],baseOrientation=[0.0,0.1,0.0,-1.5])
'''
# Load robot
robot_path = "../v1120114/urdf/v1120114.urdf"
# robot_path = "../0413_Robot_URDF/urdf/0413_Robot_URDF.urdf"
John = p.loadURDF( robot_path,
            startPos,
            startOrientation,
            useFixedBase = 0, 
            physicsClientId =physicsClient)
# initPos= [0,0,0,0, # head
#             0,0,0,0, # right_arm
#             0,0,0,0, # left_arm
#             0,
#             # 0,0,0, -0.2,  0.316, 0, # right_leg
#             0,0,0, 0, 0, 0, # right_leg
#             0,0,0,0,0,
#             # 0,0,0,  0.2, -0.316, 0,
#             0,0,0, 0, 0, 0,
#             0,0,0,0,0] # left_leg

number_of_joints = p.getNumJoints(John)
print("number_of_joints",number_of_joints)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(John, joint_number)
    print(info[0], ": ", info[1])

# for j in range (p.getNumJoints(John, physicsClient)):
#     p.changeDynamics(bodyUniqueId = John, linkIndex = j, linearDamping=0, angularDamping=0)
#     p.resetJointState(bodyUniqueId = John, jointIndex = j , targetValue = initPos[j])
#     p.setJointMotorControl2(bodyIndex= John,
#                             jointIndex=j,
#                             controlMode=p.POSITION_CONTROL,
#                             targetPosition=initPos[j],
#                             force=5000,
#                             physicsClientId = physicsClient)
# constraint_idx = p.createConstraint(John, -1, 4, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0, 0.8])
# run = 0.44
# width =1
# height =0.1
# upPosition=0.36


# upstairShape = p.createCollisionShape(p.GEOM_BOX, halfExtents = [run/2,width/2,height], physicsClientId = physicsClient)
# upstairVisBox = p.createVisualShape(p.GEOM_BOX, halfExtents = [run/2,width/2,height], rgbaColor=[0,255,255,1], physicsClientId = physicsClient)
# upstair=p.createMultiBody(baseMass = 0, baseCollisionShapeIndex = upstairShape, basePosition = [upPosition,0,0], baseVisualShapeIndex = upstairVisBox, physicsClientId = physicsClient)
# p.changeDynamics(bodyUniqueId = upstair, linkIndex = -1, lateralFriction = 100, physicsClientId = physicsClient)

        
        

constraint_idx = p.createConstraint( John, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0, 0.47])

slide_bars=SlideBars(John)
motorIndices=slide_bars.add_slidebars()

slide_values=slide_bars.get_slidebars_values()
p.setJointMotorControlArray(John,
                            motorIndices,
                            p.POSITION_CONTROL,
                            targetPositions=slide_values,
                            )

while True:
    p.stepSimulation()
    slide_values=slide_bars.get_slidebars_values()
    p.setJointMotorControlArray(John,
                                motorIndices,
                                p.POSITION_CONTROL,
                                targetPositions=slide_values,
                                )
    pos, vel, force, torque=p.getJointState(John, 5,physicsClientId=physicsClient)
    
    base = [0.0291376719735389,-0.00232841132499396,-0.0260909942531327]
    _30 = [-0.03675 ,0.026, -0.0345]
    _40 = [0.09725 ,0.026, -0.0345]
    diff_R = [_30[i]-base[i] for i in range(len(_30))]
    diff_L = [_40[i]-base[i] for i in range(len(_40))]
    basePos = [0., 0, 0.47]
    print("diff_L",diff_L)
    p.addUserDebugLine(lineFromXYZ = basePos,        # 出發點
                    lineToXYZ = [basePos[i]+diff_R[i] for i in range(len(basePos))] ,          # 結束點
                    lineColorRGB = [1, 0, 0],     # 線段顏色
                    lineWidth = 10,                 # 線段寬度
                    lifeTime = 10)                  # 渲染時間
    print("diff_R",diff_R)
    p.addUserDebugLine(lineFromXYZ = basePos,        # 出發點
                    lineToXYZ = [basePos[i]+diff_L[i] for i in range(len(basePos))],          # 結束點
                    lineColorRGB = [1, 1, 0],     # 線段顏色
                    lineWidth = 10,                 # 線段寬度
                    lifeTime = 10)                  # 渲染時間
    # print("pose and orientation",p.getBasePositionAndOrientation(John, physicsClientId = physicsClient))
    # print(force[2])

