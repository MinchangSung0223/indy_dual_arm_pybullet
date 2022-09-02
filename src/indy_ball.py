import pybullet as p

class Indy:
    def __init__(self, stepsize=1e-3, realtime=0):
        self.t = 0.0
        self.stepsize = stepsize
        self.realtime = realtime

        self.control_mode = "torque" 

        self.position_control_gain_p = [0.1,0.1,0.1,0.1,0.1,0.1]
        self.position_control_gain_d = [1.0,1.0,1.0,1.0,1.0,1.0]
        self.max_torque = [1000,1000,1000,1000,1000,1000]
        self.joint_init = [0,-0.2618,-1.5708,0,-1.309,0]

        # connect pybullet
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=89, cameraPitch=-60, cameraTargetPosition=[0, 0, 0.5])

        p.resetSimulation()
        p.setTimeStep(self.stepsize)
        p.setRealTimeSimulation(self.realtime)
        p.setGravity(0,0,-9.81)

        # load models
        p.setAdditionalSearchPath("../models")

        self.plane = p.loadURDF("plane/plane.urdf",
                                useFixedBase=True)
        p.changeDynamics(self.plane,-1,restitution=.95)
        self.p = p


        # ball
       # self.ball = p.loadURDF("ball/ball.urdf")
       # p.changeDynamics(self.ball,-1,restitution=.95, linearDamping = 1e-2, angularDamping = 1e-2)

       # self.default_ball_pos = [0.5,0,0.1]
       # self.default_ball_ori = [0,0,0,1]
       # p.resetBasePositionAndOrientation(self.ball, self.default_ball_pos, self.default_ball_ori)

        # robot
        self.robot = p.loadURDF("indy/indy.urdf",
                                useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION)
        self.robot_id = self.robot;
        # robot parameters
        self.dof = p.getNumJoints(self.robot) - 1 # Virtual fixed joint between the flange and last link
       # print(self.dof)
        #if self.dof != 6:
        #    raise Exception('wrong urdf file: number of joints is not 6')

        self.joints = []
        self.q_min = []
        self.q_max = []
        self.target_pos = []
        self.target_torque = []

        for j in range(self.dof):
            joint_info = p.getJointInfo(self.robot, j)
            self.joints.append(j)
            self.q_min.append(joint_info[8])
            self.q_max.append(joint_info[9])
            self.target_pos.append((self.q_min[j] + self.q_max[j])/2.0)
            self.target_torque.append(0.)

        self.reset()

    def reset(self):
        self.t = 0.0        
        self.control_mode = "torque"
        for j in range(self.dof):
            self.target_pos[j] =self.joint_init[j]
            self.target_torque[j] = 0.
            p.resetJointState(self.robot,j,targetValue=self.target_pos[j])

        self.resetController()

    def step(self):
        self.t += self.stepsize
        p.stepSimulation()

    # robot functions
    def resetController(self):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for i in range(self.dof)])

    def setControlMode(self, mode):
        if mode == "position":
            self.control_mode = "position"
        elif mode == "torque":
            if self.control_mode != "torque":
                self.resetController()
            self.control_mode = "torque"
        else:
            raise Exception('wrong control mode')

    def setTargetPositions(self, target_pos):
        self.target_pos = target_pos
       # print(target_pos)
        #print(self.joints)
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.target_pos,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def setTargetTorques(self, target_torque):
        self.target_torque = target_torque
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=self.target_torque)

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot, self.joints)
        joint_pos = [x[0] for x in joint_states]
        joint_vel = [x[1] for x in joint_states]
        return joint_pos, joint_vel 

    def solveInverseDynamics(self, pos, vel, acc):
        return list(p.calculateInverseDynamics(self.robot, pos, vel, acc))

    def solveInverseKinematics(self, pos, ori):
        return list(p.calculateInverseKinematics(self.robot, 6, pos, ori))

    # ball funtions
    def resetBall(self, ball_pos=None, ball_ori=None):
        if ball_pos is None:
            ball_pos = self.default_ball_pos
        if ball_ori is None:
            ball_ori = self.default_ball_ori
        p.resetBasePositionAndOrientation(self.ball, ball_pos, ball_ori)

    def getBallStates(self):
        ball_states = p.getBasePositionAndOrientation(self.ball)
        ball_pos = list(ball_states[0])
        ball_ori = list(ball_states[1])
        return ball_pos, ball_ori

if __name__ == "__main__":
    robot = Indy(realtime=1)
    while True:
        pass
