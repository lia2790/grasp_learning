from klampt import *
from klampt.vis.glrobotprogram import *
from loaders.soft_hand_loader import SoftHandLoader
from actuators.CompliantHandEmulator import CompliantHandEmulator
import numpy as np
import sys



#The hardware name
gripper_name = 'soft_hand'

#The Klamp't model name
klampt_model_name = 'data/robots/soft_hand.urdf'

#the number of Klamp't model DOFs
numLinks = 38

# ids of links that should be checked for collision while grasping
links_to_check = [3, 4, 6, 8, 10, 11, 13, 15, 17, 18, 20, 22, 24, 25, 27, 29, 31, 33, 35, 37]

# underactuated joints + mimic joints + regular joints
numJoints = 33

numUnderactuatedJoints = 19

numActuators = 1

numMimicJoints = numJoints - numUnderactuatedJoints

numRegularJoints = 0

#The number of command dimensions
numCommandDims = numActuators

#The names of the command dimensions
commandNames = ['synergy']

#default postures
openCommand = [1]
closeCommand = [0]

#named preset list
presets = {'open':openCommand,
           'closed':closeCommand
           }

#range of postures
commandMinimum = [0]
commandMaximum = [1]

#range of valid command velocities
commandMinimumVelocity = [-1]
commandMaximumVelocity = [1]

class HandEmulator(CompliantHandEmulator):
    """An simulation model for the SoftHand for use with SimpleSimulation"""
    def __init__(self, sim, robotindex=0, link_offset=0, driver_offset=0):
        CompliantHandEmulator.__init__(self, sim, robotindex, link_offset, driver_offset, a_dofs=1, d_dofs=0)

        self.synergy_reduction = 3.0  # convert cable tension into motor torque
        self.effort_scaling = -0.5

        print 'Mimic Joint Info:', self.mimic
        print 'Underactuated Joint Info:', self.hand
        print 'Joint parameters:', self.paramsLoader.handParameters
        print 'Soft Hand loaded.'

        # debug maps: OK
        """
        print self.u_to_l
        print self.l_to_i
        for i in xrange(self.driver_offset, self.robot.numDrivers()):
            print "Driver name:", self.robot.driver(i).getName()
            u_id = self.n_to_u[i]
            print "id u_id l_id:", i, u_id
            if u_id != -1:
                print "Link name (index):", self.robot.link(self.l_to_i[self.u_to_l[u_id]]).getName()
                print "Link name (id):", self.world.getName(self.u_to_l[u_id])
        """

        n_links = self.robot.numLinks()
        for i in range(n_links-link_offset):
            l_i = i + link_offset
            link_name = self.robot.link(l_i).getName()
            if 'fake' not in link_name:
                sh_link = self.sim.body(self.world.robotLink(robotindex, l_i))
                s = sh_link.getSurface()
                if 'soft_hand_palm_link' in link_name:
                    s.kFriction = 1.6
                else:
                    s.kFriction = 1.3
                sh_link.setSurface(s)

    def loadHandParameters(self):
        global klampt_model_name, gripper_name
        self.paramsLoader = SoftHandLoader(klampt_model_name)

        print "Loaded robot name is:", self.robot.getName()
        print "Number of Drivers:", self.robot.numDrivers()
        if self.robot.getName() not in [gripper_name, "temp"]:
            raise Exception('loaded robot is not a soft hand, rather %s'%self.robot.getName())

        # loading previously defined maps
        for i in xrange(self.driver_offset, self.robot.numDrivers()):
            driver = self.robot.driver(i)
            print "Driver ", i, ": ", driver.getName()
            try:
                _,_,finger, phalanx,fake_id = driver.getName().split('_')
            except ValueError:
                prefix, name = driver.getName().split(':')
                _, _, finger, phalanx, fake_id = name.split('_')
            if phalanx == "fake":
                if not self.mimic.has_key(finger):
                    self.mimic[finger] = []
                self.mimic[finger].append(i)
                self.m_to_n.append(i)
                m_id = len(self.m_to_n)-1
                self.n_to_m[i] = m_id
            elif phalanx == "wire":
                self.a_to_n.append(i)
                a_id = len(self.a_to_n) - 1
                self.n_to_a[i] = a_id
            else:
                if not self.hand.has_key(finger):
                    self.hand[finger] = dict()
                self.hand[finger][phalanx] = i
                self.u_to_n.append(i)
                u_id = len(self.u_to_n)-1
                self.n_to_u[i] = u_id

        self.u_dofs = len(self.u_to_n)
        self.m_dofs = len(self.m_to_n)
        # checking load is successful
        assert len(self.a_to_n) == self.a_dofs
        self.a_dofs = len(self.a_to_n)

        # will contain a map from underactuated joint to mimic joints
        # this means, for example, that joint id 1 has to be matched by mimic joint 19
        self.m_to_u = self.m_dofs*[-1]

        for finger in self.hand.keys():
            for phalanx in self.hand[finger].keys():
                joint_count = 0
                if phalanx == 'abd':
                    continue
                else:
                    m_id = self.n_to_m[self.mimic[finger][joint_count]]
                    self.m_to_u[m_id] = self.n_to_u[self.hand[finger][phalanx]]
                    joint_count = joint_count+1

        # loading elasticity and reduction map
        self.R = np.zeros((self.a_dofs, self.u_dofs))
        self.E = np.eye(self.u_dofs)

        for i in xrange(self.driver_offset, self.robot.numDrivers()):
            driver = self.robot.driver(i)
            try:
                _, _, finger, phalanx, fake_id = driver.getName().split('_')
            except ValueError:
                prefix, name = driver.getName().split(':')
                _, _, finger, phalanx, fake_id = name.split('_')
            u_id = self.n_to_u[i]
            if u_id != -1:
                joint_position = self.paramsLoader.phalanxToJoint(finger,phalanx)
                self.R[0, u_id] = self.paramsLoader.handParameters[finger][joint_position]['r']
                self.E[u_id,u_id] = self.paramsLoader.handParameters[finger][joint_position]['e']

        self.R = self.R*(1/np.linalg.norm(self.R[0,:]))

    def getConfiguration(self):
        q = np.array(self.sim.getActualConfig(self.robotindex))
        q = q[self.q_to_t]
        q_u = q[self.u_to_n]
        q_m = q[self.m_to_n]
        q_rollarticular = q_u
        # the indices in q_u to which a mimic is paired
        r_a = [i for i, v in enumerate(self.u_to_n) if v in (np.array(self.m_to_n)+1)]
        q_rollarticular[r_a] += q_m
        q_rollarticular *= 0.5
        q_d = q[self.d_to_n]
        return np.hstack((q_u,q_d))

class HandSimGLViewer(GLSimulationProgram):
    def __init__(self,world,base_link=0,base_driver=0):
        GLSimulationProgram.__init__(self,world,"Reflex simulation program")
        self.handsim = HandEmulator(self.sim,0,base_link,base_driver)
        self.sim.addEmulator(0,self.handsim)
        self.control_dt = 0.01

    def control_loop(self):
        #external control loop
        #print "Time",self.sim.getTime()
        return

    def display(self):
        GLSimulationProgram.display(self)

        #draw forces
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glLineWidth(4.0)
        glBegin(GL_LINES)
        for l_id in self.handsim.virtual_contacts:
            glColor3f(0,1,0)
            forcelen = 0.1
            l = self.handsim.robot.link(self.handsim.l_to_i[l_id])
            b = self.sim.body(l)
            p = [0,0,0]
            f = self.handsim.virtual_wrenches[l_id][0:3]
            glVertex3f(*se3.apply(b.getTransform(), p))
            glVertex3f(*se3.apply(b.getTransform(), vectorops.madd(p,f,forcelen)))
            """
            # draw local link frame
            for color in {(1, 0, 0), (0, 1, 0), (0, 0, 1)}:
                glColor3f(*color)
                glVertex3f(*se3.apply(b.getTransform(), p))
                glVertex3f(*se3.apply(b.getTransform(), vectorops.madd(p, color, 0.1)))
            """
        glEnd()

        glLineWidth(1)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def idle(self):
        if self.simulate:
            for l_id in self.handsim.virtual_contacts:
                glColor3f(0, 1, 0)
                l = self.handsim.robot.link(self.handsim.l_to_i[l_id])
                b = self.sim.body(l)
                f = self.handsim.virtual_wrenches[l_id][0:3]
                p = [0,0,0]
                b.applyForceAtLocalPoint(se3.apply_rotation(b.getTransform(),f),p)
            self.control_loop()
            self.sim.simulate(self.control_dt)

    def print_help(self):
        GLSimulationProgram.print_help()
        print "o/l: increase/decrease synergy command"
        print "q/a: activate/deactivate virtual force at index distal phalanx"

    def keyboardfunc(self, c, x, y):
        # Put your keyboard handler here
        # the current example toggles simulation / movie mode
        index_distal_jid = self.handsim.hand['index']['distal']
        index_distal_uid = self.handsim.n_to_u[index_distal_jid]
        index_distal_id = self.handsim.u_to_l[index_distal_uid]
        if c == 'o':
            u = self.handsim.getCommand()
            u[0] += 0.1
            self.handsim.setCommand(u)
        elif c == 'l':
            u = self.handsim.getCommand()
            u[0] -= 0.1
            self.handsim.setCommand(u)
        elif c == 'q':
            self.handsim.virtual_contacts[index_distal_id] = True
            self.handsim.virtual_wrenches[index_distal_id] = np.array([0,0.0,-5.0,0,0,0])
        elif c == 'a':
            if self.handsim.virtual_contacts.has_key(index_distal_id):
                self.handsim.virtual_contacts.pop(index_distal_id)
            if self.handsim.virtual_wrenches.has_key(index_distal_id):
                self.handsim.virtual_wrenches.pop(index_distal_id)
        else:
            GLSimulationProgram.keyboardfunc(self, c, x, y)

if __name__ == '__main__':
    world = WorldModel()
    if len(sys.argv) == 2:
        if not world.readFile(sys.argv[1]):
            print "Could not load SoftHand hand from", sys.argv[1]
            exit(1)
    else:
        if not world.readFile(klampt_model_name):
            print "Could not load SoftHand hand from", klampt_model_name
            exit(1)
    viewer = HandSimGLViewer(world)
    viewer.run()
