from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
import numpy as np
# The robot object is what you use to control the robot

class Robot_Model:
    def __init__ (self, robot = InterbotixManipulatorXS("px100", "arm", "gripper"), mode = 'h', gripper_status = 'c'):
        self.robot = robot 
        self.mode = mode
        self.gripper_status = gripper_status

    # Let the user select the position
    def select_location (self):    
        while self.mode != 'q':
            self.mode=input("[h]ome, [s]leep, [q]uit ")
            if self.mode == "h":
                self.robot.arm.go_to_home_pose()
                joints = self.robot.arm.get_joint_commands()
                T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, joints)
                [R, p] = mr.TransToRp(T)
                print(p) 
            elif self.mode == "s":
                self.robot.arm.go_to_sleep_pose()
                joints = self.robot.arm.get_joint_commands()
                T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, joints)
                [R, p] = mr.TransToRp(T)
                print(p)

    #open and close the gripper
    def gripper(self):
        while self.gripper_status != 'q':
            self.gripper_status = input("[o]pen, [c]lose, [q]uit ")
            if self.gripper_status == "o":
                self.robot.gripper.release()
            elif self.gripper_status == "c":
                self.robot.gripper.grasp()
    
    #move up and down a littel bit
    def up_down(self, direction):
        while direction != 'q':
            direction = input("[u]p, [d]own, [q]uit ")
            if direction == "u":
                angle = float(input('inter the angle you want to rotate'))
                angle = angle/180*np.pi
                print(angle)
                self.robot.arm.set_single_joint_position('shoulder', angle)
            elif direction == "d":
                angle = float(input('inter the angle you want to rotate'))
                angle = angle/180*np.pi
                print(angle)
                self.robot.arm.set_single_joint_position('shoulder', -angle)

    #print joints information and figure out where to go
    def go(self, x,y,z):
        a = self.robot.arm.set_ee_pose_components(x,y,z)
        return a

    def close(self):
        self.robot.gripper.grasp()
    
    def open(self):
        self.robot.gripper.release()

    def sleep(self):
        self.robot.arm.go_to_sleep_pose()

    #find the Tcr
    def camera_robot(self):
        Tsr = np.array([[-1, 0,  0, 0.0867908],
                        [0, 0, -1, 0.09066],
                        [0, -1,  0, 0.],
                        [0, 0,  0, 1]])
        #Trs = mr.TransInv(Tsr)
        #print(Trs)
        Tcs = np.array([[1, 0,  0, 0.17726876186561584],
                [0, 1, 0, 0.024284927070140839],
                [0, 0,  1, 0.343000009059906],
                [0, 0,  0, 1]])
        Tcr = np.dot(Tcs, Tsr)


        return Tcr

def main():

    robot = Robot_Model()
    robot.select_location()
    #robot.gripper()
    #robot.up_down('a')
    #location = [0.1,0.3,0.4]
    #robot.go(0, 0.2, 0.15)
    a = robot.camera_robot()
    print(a)

if __name__=='__main__':
    main()