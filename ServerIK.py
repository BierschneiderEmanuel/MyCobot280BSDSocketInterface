#!/usr/bin/env python
import socket
import serial
import struct
import time
import math
import re
from pymycobot.mycobot import MyCobot
import rospy, roslib, sys, copy
import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class MoveItPlanningServerDemo:
    def __init__(self, host, port):
        print ("host: ", host)
        print ("port: ", port)
        self.mc = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host,port))
        print ("Binding succeeded!")
        self.s.listen(1)
        self.mc = MyCobot('/dev/ttyTHS1', 1000000)

        # move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # ROS
        rospy.init_node("moveit_ik_demo")

        # commander planning scene interface
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        # move group self.arm group
        self.arm = moveit_commander.MoveGroupCommander("arm_group")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        #  link 
        self.end_effector_link = self.arm.get_end_effector_link()

        # reference joint1
        self.reference_frame = "joint1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        # allow replanning
        self.arm.allow_replanning(False)

        # set number of planning threads
        self.arm.set_num_planning_attempts(5)

        # set number of replan attempts
        self.arm.replan_attempts=1

        # set tolerance of planning
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        # init a node and a publisher
        #rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)
        quat_marker = quaternion_from_euler(3.1415, 0, 0)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "/joint1"
        self.marker.ns = "cube"
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(0)
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0

        # marker position initial
        self.marker.pose.position.x = 0.3
        self.marker.pose.position.y = 0.3
        self.marker.pose.position.z = 0.3
        self.marker.pose.orientation.x = quat_marker[0]
        self.marker.pose.orientation.y = quat_marker[1]
        self.marker.pose.orientation.z = quat_marker[2]
        self.marker.pose.orientation.w = quat_marker[3]

    # publish marker
    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0
        self.pub.publish(self.marker)

    def write_coords(self,command):
        print("write_coords: ", command[0], command[1], command[2], command[3], command[4], command[5])
        self.mc.send_coords([command[0], command[1], command[2], command[3], command[4], command[5]], 100, 0)

    def write_angles(self,command):
        print("write_angles: ", command[0], command[1], command[2], command[3], command[4], command[5])
        self.mc.send_angles([command[0], command[1], command[2], command[3], command[4], command[5]], 60)

    def write_gripper_state(self,command_state, command_speed):
        print("write_gripper_state: ", command_state)
        print("write_gripper_speed: ", command_speed)
        self.mc.set_gripper_state(command_state, command_speed)

    def write_cartesian_waypoint(self, cartesian_waypoint_x, cartesian_waypoint_y, cartesian_waypoint_z): #x, y, z
        waypoints = []
        if cartesian_waypoint_x != 0.00000 or cartesian_waypoint_y != 0.00000 or cartesian_waypoint_z != 0.00000:
        	wpose = self.arm.get_current_pose().pose
        	if cartesian_waypoint_x != 0.00000:
		    wpose.position.x += cartesian_waypoint_x #- nach vorne / + nach hinten
		    print("z(x):", wpose.position.x)
        	if cartesian_waypoint_y != 0.00000:
		    wpose.position.y += cartesian_waypoint_y #- nach links #+ nach rechts
		    print("x(y):", wpose.position.y)
        	if cartesian_waypoint_z != 0.00000:
		    wpose.position.z += cartesian_waypoint_z #- nach unten #+ nach oben
		    print("y(z):", wpose.position.z)
		waypoints.append(copy.deepcopy(wpose))
		(plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0)
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.arm.get_current_state()
		display_trajectory.trajectory.append(plan)
                self.arm.set_pose_target(wpose, self.end_effector_link)
		# Publish
		self.display_trajectory_publisher.publish(display_trajectory);
		self.arm.execute(plan, wait=True)
		self.arm.go()
		#rospy.sleep(1)

    def read_coords(self):
        print("read coords")
	moving_coords = []
	while moving_coords == []:
	    moving_coords = self.mc.get_coords()
        print("len read coords", len(moving_coords))
	return moving_coords

    def read_angles(self):
        rospy_set = 1
        if rospy_set == 0:
            print("read angles")
            moving_angles = []
            while moving_angles == []:
	        moving_angles = self.mc.get_angles()
            print("moving_angles:", moving_angles)
            print("type Mov:", type(moving_angles[0]))
            return moving_angles
        else:
            print("read angles")
            current_joints = []
            while current_joints == []:
                current_joints = self.arm.get_current_joint_values()
            moving_angles = [(current_joints[0]/math.pi*180), (current_joints[1]/math.pi*180), (current_joints[2]/math.pi*180), (current_joints[3]/math.pi*180), (current_joints[4]/math.pi*180), (current_joints[5]/math.pi*180)]
            print(moving_angles)
            print("type Mov:", type(moving_angles[0]))
            return moving_angles

    def read_gripper_moving(self):
        print("read gripper moving")
        gripper_moving = self.mc.is_gripper_moving()
        print("gripper_moving:", gripper_moving)
        print("type Mov:", type(gripper_moving))
        gripper_moving_arr = [float(gripper_moving), float(0), float(0), float(0), float(0), float(0)]
        return gripper_moving_arr

    def moving(self):
        # set init poise
        self.arm.set_named_target("init_pose")
        self.arm.go()

        # get init poise
        self.arm.set_start_state_to_current_state()
        current_pose = self.arm.get_current_pose().pose
        print("X: ", current_pose.position.x)
        print("Y: ", current_pose.position.y)
        print("Z: ", current_pose.position.z)
        print("orientation_x: ", current_pose.orientation.x)
        print("orientation_y: ", current_pose.orientation.y)
        print("orientation_z: ", current_pose.orientation.z)
        print("orientation_w: ", current_pose.orientation.w)
        #rospy.sleep(1)

        # set base link
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.07115506229944288
        target_pose.pose.position.y = -0.060740688967969476
        target_pose.pose.position.z = 0.3913130673533024
        target_pose.pose.orientation.x = -0.6771727561555679
        target_pose.pose.orientation.y = 0.2036801148380984
        target_pose.pose.orientation.z = -0.20361251241883715
        target_pose.pose.orientation.w = 0.6771214174183979


        # set current state
        self.arm.set_start_state_to_current_state()

        # set pose target
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        # start planning
        traj = self.arm.plan()

        # execute trajectory
        self.arm.execute(traj, wait=True)
        rospy.sleep(1)

        print("SET_WAYPOINT")
        self.arm.set_start_state_to_current_state()
        try:
            while True:
                print ("waiting connect!------------------")
                conn, addr = self.s.accept()
                port_baud = []
                while True:
                    try:
                        data = conn.recv(28)
                        command = ""
                        values = struct.unpack("<7f", data)
                        print(values)
                        command = values
                        print("type Mov:", type(command))
                        print("command[0]: ", command[0])
                        cmd = round(command[0], 1)
                        print("cmd: ", cmd)
                        if cmd == float(9999.9): #read coords
                            print("command read: ", command)
                            mc_read_res = self.read_coords()
                            res = [float(7777.7), mc_read_res[0], mc_read_res[1], mc_read_res[2], mc_read_res[3], mc_read_res[4], mc_read_res[5]]
                            res_data = struct.pack("<7f", *res)
                            conn.sendall(res_data)
                            print("send read response OK")
                        if cmd == float(5555.5): #read angles
                            print("command read: ", command)
                            mc_read_angles_res = self.read_angles()
                            res_angles = [float(4444.4), mc_read_angles_res[0], mc_read_angles_res[1], mc_read_angles_res[2], mc_read_angles_res[3], mc_read_angles_res[4], mc_read_angles_res[5]]
                            res_angles_data = struct.pack("<7f", *res_angles)
                            conn.sendall(res_angles_data)
                            print("send read angles response OK")
                        if cmd == float(2222.2): #read is gripper moving
                            print("command read: ", command)
                            mc_read_gripper_moving_res = self.read_gripper_moving()
                            res_gripper_moving = [float(1111.1),  mc_read_gripper_moving_res[0], mc_read_gripper_moving_res[1], mc_read_gripper_moving_res[2], mc_read_gripper_moving_res[3], mc_read_gripper_moving_res[4], mc_read_gripper_moving_res[5]]
                            res_gripper_moving_data= struct.pack("<7f", *res_gripper_moving)
                            conn.sendall(res_gripper_moving_data)
                            print("send read gripper state response OK")
                        elif cmd == float(8888.8): #write coords
                            write_coords = [values[1], values[2], values[3], values[4], values[5], values[6]]
                            write_coords_round = [round(each_val, 2) for each_val in write_coords]
                            self.write_coords(write_coords_round)
                        elif cmd == float(6666.6): #write angles
                            write_angles = [values[1], values[2], values[3], values[4], values[5], values[6]]
                            write_angles_round = [round(each_val, 2) for each_val in write_angles]
                            self.write_angles(write_angles_round)
                        elif cmd == float(3333.3): #write gripper state #0 open 1 close
                            write_gripper_state = [values[1], values[2], values[3], values[4], values[5], values[6]]
                            write_gripper_state_round = [round(each_val, 2) for each_val in write_gripper_state]
                            write_gripper_state, write_gripper_speed = write_gripper_state_round[:2]
                            print("write_gripper_state: ", write_gripper_state)
                            print("write_gripper_speed: ", write_gripper_speed)
                            self.write_gripper_state(int(write_gripper_state), int(write_gripper_speed))
                        elif cmd == float(11111.1): #cartesian waypoint
                            write_cartesian_wayp = [values[1], values[2], values[3], values[4], values[5], values[6]]
                            write_cartesian_wayp_round = [round(each_val, 5) for each_val in write_cartesian_wayp]
                            write_cartesian_wayp_round_x, write_cartesian_wayp_round_y, write_cartesian_wayp_round_z = write_cartesian_wayp_round[:3]
                            print("write_cartesian_wayp_round_x: ", write_cartesian_wayp_round_x)
                            print("write_cartesian_wayp_round_y: ", write_cartesian_wayp_round_y)
                            print("write_cartesian_wayp_round_z: ", write_cartesian_wayp_round_z)
                            self.write_cartesian_waypoint(write_cartesian_wayp_round_x, write_cartesian_wayp_round_y, write_cartesian_wayp_round_z)
                    except Exception as e:
                        print ("Exception: ", e)
                        break          
        except Exception as e:
            conn.close()

    def connect(self):
        self.scene.remove_world_object("suit")
        self.pub_marker(0.3, 0.3, 0.3)
        # start moving and server
        self.moving()
        # shutdown graceful
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    HOST = socket.gethostbyname(socket.gethostname())
    PORT = 9000
    o = MoveItPlanningServerDemo(HOST,PORT)
    o.connect()
