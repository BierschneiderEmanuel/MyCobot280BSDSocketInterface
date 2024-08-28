#!/usr/bin/env python2
# coding:utf-8
import socket
import struct
import serial
import time
import numpy as np
import re

class MycobotServer(object):
    def __init__(self, host, port):
        HOST = host                # The remote host
        PORT = port                # The same port as used by the server
        s = None
        for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
            af, socktype, proto, canonname, sa = res
            try:
                self.s = socket.socket(af, socktype, proto)
            except OSError as msg:
                self.s = None
                continue
            try:
                self.s.connect(sa)
            except OSError as msg:
                self.s.close()
                self.s = None

    def close_connection(self):
        if self.s != None:
            print("Closing connection...")
            self.s.close()
        else:
            print("Connect socker error...")

    def get_coords(self):
        try:
            print("sendall get_coords")
            values = [float(9999.9), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0)]
            data = struct.pack("<7f", *values)
            self.s.sendall(data)
            while True:
                print("await response------------------")
                try:
                    print("await request response get coords--------")
                    data_answ = self.s.recv(28)
                    values_answ = struct.unpack("<7f", data_answ)
                    rounded_val = [round(each_val, 2) for each_val in values_answ]
                    rcv_coords = [values_answ[1], values_answ[2], values_answ[3], values_answ[4], values_answ[5], values_answ[6]]
                    print("rcv coords: ", rcv_coords)
                    return rcv_coords
                except Exception as e:
                    print("str: ie", str(e))
                    self.close_connection()
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def get_angles(self):
        try:
            print("sendall get_angles")
            values = [float(5555.5), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0)]
            data = struct.pack("<7f", *values)
            self.s.sendall(data)
            while True:
                print("await response------------------")
                try:
                    print("wait request response get angles--------")
                    data_answ = self.s.recv(28)
                    values_answ = struct.unpack("<7f", data_answ)
                    rounded_val = [round(each_val, 2) for each_val in values_answ]
                    rcv_angles = [values_answ[1], values_answ[2], values_answ[3], values_answ[4], values_answ[5], values_answ[6]]
                    print("rcv angles: ", rcv_angles)
                    return rcv_angles
                except Exception as e:
                    print("str: ie", str(e))
                    self.close_connection()
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def is_gripper_moving(self):
        try:
            print("sendall is_gripper_moving")
            values = [float(2222.2), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0), float(0.0)]
            data = struct.pack("<7f", *values)
            self.s.sendall(data)
            while True:
                print("await response------------------")
                try:
                    print("await request response is gripper moving--------")
                    data_answ = self.s.recv(28)
                    values_answ = struct.unpack("<7f", data_answ)
                    rounded_val = [round(each_val, 1) for each_val in values_answ]
                    rcv_is_gripper_moving_arr = [rounded_val[1], rounded_val[2], rounded_val[3], rounded_val[4], rounded_val[5], rounded_val[6]]
                    rcv_is_gripper_moving, _= rcv_is_gripper_moving_arr[:2]
                    is_gripper_moving = int(rcv_is_gripper_moving)
                    print("rcv is_gripper_moving: ", is_gripper_moving)
                    return int(is_gripper_moving)
                except Exception as e:
                    print("str: ie", str(e))
                    self.close_connection()
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def send_coords(self, coords_to_send):
        try:
            print("sendall send_coords")
            coords_arr = [float(8888.8), coords_to_send[0], coords_to_send[1], coords_to_send[2], coords_to_send[3], coords_to_send[4], coords_to_send[5]]
            coords_data = struct.pack("<7f", *coords_arr)
            self.s.sendall(coords_data)
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def send_angles(self, angles_to_send):
        try:
            print("sendall send_angles")
            angles_arr = [float(6666.6), angles_to_send[0], angles_to_send[1], angles_to_send[2], angles_to_send[3], angles_to_send[4], angles_to_send[5]]
            angles_data = struct.pack("<7f", *angles_arr)
            self.s.sendall(angles_data)
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def set_gripper_state(self, gripper_state, gripper_speed):
        try:
            print("sendall send_gripper_state")
            gripper_state_to_send = [float(gripper_state), float(gripper_speed), float(0), float(0), float(0), float(0)]
            gripper_set_arr = [float(3333.3), gripper_state_to_send[0], gripper_state_to_send[1], gripper_state_to_send[2], gripper_state_to_send[3], gripper_state_to_send[4], gripper_state_to_send[5]] #gripper_state_to_send[0] actual gripper_state and gripper_state_to_send[1] gripper duration ctrl flag
            gripper_data = struct.pack("<7f", *gripper_set_arr)
            self.s.sendall(gripper_data)
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()

    def send_cartesian_waypoint(self, cartesian_waypoint_to_send):
        try:
            print("sendall send_cartesian_waypoint")
            cartesian_waypoint_arr = [float(11111.1), cartesian_waypoint_to_send[0], cartesian_waypoint_to_send[1], cartesian_waypoint_to_send[2], cartesian_waypoint_to_send[3], cartesian_waypoint_to_send[4], cartesian_waypoint_to_send[5]]
            cartesian_waypoint_data = struct.pack("<7f", *cartesian_waypoint_arr)
            self.s.sendall(cartesian_waypoint_data)
        except Exception as e:
            print("str: e", str(e))
            self.close_connection()
            
if __name__ == "__main__":
    PORT = 9000
    mc = MycobotServer("192.168.178.31",PORT)
    mc.set_gripper_state(float(1), float(50)) #set state and gripper duration
    cart_wayp_arr = [float(0), float(0), float(0), float(0), float(0), float(0)]
    mc.send_cartesian_waypoint(cart_wayp_arr)
    cart_wayp_arr = [float(0), float(0), float(-0.015), float(0), float(0), float(0)]
    mc.send_cartesian_waypoint(cart_wayp_arr)
    cart_wayp_arr = [float(0), float(-0.0125), float(0), float(0), float(0), float(0)]
    mc.send_cartesian_waypoint(cart_wayp_arr)   
    cart_wayp_arr = [float(0), float(0), float(-0.015), float(0), float(0), float(0)]
    mc.send_cartesian_waypoint(cart_wayp_arr)
    mc.close_connection()

