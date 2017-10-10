from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
import struct
import array
import time
import os
import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt
import VFControl
import utm
import copy
ignore_list = ["ATTITUDE_QUATERNION", "HIGHRES_IMU", "ATTITUDE", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED",
               "POSITION_TARGET_LOCAL_NED", "HIGHRES_IMU", "ATTITUDE_TARGET", "POSITION_TARGET_GLOBAL_INT", "VFR_HUD",
               "SYS_STATUS", "BATTERY_STATUS", "HEARTBEAT", "GPS_RAW_INT", "ALTITUDE", "WIND_COV", "EXTENDED_SYS_STATE",
               "ESTIMATOR_STATUS", "VIBRATION", "HOME_POSITION","SERVO_OUTPUT_RAW"]

target_system = 1  # 255
target_component = 0  # 0
mission_type = 0


def LoadWPFile(filename):
    fileread = open(filename, 'rb')
    wpCountMsg = pickle.load(fileread)
    wpList = []
    for i in range(0, wpCountMsg.count):
        wpList.append(pickle.load(fileread))

    fileread.close()
    return wpList


def GetWPList(mav, savefile):
    mav.mav.mission_request_list_send(target_system, target_component, mission_type)

    if (savefile is not None):
        fileout = open(savefile, 'wb')

    num_wp = -1
    print("Requesting WP List")
    while True:
        msg = mav.recv_msg()
        if msg is None:
            continue
        skip_msg = False
        for strnames in ignore_list:
            if (msg.name is strnames):
                skip_msg = True
        if (skip_msg is True):
            continue

        if msg.name is "MISSION_COUNT":
            num_wp = msg.count
            print("#WP:" + str(num_wp))
            if (num_wp == 0):
                mav.mav.mission_ack_send(target_system, target_component,
                                         type=0)  # , mission_type=0)#type=msg.seq, mission_type=0)
                break
            else:
                mav.mav.mission_request_send(target_system, target_component, seq=0)  # , mission_type=0)
            if (savefile is not None):
                pickle.dump(msg, fileout)
        elif msg.name is "MISSION_ITEM":
            print(msg)
            if (savefile is not None):
                pickle.dump(msg, fileout)

            if (msg.seq >= 0 and msg.seq + 1 < num_wp):
                print("--> REQ # " + str(msg.seq))
                mav.mav.mission_request_send(target_system, target_component, seq=msg.seq + 1)  # , mission_type=0)
            elif (msg.seq + 1 == num_wp):
                mav.mav.mission_ack_send(target_system, target_component,
                                         type=0)  # , mission_type=0)#type=msg.seq, mission_type=0)
                if (savefile is not None):
                    fileout.close()
                break
        elif msg.name is "MISSION_CURRENT":
            # print(msg)
            i = 1
        else:
            print(msg)


def WriteWPtoMAV(mavlink, WPmsg):
    count = len(WPmsg)
    target_component = 0  # 0

    mav.mav.mission_count_send(target_system, target_component, count)
    print("Send count " + str(count))
    MAV_MISSION_ACCEPTED = 0  # mission accepted OK
    seqNUM = 0
    while True:
        msg = mav.recv_msg()
        if msg is None:
            continue
        skip_msg = False
        for strnames in ignore_list:
            if (msg.name is strnames):
                skip_msg = True
        if (skip_msg is True):
            continue
        if msg.name is "MISSION_ACK":
            print(msg.name + "->GOOD")
            break
        elif msg.name is "MISSION_REQUEST":
            print(msg.name)
            # mav.mav.mission_ack_send(target_system, target_component, type=MAV_MISSION_ACCEPTED)  # , mission_type=0)#type=msg.seq, mission_type=0)
            msg = WPmsg[seqNUM]

            # if (msg.command == 22):
            #    msg.x = globalpos.lat / 1e7
            #    msg.y = globalpos.lon / 1e7
            #    msg.z = globalpos.alt / 1000

            # target_system = msg.target_system
            target_component = msg.target_component
            seq = seqNUM
            frame = msg.frame
            # if(SendFile=='Set2'):
            #    command = 16
            # else:
            command = msg.command
            current = msg.current
            autocontinue = msg.autocontinue
            param1 = msg.param1
            param2 = msg.param2
            param3 = msg.param3
            param4 = msg.param4
            x = msg.x
            y = msg.y
            z = 10  # msg.z
            print("Send SEQ " + str(seqNUM) + " " + str(msg))
            mav.mav.mission_item_send(target_system, target_component, seq, frame, command, current,
                                      autocontinue, param1, param2,
                                      param3, param4, x, y, z)
            seqNUM = seqNUM + 1

        elif msg.name is "STATUSTEXT":
            print(msg.name + " " + str(msg.text))
            break
        elif msg.name is "MISSION_CURRENT":
            # print(msg)
            z = 1
        else:
            print(msg.name)


def GetHomePositionWait(mav):
    return GetMessageData(mav, "HOME_POSITION")


def GetCurrentGlobalPositionWait(mav):
    return GetMessageData(mav, "GLOBAL_POSITION_INT")


def GetCurrentLocalPositionWait(mav):
    # print("Waiting on Local Position")
    while True:
        msg = mav.recv_msg()
        if msg is None:
            continue

        if msg.name is "LOCAL_POSITION_NED":
            return msg


def GetPositionTargetLocalNEDWait(mav):
    return GetMessageData(mav, 'POSITION_TARGET_LOCAL_NED')


def GetAttitudeWait(mav):
    return GetMessageData(mav, 'ATTITUDE')


def GetMessageData(mav, msg_name):
    while True:
        msg = mav.recv_msg()
        if msg is None:
            continue

        if msg.name is msg_name:
            return msg


mav = mavutil.mavlink_connection('udpin::14552')
# mav = mavutil.mavlink_connection("COM7",baud=115200)
print(mav.address)
print('wait')
mav.wait_heartbeat()
print('got beat')

# SET_POSITION_TARGET_LOCAL_NED
# set_position_target_local_ned_send(self, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x,
#                                   y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, force_mavlink1=False):

# mav.mav.set_position_target_local_ned_send()
dt = 0.01
VFTarget = [500, 500]
cvf = VFControl.CircleVectorField('Gradient')
cvf.mCircleRadius = 20
cvf.xc = VFTarget[0]
cvf.yc = VFTarget[1]
# cvf.bUsePathFunc = True
# cvf.velPathFunc = expPathFunc
# MAV_FRAME_LOCAL_NED


# Save the WP (not used)
#GetWPList(mav,'twoWPtemplate.pickl')
#sys.exit(1)
wpBlank = LoadWPFile('twoWPtemplate.pickl')

HP = GetHomePositionWait(mav)
print(HP)
CGP = GetCurrentGlobalPositionWait(mav)
print(CGP)
# WPList = GetWPList(mav)
plt.figure()
plt.ion()
uav = VFControl.VFUAV(dt)
BasePosLatLon = GetCurrentGlobalPositionWait(mav)

while True:
    CHP = GetHomePositionWait(mav)
    CLP = GetCurrentLocalPositionWait(mav)
    CGP = GetCurrentGlobalPositionWait(mav)

    ATT = GetAttitudeWait(mav)

    # print(CLP)
    plt.cla()
    plt.scatter(CLP.x, CLP.y)
    TargetLocal = GetPositionTargetLocalNEDWait(mav)

    # plt.scatter(TargetLocal.x,TargetLocal.y)
    # uav.SetPosition([CLP.x,CLP.y])
    curr_theta = ATT.yaw  # np.arctan2(CLP.vy,CLP.vx)

    new_vector = {'pos': [CLP.x, CLP.y], 'theta': curr_theta, 'velocity_vec': [CLP.vx, CLP.vy]}
    dir_theta = uav.ExportNewTurnAngleFromVF(new_vector, cvf, t=0)
    new_arm = 25.0  # m

    target_x = CLP.x + new_arm * np.cos(dir_theta)
    target_y = CLP.y + new_arm * np.sin(dir_theta)
    print("Now: " + str(curr_theta) + ' [' + str(CLP.x) + ', ' + str(CLP.y) + ']')
    print("New: " + str(dir_theta) + ' [' + str(target_x) + ', ' + str(target_y) + ']')

    vf = cvf.GetVF_XYUV(0, dt, uav, IncludeUAVPos=True)
    Q = plt.quiver(vf['x'], vf['y'], vf['u'], vf['v'])

    plt.scatter(target_x, target_y, marker='x')

    circle1 = plt.Circle((vf['xc'], vf['yc']), cvf.mCircleRadius, color='b', fill=False)
    ax = plt.gca()

    ax.add_artist(circle1)
    ax.add_artist(plt.scatter(vf['xc'], vf['yc']))
    plt.scatter(cvf.xc, cvf.yc)
    plt.axis('equal')
    plt.pause(0.000000000001)

    #continue #use to avoid waypoint changes


    # get vehicle XYZ (on earth)

    vlat = BasePosLatLon.lat / 1e7
    vlon = BasePosLatLon.lon / 1e7

    Gxy = utm.from_latlon(vlat, vlon)

    x = CLP.x
    y = CLP.y
    z = CLP.z

    #test for same coor
    #CPLL = utm.to_latlon(x + Gxy[0], y + Gxy[1], Gxy[2], Gxy[3])
    #actual data needed
    #CPLL = utm.to_latlon(Gxy[0], Gxy[1], Gxy[2], Gxy[3])

    CTLL = utm.to_latlon(target_x + Gxy[0], target_y + Gxy[1], Gxy[2], Gxy[3])
    CirCenter = utm.to_latlon(cvf.xc + Gxy[0], cvf.yc + Gxy[1], Gxy[2], Gxy[3])

    print("Base Pos",vlat, vlon)
    print("My Pos", CGP.lat/1e7, CGP.lon/1e7)
    print("Target Pos",CTLL[0],CTLL[1])
    print("Circ Pos",CirCenter[0],CirCenter[1])

    #print(CPLL[0], CPLL[1])

    wpBlank = LoadWPFile('twoWPtemplate.pickl')
    wpOut = wpBlank[:]
    wpOut = []
    wpOut.append([])
    wpOut[0] = copy.deepcopy(wpBlank[0])
    wpOut.append([])
    wpOut[1] = copy.deepcopy(wpBlank[1])
    wpOut.append([])
    wpOut[2] = copy.deepcopy(wpOut[1])
    #print(str(len(wpOut))+"#######################################"+str(len(wpBlank)))
    # sys.exit(1)
    wpOut[0].x = CGP.lat/1e7#CPLL[0]
    wpOut[0].y = CGP.lon/1e7#CPLL[1]
    wpOut[0].z = CLP.z
    wpOut[0].current = 0

    # wpOut[0].frame = 1 #MAV_FRAME_LOCAL_NED

    wpOut[1].x = CTLL[0]
    wpOut[1].y = CTLL[1]
    wpOut[1].z = CLP.z
    wpOut[1].seq = 1
    wpOut[1].current = 1

    # wpOut[1].frame = 1

    #wpOut.append([])
    #wpOut[2] = copy.deepcopy(wpOut[1])
    wpOut[2].x = CirCenter[0]
    wpOut[2].y = CirCenter[1]
    wpOut[2].z = CLP.z
    wpOut[2].seq = 2
    wpOut[2].current = 0

    WriteWPtoMAV(mav, wpOut)
    print('Sent WPs')
    plt.pause(2.0)

sys.exit(1)
