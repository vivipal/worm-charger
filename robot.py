import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
import struct
from math import pi
import time


def get_position(conn):
    conn.write(b'P')
    time.sleep(0.2)
    servo_pos = [int(conn.readline()) for _ in range(4)]
    return servo_pos


def increment2angle(pos): # servo resolution : 4096 and offset of 1024
    angles =  [(i-1024)*2*np.pi/4096 for i in pos]
    return angles

def angle2increment(angle): # servo resolution : 4096
    pos = [i/2/np.pi*4096 for i in angle]
    return pos

def get_head_position(conn,a=145,b=52,c=150,servo_pos = None): # a,b,c are the geometry of the robot

    #check if the servo position has been passed to the function
    if servo_pos is not None :
        servo_angle = increment2angle(get_position(conn))
    else :
        servo_angle = increment2angle(servo_pos)

    # just a little bit of geometry
    r = -a*np.cos(servo_angle[1])+b*np.sin(servo_angle[1])+c*np.cos(servo_angle[1]-servo_angle[2])
    h = a*np.sin(servo_angle[1])+b*np.cos(servo_angle[1])-c*np.sin(servo_angle[1]-servo_angle[2])

    return (r,h)

def go_down(connection):

    servo_position = get_position(connection)
    time.sleep(1)
    N1 = int(servo_position[0])


    head_position = get_head_position(connection,servo_pos = servo_position)

    rbot = rtb.models.DH.widowx()

    pos = [head_position[0],0,head_position[1]+36] # [x,y,z] = [l, 0, 36+h], 0 mais ballec faut juste pas commander le servo 1
    pas = 20
    dt = 0.31
    H = np.flip(np.append(  np.arange(-50,pos[2],pas), pos[2]  ))
    viapt = []
    ang_reel = [servo_position[0]*(pi/2048), (servo_position[1]-1024)*(pi/2048), (servo_position[2]-1024)*(pi/2048)] # [theta_1, theta_2, theta_3], angles réels du robot
    ang = [ang_reel[0], pi - 0.3443 - ang_reel[1], ang_reel[2] + 0.3443 - pi] # [phi_1, phi_2, phi_3], angles utilisés par le modèle

    for h in H:
        T = SE3(pos[0], pos[1], h) * SE3.OA([0, 1, 0], [0, 0, -1])
        sol = rbot.ikinem(T, q0=np.array(([ang[0]], [ang[1]], [ang[2]])).T) # ikinem: cinématique inverse numérique,
        viapt.append(sol[0])
        ang = [sol[0][0], sol[0][1], sol[0][2]]

    VIA = np.array(viapt)
    qt = rtb.trajectory.mstraj(VIA, dt, tacc=0.2, qdmax=10) # tacc : temps d'accélération | qdmax : vitesse max | dt : delay
    # rbot.plot(qt.q, movie='panda1.gif') # plot de la trajectoire
    traj = qt.q
    print("trajectoire :"+str(len(traj)))

    for i in range(len(traj)):

        ang2_mod = traj[i][1]
        ang3_mod = traj[i][2]

        ang2_reel = pi-0.3443-ang2_mod
        ang3_reel = ang3_mod-0.3443+pi

        N2_pile = (2048/pi)*ang2_reel+1024
        N3_pile = (2048/pi)*ang3_reel+1024

        N2 = int((2048/pi)*ang2_reel+1024)
        N3 = int((2048/pi)*ang3_reel+1024)

        N4 = int(1024+N2_pile-N3_pile)

        # # send to the arbotix the next position
        # if 1024<=N2<=3072 and 1024<=N2<=3072:
        #     commande=bytes(';'+str(N1)+';'+str(N2)+';'+str(N3)+';'+str(N4)+';', 'utf-8')
        #     connection.write(b'M')
        #     time.sleep(dt)
        #     connection.write(commande)
        #     print(commande)
        #     time.sleep(dt)
        #     time.sleep(1)
