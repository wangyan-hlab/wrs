import matplotlib.pyplot as plt
from numpy import *
import snsplot
import math
import time
import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.fr5.fr5 as fr5
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm


def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
    gm.gen_frame().attach_to(base)
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True)

    seed_jnt_values = radians([30, -80, 50, -60, -90, 80])
    # seed_jnt_values = None
    pos = array([.50, 0.03, .57])
    orn = array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    print("Robot tcp pose = ", pos, orn)
    jnt_values = robot_s.ik(tgt_pos=pos, tgt_rotmat=orn, seed_jnt_values=seed_jnt_values)
    robot_s.fk(component_name=component_name, jnt_values=jnt_values)
    # if not robot_s.is_collided():
    #     robot_s.gen_meshmodel(toggle_tcpcs=True).attach_to(base)

    theta = linspace(0, 4 * pi, 50)
    r = (0.5 + 0.5 * theta) * .001
    spiral_x = r * cos(theta)
    spiral_y = r * sin(theta)
    print(len(spiral_x), len(spiral_y))
    ## Plot the spiral
    # fig, ax = plt.subplots(figsize=(6, 6))
    # ax.plot(spiral_x, spiral_y, label='1_spiral')
    # lim = max(r) * 1.2
    # ax.set_xlim([-lim, lim])
    # ax.set_ylim([-lim, lim])
    # ax.legend()
    # plt.show()

    path = []
    count = 0
    for x, y in zip(spiral_x, spiral_y):
        count += 1
        print(">> pt# ", count)
        print("x={}, y={}".format(x, y))
        pos = array([pos[0]+x, pos[1]+y, pos[2]+0.001])
        jnt_values = robot_s.ik(tgt_pos=pos, tgt_rotmat=orn, seed_jnt_values=jnt_values)
        robot_s.fk(component_name=component_name, jnt_values=jnt_values)
        if not robot_s.is_collided():
            print("jnt_values = ", degrees(jnt_values))
            path.append(jnt_values)
            # robot_s.gen_meshmodel(toggle_tcpcs=False, rgba=[1, 0, 0, 0.03]).attach_to(base)

    path = path[::-1]
    def update(rbtmnp, motioncounter, robot, path, armname, task):
        if motioncounter[0] < len(path):
            if rbtmnp[0] is not None:
                rbtmnp[0].detach()
            pose = path[motioncounter[0]]
            robot.fk(armname, pose)
            rbtmnp[0] = robot.gen_meshmodel(toggle_tcpcs=True)
            rbtmnp[0].attach_to(base)
            genSphere(robot.get_gl_tcp(component_name)[0], radius=0.002, rgba=[1, 1, 0, 1])
            motioncounter[0] += 1
        else:
            motioncounter[0] = 0
        return task.again

    rbtmnp = [None]
    motioncounter = [0]
    taskMgr.doMethodLater(0.07, update, "update",
                          extraArgs=[rbtmnp, motioncounter, robot_s, path, component_name],
                          appendTask=True)
    base.setFrameRateMeter(True)
    base.run()
