import copy
import math
import time

import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
# from robot_sim.robots.ur5e_ballpeg import ur5e_ballpeg as ur5e
from robot_sim.robots.fr5 import fr5 as fr5
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm
from movement_primitives.dmp import CartesianDMP
import matplotlib.pyplot as plt
import pickle
import snsplot
snsplot.set()
np.set_printoptions(suppress=True)
np.set_printoptions(precision=4)   #设精度

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[-2, -3, 1], lookat_pos=[0, 0, 0.5], w=960, h=720, backgroundcolor=[.8, .8, .8, .5])
    gm.gen_frame().attach_to(base)

    # robot_s
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True, peg_attached=True)
    robot_s.fix_to(pos=[0, 0, 0], rotmat=rm.rotmat_from_euler(0, 0, 0))
    start_conf = np.radians([0, -97.36, 113.41, -106.04, -90.0, 0.89])
    robot_s.fk(component_name, start_conf)

    jnt_traj = np.loadtxt('circle_jnt_values.txt', delimiter=',', usecols=(0, 1, 2, 3, 4, 5))
    path = []
    xyz = np.zeros_like(jnt_traj[:, :3])

    for i, jnt in enumerate(jnt_traj):
        path.append(np.radians(jnt))
        robot_s.fk(component_name, np.radians(jnt))
        xyz[i] = robot_s.get_gl_tcp()[0]
    # np.savetxt('circle_xyz.txt', X=1000*xyz, delimiter=',', fmt='%.04f')
    # plt.plot(xyz[:,0], xyz[:,1])
    # plt.gca().set_aspect('equal', adjustable='box')
    # plt.show()

    rough_path = path[:len(path)//1:50]
    n_steps = len(rough_path)
    dt = 0.01
    execution_time = (n_steps - 1) * dt
    T = np.linspace(0, execution_time, n_steps)
    Y = np.empty((n_steps, 7))

    for i, rp in enumerate(rough_path):
        robot_s.fk(component_name=component_name, jnt_values=rp)
        pos = robot_s.get_gl_tcp(component_name)[0]
        rotmat = robot_s.get_gl_tcp(component_name)[1]
        rot_q = rm.quaternion_from_matrix(rotmat)
        Y[i] = (np.concatenate((pos, rot_q)))

    dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=10)
    dmp.imitate(T, Y)
    new_goal_pos = Y[-1][:3] + np.array(([-.1, -.1, .1]))
    new_goal_rotq = Y[-1][3:]
    new_goal = np.concatenate((new_goal_pos, new_goal_rotq))
    dmp.goal_y = new_goal
    _, Y = dmp.open_loop()

    dmp_path = []
    for pt in Y:
        pos = pt[:3]
        rotmat = rm.rotmat_from_quaternion(pt[3:])[:3, :3]
        # dmp_path.append(
        #     robot_s.ik(component_name=component_name,
        #                tgt_pos=pos,
        #                tgt_rotmat=rotmat,
        #                seed_jnt_values=start_conf)
        # )

        gm.gen_sphere(pos=pos, radius=0.01, rgba=[0, 1, 0, 1]).attach_to(base)

    def update(rbtmnp, motioncounter, robot, path, armname, task):
        if motioncounter[0] < len(path):
            if rbtmnp[0] is not None:
                rbtmnp[0].detach()
            pose = path[motioncounter[0]]
            robot.fk(armname, pose)
            rbtmnp[0] = robot.gen_meshmodel(toggle_tcpcs=True)
            rbtmnp[0].attach_to(base)
            genSphere(robot.get_gl_tcp(component_name)[0], radius=0.01, rgba=[1, 1, 0, 1])
            motioncounter[0] += 1
        else:
            motioncounter[0] = 0
        return task.again


    rbtmnp = [None]
    motioncounter = [0]
    taskMgr.doMethodLater(0.07, update, "update",
                          extraArgs=[rbtmnp, motioncounter, robot_s, rough_path, component_name], appendTask=True)
    base.setFrameRateMeter(True)
    base.run()
