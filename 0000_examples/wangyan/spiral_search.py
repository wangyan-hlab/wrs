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
import motion.optimization_based.incremental_nik as inik

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

def spiral(start_angle, start_radius, angle_to_step, radius_to_step, max_angle, max_radius):
    r, phi = start_radius, start_angle
    x, y = [], []
    while phi <= max_angle and r <= max_radius:
        x.append(r * cos(phi))
        y.append(r * sin(phi))
        r += radius_to_step
        phi += angle_to_step

    return x, y

if __name__ == '__main__':

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
    gm.gen_frame().attach_to(base)
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True, arm_jacobian_offset=np.array([0, 0, .145]), hnd_attached=True)

    seed_jnt_values = radians([30, -80, 50, -60, -90, 80])
    # seed_jnt_values = None
    init_pos = array([.50, 0.03, .45])
    init_orn = array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    print("Robot tcp pose = ", init_pos, init_orn)
    jnt_values = robot_s.ik(tgt_pos=init_pos, tgt_rotmat=init_orn, seed_jnt_values=seed_jnt_values)
    robot_s.fk(component_name=component_name, jnt_values=jnt_values)
    # if not robot_s.is_collided():
    #     robot_s.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    spiral_x, spiral_y = spiral(start_angle=0, start_radius=5e-4,
                                angle_to_step=pi/12, radius_to_step=5e-4,
                                max_angle=6*pi, max_radius=0.15)

    print(len(spiral_x), len(spiral_y))
    ## Plot the spiral
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(spiral_x, spiral_y, label='spiral')
    ax.legend()
    plt.show()

    path = []
    count = 0
    pos, orn = init_pos, init_orn
    for x, y in zip(spiral_x, spiral_y):
        count += 1
        print("----------")
        print(">> pt# ", count)
        print(">> x={}, y={}".format(x, y))
        print(">> pos = ", pos)
        pos = array([init_pos[0]+x, init_pos[1]+y, pos[2]+0.001])
        jnt_values = robot_s.ik(tgt_pos=pos, tgt_rotmat=orn, seed_jnt_values=jnt_values)
        robot_s.fk(component_name=component_name, jnt_values=jnt_values)
        if not robot_s.is_collided():
            print("jnt_values = ", degrees(jnt_values))
            path.append(jnt_values)
            # robot_s.gen_meshmodel(toggle_tcpcs=False, rgba=[1, 0, 0, 0.03]).attach_to(base)

    path = path[::-1]
    robot_s.fk(component_name=component_name, jnt_values=path[-1])
    linear_start_pos = robot_s.get_gl_tcp()[0]
    linear_goal_pos = linear_start_pos + np.array([0, 0, -0.03])
    print("linear_start_pos = ", linear_start_pos)
    print("linear_goal_pos = ", linear_goal_pos)
    robot_inik_solver = inik.IncrementalNIK(robot_s)
    linear_path = robot_inik_solver.gen_linear_motion(component_name,
                                                      start_tcp_pos=linear_start_pos,
                                                      start_tcp_rotmat=orn,
                                                      goal_tcp_pos=linear_goal_pos,
                                                      goal_tcp_rotmat=orn,
                                                      obstacle_list=[], granularity=0.005)
    path += linear_path

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
