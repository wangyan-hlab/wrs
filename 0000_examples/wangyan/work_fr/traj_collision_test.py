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

    base = wd.World(cam_pos=[2, -3, 1], lookat_pos=[0, 0, 0.5], w=960, h=720, backgroundcolor=[.8, .8, .8, .5])
    gm.gen_frame().attach_to(base)
    # robot_s
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True, hnd_attached=False)
    robot_s.fix_to(pos=[0,0,0], rotmat=rm.rotmat_from_euler(0,0,0))
    # robot_s.show_cdprimit()

    path = np.loadtxt('fr5_jnt_values_test_new.txt', delimiter="  ") * math.pi/180
    print(path)
    count_collide = 0
    for i, pose in enumerate(path):
        robot_s.fk(component_name, pose)
        if robot_s.is_collided():
            count_collide += 1
            print("Collide! Pose #{} = {}".format(i, np.degrees(pose)))
            robot_s.gen_meshmodel(toggle_tcpcs=True, rgba=[1, 0, 0, .1]).attach_to(base)
            # base.run()
        # else:
        #     robot_s.gen_meshmodel(rgba=[0, 1, 0, .5]).attach_to(base)
    print("Total #collision = {}/{}".format(count_collide, len(path)))
    #
    base.run()

    # jnt_values = np.radians([-54.479282 , 152.9468 ,   149.35056  ,  39.648462, -8.7511052 ,-39.513021])
    # robot_s.fk(component_name=component_name, jnt_values=jnt_values)
    # robot_s.gen_meshmodel(toggle_jntscs=True).attach_to(base)
    # base.run()

    ## todo: comment this when collision detecting
    # with open("fr5_jnt_values_test.txt") as f:
    #     lines = f.readlines()
    # new_lines = []
    # for line in lines:
    #     line = list(line)
    #     line[:2] = ''
    #     line = ''.join(line)
    #     new_lines.append(line)
    #
    # with open("fr5_jnt_values_test_new.txt", "w") as f:
    #     f.writelines(new_lines)

