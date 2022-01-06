import math
import numpy as np
import sys
sys.path.append(".")
from visualization.panda import world as wd
from modeling import geometric_model as gm
from modeling import collision_model as cm
from robot_sim.robots.fr5 import fr5 as fr5
from motion.probabilistic import rrt_connect as rrtc
from basis import robot_math as rm

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
    gm.gen_frame().attach_to(base)
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True, showhnd=True)
    # goal_conf = np.array([90/180*math.pi, -90/180*math.pi, -90/180*math.pi, -180/180*math.pi, 0/180*math.pi, 0/180*math.pi])
    # goal_conf = np.array([0,0,0,0,0,0])
    goal_pos = np.array([0.232, 0.395, 0.532])
    goal_rotmat = rm.rotmat_from_euler(0, math.pi/2.0, math.pi/180*0)
    goal_conf = robot_s.ik(component_name=component_name, tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
    robot_s.fk(component_name, goal_conf)
    genSphere(robot_s.get_gl_tcp(component_name)[0])
    robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
    robot_meshmodel.attach_to(base)

    goal_pos = np.array([0.232, 0.395, 0.432])
    goal_rotmat = rm.rotmat_from_euler(0, math.pi/2.0, math.pi/180*20)
    goal_conf = robot_s.ik(component_name=component_name, tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
    robot_s.fk(component_name, goal_conf)
    genSphere(robot_s.get_gl_tcp(component_name)[0])
    robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
    robot_meshmodel.attach_to(base)
    
    base.run()
