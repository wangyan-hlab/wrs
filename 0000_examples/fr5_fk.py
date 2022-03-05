import math
import numpy as np
import sys
sys.path.append(".")
from visualization.panda import world as wd
from modeling import geometric_model as gm
from robot_sim.robots.fr5 import fr5 as fr5

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
    gm.gen_frame().attach_to(base)
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True, hnd_attached=True)

    goal_conf = np.array([0,-60,-160,0,0,90])*math.pi/180
    robot_s.fk(component_name, goal_conf)
    if not robot_s.is_collided():
        genSphere(robot_s.get_gl_tcp(component_name)[0])
        print(goal_conf)
        robot_s.gen_meshmodel(toggle_tcpcs=False).attach_to(base)
    else:
        print("Collided!")
    base.run()
