import math
import numpy as np
# import sys
# sys.path.append(".")
from visualization.panda import world as wd
from modeling import geometric_model as gm
from modeling import collision_model as cm
from robot_sim.robots.fr5 import fr5 as fr5

def genSphere(pos, radius=0.005, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    """
    How about creating an NN as the FK model of the FR5 robot?
    author: wangyan
    date: 2022/02/21, Suzhou
    """

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
    gm.gen_frame().attach_to(base)
    component_name = 'arm'
    robot_s = fr5.FR5_robot(enable_cc=True)
    seed_jnt_values = np.array([120, -90, 60, 80, 10, 0]) * math.pi / 180
    # seed_jnt_values = robot_s.get_jnt_values(component_name=component_name)
    print("seed_jnt_values = ", seed_jnt_values)
    robot_s.fk(component_name=component_name, jnt_values=seed_jnt_values)
    if not robot_s.is_collided():
        robot_s.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    position = robot_s.get_gl_tcp()[0]
    orientation = robot_s.get_gl_tcp()[1]
    print("Robot tcp pose = ", position, orientation)
    genSphere(pos=robot_s.get_gl_tcp(component_name)[0], radius=0.01)

    base.run()
