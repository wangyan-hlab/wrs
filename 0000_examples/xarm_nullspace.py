import os
import math
import numpy as np
import basis.robot_math as rm
import robot_sim.robots.xarm_shuidi.xarm_shuidi as xa
import visualization.panda.world as wd
import modeling.geometric_model as gm

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[1.5, 0, 3], lookat_pos=[0, 0, .5])
    gm.gen_frame().attach_to(base)

    # robot_s
    component_name = 'arm'
    robot_s = xa.XArmShuidi(enable_cc=True)
    homeconf = np.radians([0, -60, 0, 15, 0, 15, 0])
    # robot_s.gen_meshmodel(rgba=[0,1,0,.4]).attach_to(base)
    homeconf[1] = -math.pi/2
    robot_s.fk(component_name=component_name, jnt_values=homeconf)
    robot_s.gen_meshmodel(rgba=[1, 0, 0, .3]).attach_to(base)

    # null space planning
    path = []
    ratio = .01
    for t in range(0, 5000, 1):
        print("-------- timestep = ", t, " --------")
        xa_jacob = robot_s.jacobian()
        # xa_ns = rm.null_space(xa_jacob)
        # cur_jnt_values = robot_s.get_jnt_values()
        # cur_jnt_values += np.ravel(xa_ns)*.01
        # nullspace rotate
        xa_ns = rm.null_space(xa_jacob[:3,:])
        cur_jnt_values = robot_s.get_jnt_values()
        cur_jnt_values += np.ravel(xa_ns[:, 0]) * ratio
        status = robot_s.fk(component_name=component_name, jnt_values=cur_jnt_values)
        if status == "succ":
            if t % 20 == 0:
                path.append(cur_jnt_values)
                robot_s.gen_meshmodel(rgba=[0, 1, 1, .1]).attach_to(base)
        elif status == "out_of_rng":
            ratio=-ratio

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
    taskMgr.doMethodLater(0.1, update, "update",
                          extraArgs=[rbtmnp, motioncounter, robot_s, path, component_name], appendTask=True)
    base.setFrameRateMeter(True)
    base.run()
    