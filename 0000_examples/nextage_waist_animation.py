import math
import numpy as np
import basis.robot_math as rm
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.nextage.nextage as nxt
import motion.probabilistic.rrt_connect as rrtc

def genSphere(pos, radius=0.02, rgba=None):
    if rgba is None:
        rgba = [1, 0, 0, 1]
    gm.gen_sphere(pos=pos, radius=radius, rgba=rgba).attach_to(base)

if __name__ == '__main__':

    base = wd.World(cam_pos=[4, -1, 2], lookat_pos=[0, 0, 0], backgroundcolor=[.4,.4,.4,1])
    gm.gen_frame().attach_to(base)
    # object
    object_box = cm.gen_box(extent=[.15,.15,.15])
    object_box.set_pos(np.array([.4, .3, .4]))
    object_box.set_scale([.7,.7,.7])
    object_box.set_rgba([.5, .7, .3, 1])
    object_box.attach_to(base)
    # robot_s
    component_name = 'lft_arm_waist'
    robot_s = nxt.Nextage()

    start_conf = np.radians([45, 0, -60, -120, 0, 0, 0])
    goal_conf = np.radians([-35, 0, -40, -100, 0, 30, 0])
    rrtc_planner = rrtc.RRTConnect(robot_s)
    path = rrtc_planner.plan(component_name=component_name,
                             start_conf=start_conf,
                             goal_conf=goal_conf,
                             obstacle_list=[object_box],
                             ext_dist=.05,
                             smoothing_iterations=150,
                             max_time=300)
    print(path)
    for pose in path[1:-2]:
        print(pose)
        robot_s.fk(component_name, pose)
        # robot_s.gen_stickmodel().attach_to(base)

    robot_attached_list = []
    counter = [0]
    def update(robot_s, path, robot_attached_list, counter, task):
        if counter[0] >= len(path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            robot_attached_list.clear()
        pose = path[counter[0]]
        robot_s.fk(component_name, pose)
        robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)
        genSphere(robot_s.get_gl_tcp(component_name)[0], radius=0.01, rgba=[1, 1, 0, 1])
        counter[0]+=1
        return task.again

    taskMgr.doMethodLater(0.07, update, "update",
                          extraArgs=[robot_s, path[1:-1:3], robot_attached_list, counter],
                          appendTask=True)

    base.run()
