import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.ur5e.ur5e as ur5e
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm

base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
gm.gen_frame().attach_to(base)
# object
object = cm.CollisionModel("objects/mug.stl")
object.set_pos(np.array([0.35, 0, 0.45]))
object.set_rgba([.5, .7, .3, 1])
object.attach_to(base)
# robot_s
component_name = 'arm'
robot_s = ur5e.UR5E_robot()
# start
start_pos = np.array([0.3,  -0.4,  0.6])
start_rotmat = np.array([[1,  0, 0],
                         [0,  1, 0],
                         [0,  0, 1]])
start_conf = robot_s.ik(tgt_pos=start_pos, tgt_rotmat=start_rotmat)
# goal
goal_pos = np.array([0.3, 0.4, 0.6])
goal_rotmat = np.array([[1,  0, 0],
                        [0,  1, 0],
                        [0,  0, 1]])
goal_conf = robot_s.ik(tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)

rrtc_planner = rrtc.RRTConnect(robot_s)
path = rrtc_planner.plan(component_name=component_name,
                         start_conf=start_conf,
                         goal_conf=goal_conf,
                         obstacle_list=[object],
                         ext_dist=0.2,
                         max_time=300)
print(len(path), path)
for pose in path:
    # print(pose)
    robot_s.fk(component_name, pose)
    robot_meshmodel = robot_s.gen_meshmodel()
    robot_meshmodel.attach_to(base)
    # robot_stickmodel = robot_s.gen_stickmodel()
    # robot_stickmodel.attach_to(base)

# def update(rbtmnp, motioncounter, robot, path, armname, task):
#     if motioncounter[0] < len(path):
#         if rbtmnp[0] is not None:
#             rbtmnp[0].detach()
#         pose = path[motioncounter[0]]
#         robot.fk(armname, pose)
#         rbtmnp[0] = robot.gen_meshmodel()
#         rbtmnp[0].attach_to(base)
#         motioncounter[0] += 1
#     else:
#         motioncounter[0] = 0
#     return task.again
# rbtmnp = [None]
# motioncounter = [0]
# taskMgr.doMethodLater(0.05, update, "update",
#                       extraArgs=[rbtmnp, motioncounter, robot_s, path, component_name], appendTask=True)
base.setFrameRateMeter(True)
base.run()
