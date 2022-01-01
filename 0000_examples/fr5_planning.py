import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.fr5.fr5 as fr5
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm

base = wd.World(cam_pos=[-2, -3, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
gm.gen_frame().attach_to(base)
# object
object = cm.CollisionModel("objects/mug.stl")
object.set_pos(np.array([0.5, 0, 0.054]))
object.set_rgba([.5, .7, .3, 1])
object.attach_to(base)
object1 = cm.CollisionModel("objects/mug.stl")
object1.set_pos(np.array([-0.5, 0, 0.054]))
object1.set_rgba([.5, .9, .1, 1])
object1.attach_to(base)
# robot_s
component_name = 'arm'
robot_s = fr5.FR5_robot(enable_cc=True)
robot_s.hnd.jaw_to(0.07)
# start
# start_pos = np.array([0.4, 0.3, 0.3])
# start_rotmat = np.array([[-1,  0,  0],
#                         [0,  1,  0],
#                         [0,  0,  -1]])
# start_conf = robot_s.ik(component_name=component_name, tgt_pos=start_pos, tgt_rotmat=start_rotmat)
start_conf = np.array([math.pi*120/180,-math.pi*120/180,math.pi*120/180,0,0,0])
# goal
# goal_pos = np.array([0.5, 0.3, 0.4])
# goal_rotmat = np.array([[-1,  0,  0],
#                         [0,  1,  0],
#                         [0,  0,  -1]])
# goal_conf = robot_s.ik(component_name=component_name, tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
goal_conf = np.array([math.pi*0/180,-math.pi*110/180,math.pi*80/180,-math.pi*80/180,-math.pi*70/180,math.pi*20/180])
# planner
rrtc_planner = rrtc.RRTConnect(robot_s)
path = rrtc_planner.plan(component_name=component_name,
                         start_conf=start_conf,
                         goal_conf=goal_conf,
                         obstacle_list=[object, object1],
                         ext_dist=0.1,
                         max_time=300)
print(len(path), path)
robot_s.fk(component_name, path[-2])
robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
robot_meshmodel.attach_to(base)
# n = 0.2
# for pose in path:
#     # print(pose)
#     robot_s.fk(component_name, pose)
#     robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
#     robot_meshmodel.attach_to(base)
#     n += 0.05
    # robot_stickmodel = robot_s.gen_stickmodel()
    # robot_stickmodel.attach_to(base)

# def update(rbtmnp, motioncounter, robot, path, armname, task):
#     if motioncounter[0] < len(path):
#         if rbtmnp[0] is not None:
#             rbtmnp[0].detach()
#         pose = path[motioncounter[0]]
#         robot.fk(armname, pose)
#         rbtmnp[0] = robot.gen_meshmodel(toggle_tcpcs=True)
#         rbtmnp[0].attach_to(base)
#         motioncounter[0] += 1
#     else:
#         motioncounter[0] = 0
#     return task.again
# rbtmnp = [None]
# motioncounter = [0]
# taskMgr.doMethodLater(0.2, update, "update",
#                       extraArgs=[rbtmnp, motioncounter, robot_s, path, component_name], appendTask=True)
base.setFrameRateMeter(True)
base.run()
