import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import robot_sim.robots.fr5.fr5 as fr5
import motion.probabilistic.rrt_connect as rrtc
import basis.robot_math as rm

base = wd.World(cam_pos=[0, 3, 1], lookat_pos=[0, 0, 0.5], w=960, h=720)
gm.gen_frame().attach_to(base)
# object
object = cm.CollisionModel("objects/mug.stl")
object.set_pos(np.array([0.5, 0, 0.054]))
object.set_rgba([.5, .7, .3, 1])
object.attach_to(base)
# robot_s
component_name = 'arm'
robot_s = fr5.FR5_robot(enable_cc=True)
# start
start_pos = np.array([0, -0.3, 0.6])
start_rotmat = np.array([[1,  0,  0],
                        [0,  1,  0],
                        [0,  0,  1]])
start_conf = robot_s.ik(component_name=component_name, tgt_pos=start_pos, tgt_rotmat=start_rotmat)
# goal
goal_pos = np.array([0.5, 0.4, 0.3])
goal_rotmat = np.array([[1,  0,  0],
                        [0,  1,  0],
                        [0,  0,  1]])
goal_conf = robot_s.ik(component_name=component_name, tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
# planner
rrtc_planner = rrtc.RRTConnect(robot_s)
path = rrtc_planner.plan(component_name=component_name,
                         start_conf=start_conf,
                         goal_conf=goal_conf,
                         obstacle_list=[object],
                         ext_dist=0.2,
                         max_time=300)
# path = [np.array([-2.8170696 , -1.72539192,  1.39924476,  0.3329817 , -0.20066035,
#         1.56339579]), np.array([-2.64380047, -1.79203978,  1.3135923 ,  0.32939927, -0.16535833,
#         1.59565194]), np.array([-2.47053134, -1.85868763,  1.22793984,  0.32581684, -0.13005631,
#         1.62790809]), np.array([-2.25321056, -1.82683864,  1.13688478,  0.29709417, -0.14467949,
#         1.63314931]), np.array([-2.03588979, -1.79498964,  1.04582973,  0.26837151, -0.15930267,
#         1.63839053]), np.array([-1.81856901, -1.76314064,  0.95477467,  0.23964884, -0.17392585,
#         1.64363175]), np.array([-1.60124824, -1.73129164,  0.86371962,  0.21092618, -0.18854904,
#         1.64887296]), np.array([-1.38392746, -1.69944265,  0.77266457,  0.18220351, -0.20317222,
#         1.65411418]), np.array([-1.37757176, -1.58905281,  0.80100822,  0.28663746, -0.249838  ,
#         1.4682025 ]), np.array([-1.32869886, -1.47243196,  0.81153764,  0.38545202, -0.2993647 ,
#         1.28331622]), np.array([-1.27982597, -1.35581111,  0.82206706,  0.48426659, -0.3488914 ,
#         1.09842995]), np.array([-1.23095308, -1.23919026,  0.83259647,  0.58308116, -0.3984181 ,
#         0.91354367]), np.array([-1.21867851, -1.19093277,  0.84117442,  0.63078356, -0.41852155,
#         0.82933984]), np.array([-1.18460123, -1.09064949,  0.85444966,  0.72256819, -0.46071938,
#         0.66265628]), np.array([-1.15052395, -0.9903662 ,  0.8677249 ,  0.81435282, -0.50291722,
#         0.49597272]), np.array([-1.11644666, -0.89008291,  0.88100014,  0.90613745, -0.54511505,
#         0.32928916]), np.array([-1.08236938, -0.78979963,  0.89427537,  0.99792208, -0.58731289,
#         0.1626056 ]), np.array([-1.03636505, -0.65441719,  0.91219694,  1.12183132, -0.64427996,
#        -0.06241721]), np.array([-0.99036072, -0.51903475,  0.93011851,  1.24574057, -0.70124704,
#        -0.28744002]), np.array([-0.98013753, -0.48894977,  0.93410109,  1.27327596, -0.71390639,
#        -0.33744509]), np.array([-1.18435653, -0.53077387,  1.04139478,  1.49673215, -0.67956155,
#        -0.56965442]), np.array([-1.31596245, -0.5703769 ,  1.1079001 ,  1.63639625, -0.65149827,
#        -0.7071132 ]), np.array([-1.44756837, -0.60997993,  1.17440542,  1.77606034, -0.62343498,
#        -0.84457199]), np.array([-1.57917428, -0.64958296,  1.24091074,  1.91572444, -0.5953717 ,
#        -0.98203077]), np.array([-1.7107802 , -0.68918599,  1.30741606,  2.05538853, -0.56730841,
#        -1.11948956]), np.array([-1.69996508, -0.68609241,  1.30188561,  2.04359418, -0.56955878,
#        -1.10772676]), np.array([-1.82837459, -0.76441216,  1.35975755,  2.17667686, -0.52279531,
#        -1.21603203]), np.array([-1.96213449, -0.84599524,  1.42004082,  2.31530465, -0.47408336,
#        -1.32885003]), np.array([-2.09589439, -0.92757832,  1.48032409,  2.45393244, -0.42537141,
#        -1.44166802]), np.array([-2.22965429, -1.0091614 ,  1.54060736,  2.59256023, -0.37665946,
#        -1.55448601]), np.array([-2.36341419, -1.09074448,  1.60089063,  2.73118801, -0.32794751,
#        -1.66730401])]
print(len(path), path)
n = 0.2
for pose in path:
    # print(pose)
    robot_s.fk(component_name, pose)
    robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
    robot_meshmodel.attach_to(base)
    n += 0.05
    # robot_stickmodel = robot_s.gen_stickmodel()
    # robot_stickmodel.attach_to(base)

# def update(rbtmnp, motioncounter, robot, path, armname, task):
#     if motioncounter[0] < len(path):
#         if rbtmnp[0] is not None:
#             rbtmnp[0].detach()
#         pose = path[motioncounter[0]]
#         robot.fk(armname, pose)
#         rbtmnp[0] = robot.gen_meshmodel(toggle_tcpcs=True, rgba=[0.7,0,0,0.7])
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
