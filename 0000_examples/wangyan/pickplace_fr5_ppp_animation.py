import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import math
import numpy as np
import basis.robot_math as rm
import robot_sim.robots.fr5.fr5 as fr5
import manipulation.pick_place_planner as ppp
import motion.probabilistic.rrt_connect as rrtc

if __name__ == '__main__':

    base = wd.World(cam_pos=[2.1, -2.1, 2.1], lookat_pos=[.0, 0, .3])
    gm.gen_frame().attach_to(base)
    # object to grasp
    tubebig = cm.CollisionModel("../objects/tubebig.stl")
    tubebig.set_rgba([.9, .75, .35, 1])
    # object start
    tubebig_gl_pos = np.array([-0.3, -0.3, 0.6])
    tubebig_gl_rotmat = rm.rotmat_from_euler(0, math.pi/2, 0)
    obgl_start_homomat = rm.homomat_from_posrot(tubebig_gl_pos, tubebig_gl_rotmat)
    tubebig.set_pos(tubebig_gl_pos)
    tubebig.set_rotmat(tubebig_gl_rotmat)
    gm.gen_frame().attach_to(tubebig)
    tubebig_copy = tubebig.copy()
    tubebig_copy.set_rgba([1, 0, 0, .4])
    tubebig_copy.attach_to(base)
    # object goal
    tubebig_gl_goal_pos = np.array([-0.4, -0.4, 0.55])
    tubebig_gl_goal_rotmat = rm.rotmat_from_euler(0, math.pi/2, 0)
    obgl_goal_homomat = rm.homomat_from_posrot(tubebig_gl_goal_pos, tubebig_gl_goal_rotmat)
    tubebig_goal_copy = tubebig.copy()
    tubebig_goal_copy.set_rgba([0, 1, 0, .4])
    tubebig_goal_copy.set_homomat(obgl_goal_homomat)
    tubebig_goal_copy.attach_to(base)

    homeconf = np.array([-40, -60, -80, -120, 75, 20])*math.pi/180
    robot_s = fr5.FR5_robot(homeconf=homeconf)
    robot_s.gen_meshmodel(rgba=[1, 0, 1, .3]).attach_to(base)
    # base.run()

    obj = cm.CollisionModel("../objects/bunnysim.stl")
    obj.set_pos(robot_s.get_gl_tcp()[0]+np.array([0.15, 0, 0.03]))
    obj.set_rpy(0, 0, 0)
    obj.set_scale([1.5, 1.5, 1.5])
    obj.set_rgba([.1, .2, .8, 1])
    obj.attach_to(base)

    rrtc_s = rrtc.RRTConnect(robot_s)
    ppp_s = ppp.PickPlacePlanner(robot_s)

    original_grasp_info_list = gpa.load_pickle_file('tubebig', './', 'fr5_tubebig.pickle')
    hnd_name = "hnd"
    start_conf = robot_s.get_jnt_values(hnd_name)
    conf_list, jawwidth_list, objpose_list = \
        ppp_s.gen_pick_and_place_motion(hnd_name=hnd_name,
                                        objcm=tubebig,
                                        grasp_info_list=original_grasp_info_list,
                                        start_conf=start_conf,
                                        end_conf=start_conf,
                                        goal_homomat_list=[obgl_start_homomat, obgl_goal_homomat],
                                        approach_direction_list=[None, np.array([-1, 0, 0])],
                                        approach_distance_list=[.2] * 2,
                                        depart_direction_list=[np.array([1, 0, 0]), None],
                                        depart_distance_list=[.2] * 2,
                                        obstacle_list=[obj])
    robot_attached_list = []
    object_attached_list = []
    counter = [0]
    def update(robot_s,
               hnd_name,
               tubebig,
               robot_path,
               jawwidth_path,
               obj_path,
               robot_attached_list,
               object_attached_list,
               counter,
               task):
        if counter[0] >= len(robot_path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            object_attached_list.clear()
        pose = robot_path[counter[0]]
        robot_s.fk(hnd_name, pose)
        robot_s.jaw_to(hnd_name, jawwidth_path[counter[0]])
        robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)
        obj_pose = obj_path[counter[0]]
        objb_copy = tubebig.copy()
        objb_copy.set_homomat(obj_pose)
        objb_copy.attach_to(base)
        object_attached_list.append(objb_copy)
        counter[0] += 1
        return task.again
    taskMgr.doMethodLater(0.02, update, "update",
                          extraArgs=[robot_s,
                                     hnd_name,
                                     tubebig,
                                     conf_list,
                                     jawwidth_list,
                                     objpose_list,
                                     robot_attached_list,
                                     object_attached_list,
                                     counter],
                          appendTask=True)
    base.setFrameRateMeter(True)
    base.run()
