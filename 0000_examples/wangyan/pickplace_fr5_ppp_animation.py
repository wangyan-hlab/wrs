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

    robot_s = fr5.FR5_robot(zrot_to_gndbase=0, arm_jacobian_offset=np.array([0, 0, .145]), hnd_attached=True)
    tgt_pos = np.array([-0.4, 0, 0.6])
    tgt_rotmat = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    jnt_values = robot_s.ik(component_name='arm', tgt_pos=tgt_pos, tgt_rotmat=tgt_rotmat,
                            seed_jnt_values=np.radians([-15,-60,20,0,-90,-90]))
    print(np.degrees(jnt_values))
    robot_s.fk(component_name="arm", jnt_values=jnt_values)
    robot_s.gen_meshmodel(toggle_tcpcs=True, rgba=[1, 0, 1, .3]).attach_to(base)
    # base.run()

    tcp_pos = robot_s.get_gl_tcp()[0]
    obstacle = cm.CollisionModel("../objects/hole.stl")
    obstacle.set_pos(tcp_pos + np.array([0, 0.15, -0.16]))
    obstacle.set_rpy(0, 0, 0)
    obstacle.set_rgba([.9, .2, .2, 1])
    obstacle.attach_to(base)

    # object to grasp
    obj_name = 'peg'
    obj = cm.CollisionModel("../objects/" + obj_name + ".stl")
    obj.set_rgba([.9, .75, .35, 1])

    # object start
    obj_gl_pos = tcp_pos + np.array([0, 0, -0.1])
    obj_gl_rotmat = rm.rotmat_from_euler(0, 0, 0)
    obgl_start_homomat = rm.homomat_from_posrot(obj_gl_pos, obj_gl_rotmat)
    obj.set_pos(obj_gl_pos)
    obj.set_rotmat(obj_gl_rotmat)
    gm.gen_frame().attach_to(obj)
    obj_copy = obj.copy()
    obj_copy.set_rgba([1, 0, 0, .4])
    obj_copy.attach_to(base)
    # object goal
    obj_gl_goal_pos = tcp_pos + np.array([0, 0.15, -0.1])
    obj_gl_goal_rotmat = rm.rotmat_from_euler(0, 0, 0)
    obgl_goal_homomat = rm.homomat_from_posrot(obj_gl_goal_pos, obj_gl_goal_rotmat)
    obj_goal_copy = obj.copy()
    obj_goal_copy.set_rgba([0, 1, 0, .4])
    obj_goal_copy.set_homomat(obgl_goal_homomat)
    obj_goal_copy.attach_to(base)
    # base.run()

    rrtc_s = rrtc.RRTConnect(robot_s)
    ppp_s = ppp.PickPlacePlanner(robot_s)

    original_grasp_info_list = gpa.load_pickle_file(obj_name, './', 'fr5_'+obj_name+'.pickle')
    hnd_name = "hnd"
    start_conf = robot_s.get_jnt_values(hnd_name)
    conf_list, jawwidth_list, objpose_list = \
        ppp_s.gen_pick_and_place_motion(hnd_name=hnd_name,
                                        objcm=obj,
                                        grasp_info_list=original_grasp_info_list,
                                        start_conf=start_conf,
                                        end_conf=start_conf,
                                        goal_homomat_list=[obgl_start_homomat, obgl_goal_homomat],
                                        approach_direction_list=[np.array([0,0,-1]), np.array([0,0,-1])],
                                        approach_distance_list=[.05] * 2,
                                        depart_direction_list=[np.array([0,0,1]), np.array([0,0,1])],
                                        depart_distance_list=[.05] * 2,
                                        obstacle_list=[])
    robot_attached_list = []
    object_attached_list = []
    counter = [0]
    def update(robot_s,
               hnd_name,
               obj,
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
        objb_copy = obj.copy()
        objb_copy.set_homomat(obj_pose)
        objb_copy.attach_to(base)
        object_attached_list.append(objb_copy)
        counter[0] += 1
        return task.again
    taskMgr.doMethodLater(0.04, update, "update",
                          extraArgs=[robot_s,
                                     hnd_name,
                                     obj,
                                     conf_list,
                                     jawwidth_list,
                                     objpose_list,
                                     robot_attached_list,
                                     object_attached_list,
                                     counter],
                          appendTask=True)
    base.setFrameRateMeter(True)
    base.run()
