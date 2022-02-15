import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.gripper.robotiq85_gelsight.robotiq85_gelsight as rtq85
import math

base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
gm.gen_frame().attach_to(base)
# object
obj = cm.CollisionModel("tubebig.stl")
obj.set_rgba([.9, .75, .35, .3])
obj.attach_to(base)
# hnd_s
gripper_s = rtq85.Robotiq85Gelsight()
grasp_info_list = gpa.plan_grasps(gripper_s, obj,
                                  angle_between_contact_normals=math.radians(170), openning_direction='loc_y',
                                  max_samples=100, min_dist_between_sampled_contact_points=.01,
                                  rotation_interval=math.radians(180))
print(len(grasp_info_list))
for grasp_info in grasp_info_list:
    # print(grasp_info)
    jaw_width, gl_jaw_center, gl_jaw_rotmat, hnd_pos, hnd_rotmat = grasp_info
    gripper_s.jaw_to(jaw_width)
    gripper_s.fix_to(hnd_pos, hnd_rotmat)
    gripper_s.gen_meshmodel().attach_to(base)
base.run()