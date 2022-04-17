import os
import numpy as np
import basis.robot_math as rm
import robot_sim.robots.cobotta.cobotta_ripps as cbtr
import robot_sim.end_effectors.gripper.cobotta_pipette.cobotta_pipette as cbtg
import modeling.collision_model as cm
import visualization.panda.world as wd
import modeling.geometric_model as gm
import utils
import motion.probabilistic.rrt_connect as rrtc
import motion.trajectory.piecewisepoly_toppra as trajp

if __name__ == '__main__':
    base = wd.World(cam_pos=[1.7, .7, .7], lookat_pos=[0, 0, 0])
    traj_gen = trajp.PiecewisePolyTOPPRA()
    max_vels = [np.pi * .6, np.pi * .4, np.pi, np.pi, np.pi, np.pi * 1.5]
    gm.gen_frame().attach_to(base)

    this_dir, this_filename = os.path.split(__file__)
    file_frame = os.path.join(this_dir, "meshes", "frame_bottom.stl")
    frame_bottom = cm.CollisionModel(file_frame)
    frame_bottom.set_rgba([.55, .55, .55, 1])
    frame_bottom.attach_to(base)

    table_plate = cm.gen_box(extent=[.405, .26, .003])
    table_plate.set_pos([0.07 + 0.2025, .055, .0015])
    table_plate.set_rgba([.87, .87, .87, 1])
    table_plate.attach_to(base)

    file_dispose_box = os.path.join(this_dir, "objects", "tip_rack_cover.stl")
    dispose_box = cm.CollisionModel(file_dispose_box, expand_radius=.007)
    dispose_box.set_rgba([140 / 255, 110 / 255, 170 / 255, 1])
    dispose_box.set_pos(pos=np.array([.14, 0.07, .003]))
    dispose_box.attach_to(base)

    file_tip_rack = os.path.join(this_dir, "objects", "tip_rack.stl")
    tip_rack = utils.Rack96(file_tip_rack)
    tip_rack.set_rgba([140 / 255, 110 / 255, 170 / 255, 1])
    tip_rack.set_pose(pos=np.array([.25, 0.0, .003]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    tip_rack.attach_to(base)

    file_tip = os.path.join(this_dir, "objects", "tip.stl")
    tip = cm.CollisionModel(file_tip)
    tip.set_rgba([200 / 255, 180 / 255, 140 / 255, 1])
    tip_cm_list = []
    for id_x in range(8):
        for id_y in range(12):
            pos, rotmat = tip_rack.get_rack_hole_pose(id_x=id_x, id_y=id_y)
            tip_new = tip.copy()
            tip_new.set_pose(pos, rotmat)
            # gm.gen_frame(pos=pos, rotmat=rotmat).attach_to(base)
            tip_new.attach_to(base)
            tip_cm_list.append(tip_new)

    file_microplate = os.path.join(this_dir, "objects", "microplate_96.stl")
    microplate96 = utils.Microplate96(file_microplate)
    microplate96.set_rgba([200 / 255, 180 / 255, 140 / 255, 1])
    microplate96.set_pose(pos=np.array([.25, 0.1, .003]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    microplate96.attach_to(base)

    file_microplate = os.path.join(this_dir, "objects", "microplate_24.stl")
    microplate24_0 = utils.Microplate96(file_microplate)
    microplate24_0.set_rgba([180 / 255, 180 / 255, 180 / 255, .7])
    # exactly 0.225, 0.31
    microplate24_0.set_pose(pos=np.array([.15, 0.23, -.03]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    microplate24_0.attach_to(base)
    microplate24_1 = microplate24_0.copy()
    microplate24_1.set_pose(pos=np.array([.3, 0.23, -.03]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    microplate24_1.attach_to(base)
    microplate24_2 = microplate24_0.copy()
    microplate24_2.set_pose(pos=np.array([.15, 0.32, -.03]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    microplate24_2.attach_to(base)
    microplate24_3 = microplate24_0.copy()
    microplate24_3.set_pose(pos=np.array([.3, 0.32, -.03]), rotmat=rm.rotmat_from_axangle([0, 0, 1], np.pi / 2))
    microplate24_3.attach_to(base)

    rbt_s = cbtr.CobottaRIPPS()
    init_joint_values = np.radians(np.asarray([0.0, 0.0, 70.0, 0.0, 90.0, 0.0]))
    component_name = "arm"
    rbt_s.fk(component_name=component_name, jnt_values=init_joint_values)
    rbt_s.gen_meshmodel().attach_to(base)
    planner = rrtc.RRTConnect(rbt_s)
    ee_s = cbtg.CobottaPipette()


    id_x = 6
    id_y = 3
    tip_pos, tip_rotmat = tip_rack.get_rack_hole_pose(id_x=id_x, id_y=id_y)
    z_offset = np.array([0, 0, .03])
    base.change_campos_and_lookat_pos(cam_pos=[.3, .0, .12], lookat_pos=tip_pos+z_offset)
    spiral_points = rm.gen_3d_equilateral_verts(pos=tip_pos+z_offset, rotmat=tip_rotmat)
    print(spiral_points)
    pre_point = None
    for point in spiral_points:
        gm.gen_sphere(point, radius=.00016).attach_to(base)
        if pre_point is not None:
            gm.gen_stick(pre_point, point, thickness=.00012).attach_to(base)
        pre_point = point
    base.run()