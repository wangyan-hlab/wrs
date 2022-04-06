import os

import nptyping
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm


class Base(cm.CollisionModel):
    def __init__(self, file):
        super().__init__(initor=file, expand_radius=.009)
        self._hole_pos_list = []
        self._pos_z0 = .037
        self._pos_x0 = .0315
        self._pos_y0 = .0495
        self._x_step = .009
        self._y_step = .009
        self._x_holes = 8
        self._y_holes = 12
        self._reinitialize()

    def _reinitialize(self):
        self._hole_pos_list = []
        pos_z = self._pos_z0
        for id_x in range(self._x_holes):
            pos_x = self._pos_x0 - self._x_step * id_x
            for id_y in range(self._y_holes):
                pos_y = self._pos_y0 - self._y_step * id_y
                self._hole_pos_list.append(np.array([pos_x, pos_y, pos_z]))

    def _update_hole_pos_list(self):
        self._hole_pos_list = list(self.get_rotmat().dot(np.asarray(self._hole_pos_list).T).T + self.get_pos())

    def set_pos(self, pos):
        super().set_pos(pos)
        self._update_hole_pos_list()

    def set_rotmat(self, rotmat):
        super().set_rotmat(rotmat)
        self._update_hole_pos_list()

    def set_pose(self, pos, rotmat):
        super().set_pose(pos, rotmat)
        self._update_hole_pos_list()

    def set_homomat(self, npmat4):
        super().set_homomat(npmat4)
        self._update_hole_pos_list()

    def get_rack_hole_pose(self, id_x, id_y):
        """
        get the rack hole pose given the hole id
        :param id_x, id_y: (0,0) indicates the upper_left corner when the rack is at a 12 row x 8 column view
        :return:
        author: weiwei
        date: 20220403
        """
        id = id_x * 12 + id_y
        return self._hole_pos_list[id], self.get_rotmat()

    def copy(self):
        return Base(self)


class Base96(Base):
    def __init__(self, file):
        super().__init__(file)


class Rack96(Base96):
    def __init__(self, file):
        super().__init__(file)


class Microplate96(Base96):
    def __init__(self, file):
        super().__init__(file)


class Base24(Base):
    def __init__(self, file):
        super().__init__(file)
        self._pos_z0 = .02
        self._pos_x0 = .0295
        self._pos_y0 = .049
        self._x_step = .019667
        self._y_step = .0196
        self._x_holes = 4
        self._y_holes = 6
        self._reinitialize()

    def copy(self):
        return Base24(self)


class Microplate24(Base24):
    def __init__(self, file):
        super().__init__(file)


def search_reachable_configuration(rbt_s,
                                   ee_s,
                                   component_name,
                                   tgt_pos,
                                   cone_axis,
                                   cone_angle=0,
                                   rotation_interval=np.radians(22.5),
                                   obstacle_list=[],
                                   seed_jnt_values=None,
                                   toggle_debug=False) -> np.typing.NDArray:
    """
    search reachable configuration in a cone
    when the cone_angle is 0, the function degenerates into a search around the cone_axis
    :param rbt_s: instance of a robot
    :param ee_s: instance of an end-effector
    :param tgt_pos:
    :param cone_axis:
    :param cone_angle:
    :param granularity:
    :param obstacle_list
    :return:
    author: weiwei
    date: 20220404
    """
    jnt_values_bk = rbt_s.get_jnt_values(component_name=component_name)
    if seed_jnt_values is None:
        seed_jnt_values = jnt_values_bk
    rotmat_list = []
    if cone_angle != 0:
        rotmat_list = rm.gen_icorotmats(icolevel=3,
                                        rotation_interval=rotation_interval,
                                        crop_normal=-cone_axis,
                                        crop_angle=cone_angle,
                                        toggle_flat=True)
    else:
        rotmat = rm.rotmat_from_axangle([0, 0, 1], 0).dot(rm.rotmat_from_normal(cone_axis))
        for angle in np.linspace(0, np.pi * 2, int(np.pi * 2 / rotation_interval), endpoint=False):
            rotmat_list.append(rm.rotmat_from_axangle([0, 0, 1], -angle).dot(rotmat))
    for rotmat in rotmat_list:
        jnt_values = rbt_s.ik(component_name=component_name,
                              tgt_pos=tgt_pos,
                              tgt_rotmat=rotmat,
                              seed_jnt_values=seed_jnt_values)
        if jnt_values is not None:
            rbt_s.fk(jnt_values=jnt_values)
            if rbt_s.is_collided(obstacle_list=obstacle_list):
                if toggle_debug:
                    rbt_s.gen_meshmodel(rgba=[.9, .5, 0, .3]).attach_to(base)
            else:
                if toggle_debug:
                    rbt_s.gen_meshmodel().attach_to(base)
                if not toggle_debug:
                    rbt_s.fk(component_name=component_name,
                             jnt_values=jnt_values_bk)
                    return jnt_values
        else:
            if toggle_debug:
                ee_s.grip_at_with_jcpose(gl_jaw_center_pos=tgt_pos,
                                         gl_jaw_center_rotmat=rotmat,
                                         jaw_width=0)
                ee_s.gen_meshmodel(rgba=[1, 0, 0, .3]).attach_to(base)
    rbt_s.fk(component_name=component_name, jnt_values=jnt_values_bk)
    return None
