import os
import math
import numpy as np
import basis.robot_math as rm
import modeling.model_collection as mc
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.ur5e.ur5e as ur5e
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq
import robot_sim.robots.robot_interface as ri

class UR5E_robot(ri.RobotInterface):
    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name='ur5e', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        arm_homeconf = np.zeros(6)
        self.arm = ur5e.UR5E(pos=np.zeros(3),rotmat=np.eye(3),homeconf=arm_homeconf,enable_cc=False)
        if enable_cc:
            self.enable_cc()
        self.manipulator_dict['arm'] = self.arm

    def enable_cc(self):
        super().enable_cc()
        self.cc.add_cdlnks(self.arm, [0, 1, 2, 3, 4, 5, 6])
        activelist = [self.arm.lnks[0],
                      self.arm.lnks[1],
                      self.arm.lnks[2],
                      self.arm.lnks[3],
                      self.arm.lnks[4],
                      self.arm.lnks[5],
                      self.arm.lnks[6]]
        self.cc.set_active_cdlnks(activelist)
        fromlist = [self.arm.lnks[0],
                    self.arm.lnks[1]]
        intolist = [self.arm.lnks[3],
                    self.arm.lnks[5],
                    self.arm.lnks[6]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.arm.lnks[2]]
        intolist = [self.arm.lnks[4],
                    self.arm.lnks[5],
                    self.arm.lnks[6]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.arm.lnks[3]]
        intolist = [self.arm.lnks[6]]
        self.cc.set_cdpair(fromlist, intolist)

    def get_gl_tcp(self, manipulator_name="arm"):
        return super().get_gl_tcp(manipulator_name=manipulator_name)

    def get_jnt_values(self, component_name="arm"):
        if component_name in self.manipulator_dict:
            return self.manipulator_dict[component_name].get_jnt_values()

    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.arm.fix_to(self.pos, self.rotmat)

    def fk(self, component_name, jnt_values):
        """
        :param jnt_values: nparray 1x6
        :param component_name:
        :return:
        author: weiwei
        date: 20201208toyonaka, 20210403osaka
        """

        def update_component(component_name='arm', jnt_values=np.zeros(6)):
            self.manipulator_dict[component_name].fk(jnt_values=jnt_values)
        super().fk(component_name, jnt_values)
        # examine length
        if component_name in self.manipulator_dict:
            if not isinstance(jnt_values, np.ndarray) or jnt_values.size != 6:
                raise ValueError("An 1x6 npdarray must be specified to move the arm!")
            update_component(component_name, jnt_values)
        else:
            raise ValueError("The given component name is not available!")

    def gen_meshmodel(self,
                      tcp_jntid=None,
                      tcp_loc_pos=None,
                      tcp_loc_rotmat=None,
                      toggle_tcpcs=False,
                      toggle_jntscs=False,
                      rgba=None,
                      name='ur5e_meshmodel'):
        meshmodel = mc.ModelCollection(name=name)
        self.arm.gen_meshmodel(tcp_jntid=tcp_jntid,
                               tcp_loc_pos=tcp_loc_pos,
                               tcp_loc_rotmat=tcp_loc_rotmat,
                               toggle_tcpcs=toggle_tcpcs,
                               toggle_jntscs=toggle_jntscs,
                               rgba=rgba).attach_to(meshmodel)
        return meshmodel

    def gen_stickmodel(self,
                       tcp_jntid=None,
                       tcp_loc_pos=None,
                       tcp_loc_rotmat=None,
                       toggle_tcpcs=False,
                       toggle_jntscs=False,
                       toggle_connjnt=False,
                       name='ur5e_stickmodel'):
        stickmodel = mc.ModelCollection(name=name)
        self.arm.gen_stickmodel(tcp_jntid=tcp_jntid,
                                tcp_loc_pos=tcp_loc_pos,
                                tcp_loc_rotmat=tcp_loc_rotmat,
                                toggle_tcpcs=toggle_tcpcs,
                                toggle_jntscs=toggle_jntscs,
                                toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        return stickmodel


if __name__ == '__main__':
    import time
    import visualization.panda.world as wd
    import modeling.geometric_model as gm

    base = wd.World(cam_pos=[-2, -2, 1], lookat_pos=[0, 0, 0], w=960, h=720)
    gm.gen_frame().attach_to(base)
    ur5e = UR5E_robot(enable_cc=True)
    # fk test
    # gl_conf = np.zeros(6)
    # gl_conf[0] = math.pi * 2.0 / 3.0
    # gl_conf[1] = -math.pi * 1.0 / 3.0
    # gl_conf[2] = -math.pi * 2.0 / 3.0
    # gl_conf[3] = math.pi
    # gl_conf[4] = -math.pi / 2.0
    # gl_conf[5] = math.pi / 2.0
    # fr5.fk(component_name="arm", jnt_values=gl_conf)
    # fr5.gen_meshmodel().attach_to(base)
    # tcp = fr5.get_gl_tcp(manipulator_name="arm")
    # print(tcp)
    pos1 = np.array([-0.03,  0.31,  0.42])
    rotmat1 = np.array([[0,  0.866, -0.5],
                        [0,  0.5,   0.866],
                        [1,  0,     0]])
    conf1 = ur5e.ik(component_name="arm", tgt_pos=pos1, tgt_rotmat=rotmat1)
    ur5e.fk(component_name="arm", jnt_values=conf1)
    ur5e.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    pos2 = np.array([-0.03, 0.51, 0.42])
    rotmat2 = np.array([[0, 0.866, -0.5],
                        [0, 0.5, 0.866],
                        [1, 0, 0]])
    conf2 = ur5e.ik(component_name="arm", tgt_pos=pos2, tgt_rotmat=rotmat2)
    ur5e.fk(component_name="arm", jnt_values=conf2)
    ur5e.gen_meshmodel(toggle_tcpcs=True).attach_to(base)

    # fr5.fix_to(np.ones(3), np.eye(3))
    # fr5.gen_meshmodel().attach_to(base)
    # jnt_val = fr5.get_jnt_values()
    # print(jnt_val)
    # tcp = fr5.get_gl_tcp(manipulator_name="arm")
    # print(tcp)
    # gm.gen_sphere(tcp[0], radius=.02, rgba=[0.0, 0.0, 1.0, 1.0]).attach_to(base)

    # manipulator_meshmodel.show_cdprimit()   # show the collision model
    # fr5.gen_stickmodel().attach_to(base)
    # tic = time.time()
    # print(fr5.is_collided())
    # toc = time.time()
    # print(toc - tic)

    base.run()