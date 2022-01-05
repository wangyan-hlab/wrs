import os
import math
import numpy as np
import basis.robot_math as rm
import modeling.model_collection as mc
import modeling.collision_model as cm
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.fr5.fr5 as fr
import robot_sim.end_effectors.grippers.robotiq85.robotiq85 as rtq
from panda3d.core import CollisionNode, CollisionBox, Point3
import robot_sim.robots.robot_interface as ri

class FR5_robot(ri.RobotInterface):
    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name='fr5', enable_cc=True, showhnd=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        self.table = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=np.zeros(1), name="fr5_to_table")
        self.table.jnts[0]['loc_pos'] = np.array([0, 0, 0])
        self.table.jnts[1]['loc_pos'] = np.array([0, 0, 0])
        self.table.lnks[0]['name'] = "table"
        self.table.lnks[0]['loc_pos'] = np.array([0, 0, 0])
        self.table.lnks[0]['collisionmodel'] = cm.CollisionModel(
            os.path.join(this_dir, "meshes/table1820x54x800.stl"), expand_radius=.005
        )
        self.table.lnks[0]['rgba'] = [.3, .3, .3, 1.0]
        self.table.reinitialize()
        arm_homeconf = np.zeros(6)
        self.offset = np.array([0, 0, 0.054])
        self.arm = fr.FR5(pos=self.table.jnts[-1]['gl_posq']+self.offset,
                          rotmat=self.table.jnts[-1]['gl_rotmatq'],
                          homeconf=arm_homeconf,
                          enable_cc=False)
        self.hnd = rtq.Robotiq85(pos=self.arm.jnts[-1]['gl_posq'],
                                 rotmat=self.arm.jnts[-1]['gl_rotmatq'],
                                 enable_cc=False)
        if enable_cc:
            self.enable_cc()
        self.manipulator_dict['arm'] = self.arm
        self.hnd_dict['arm'] = self.hnd
        self.showhnd = showhnd

    def enable_cc(self):
        super().enable_cc()
        self.cc.add_cdlnks(self.table, [0])
        self.cc.add_cdlnks(self.arm, [0, 1, 2, 3, 4, 5, 6])
        self.cc.add_cdlnks(self.hnd.lft_outer, [0, 1, 2, 3, 4])
        self.cc.add_cdlnks(self.hnd.rgt_outer, [1, 2, 3, 4])
        # lnks used for cd with external stationary objects
        activelist = [self.arm.lnks[0],
                      self.arm.lnks[1],
                      self.arm.lnks[2],
                      self.arm.lnks[3],
                      self.arm.lnks[4],
                      self.arm.lnks[5],
                      self.arm.lnks[6],
                      self.hnd.lft_outer.lnks[0],
                      self.hnd.lft_outer.lnks[1],
                      self.hnd.lft_outer.lnks[2],
                      self.hnd.lft_outer.lnks[3],
                      self.hnd.lft_outer.lnks[4],
                      self.hnd.rgt_outer.lnks[1],
                      self.hnd.rgt_outer.lnks[2],
                      self.hnd.rgt_outer.lnks[3],
                      self.hnd.rgt_outer.lnks[4]
                      ]
        self.cc.set_active_cdlnks(activelist)
        # lnks used for arm-body collision detection
        fromlist = [self.table.lnks[0]]
        intolist = [self.arm.lnks[2],
                    self.arm.lnks[3],
                    self.arm.lnks[4],
                    self.arm.lnks[5],
                    self.arm.lnks[6],
                    self.hnd.lft_outer.lnks[0],
                    self.hnd.lft_outer.lnks[1],
                    self.hnd.lft_outer.lnks[2],
                    self.hnd.lft_outer.lnks[3],
                    self.hnd.lft_outer.lnks[4],
                    self.hnd.rgt_outer.lnks[1],
                    self.hnd.rgt_outer.lnks[2],
                    self.hnd.rgt_outer.lnks[3],
                    self.hnd.rgt_outer.lnks[4]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.arm.lnks[0],
                    self.arm.lnks[1]]
        intolist = [self.arm.lnks[3],
                    self.arm.lnks[5],
                    self.arm.lnks[6],
                    self.hnd.lft_outer.lnks[0],
                    self.hnd.lft_outer.lnks[1],
                    self.hnd.lft_outer.lnks[2],
                    self.hnd.lft_outer.lnks[3],
                    self.hnd.lft_outer.lnks[4],
                    self.hnd.rgt_outer.lnks[1],
                    self.hnd.rgt_outer.lnks[2],
                    self.hnd.rgt_outer.lnks[3],
                    self.hnd.rgt_outer.lnks[4]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.arm.lnks[2]]
        intolist = [self.arm.lnks[4],
                    self.arm.lnks[5],
                    self.arm.lnks[6],
                    self.hnd.lft_outer.lnks[0],
                    self.hnd.lft_outer.lnks[1],
                    self.hnd.lft_outer.lnks[2],
                    self.hnd.lft_outer.lnks[3],
                    self.hnd.lft_outer.lnks[4],
                    self.hnd.rgt_outer.lnks[1],
                    self.hnd.rgt_outer.lnks[2],
                    self.hnd.rgt_outer.lnks[3],
                    self.hnd.rgt_outer.lnks[4]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.arm.lnks[3]]
        intolist = [self.arm.lnks[6],
                    self.hnd.lft_outer.lnks[0],
                    self.hnd.lft_outer.lnks[1],
                    self.hnd.lft_outer.lnks[2],
                    self.hnd.lft_outer.lnks[3],
                    self.hnd.lft_outer.lnks[4],
                    self.hnd.rgt_outer.lnks[1],
                    self.hnd.rgt_outer.lnks[2],
                    self.hnd.rgt_outer.lnks[3],
                    self.hnd.rgt_outer.lnks[4]]
        self.cc.set_cdpair(fromlist, intolist)

    def get_hnd_on_manipulator(self, manipulator_name):
        if manipulator_name == 'arm':
            return self.hnd
        else:
            raise ValueError("The given jlc does not have a hand!")

    def get_gl_tcp(self, manipulator_name="arm"):
        return super().get_gl_tcp(manipulator_name=manipulator_name)

    def get_jnt_values(self, component_name="arm"):
        if component_name in self.manipulator_dict:
            return self.manipulator_dict[component_name].get_jnt_values()

    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.table.fix_to(self.pos, self.rotmat)
        self.arm.fix_to(pos=self.table.jnts[-1]['gl_posq']+self.offset,
                        rotmat=self.table.jnts[-1]['gl_rotmatq'])
        self.hnd.fix_to(pos=self.arm.jnts[-1]['gl_posq'],
                        rotmat=self.arm.jnts[-1]['gl_rotmatq'])

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
            self.get_hnd_on_manipulator(component_name).fix_to(
                pos=self.manipulator_dict[component_name].jnts[-1]['gl_posq'],
                rotmat=self.manipulator_dict[component_name].jnts[-1]['gl_rotmatq'])

        super().fk(component_name, jnt_values)
        # examine length
        if component_name in self.manipulator_dict:
            if not isinstance(jnt_values, np.ndarray) or jnt_values.size != 6:
                raise ValueError("An 1x6 npdarray must be specified to move the arm!")
            update_component(component_name, jnt_values)
        elif component_name == "fr5_to_table":
            self.table.fk(jnt_values)
            self.arm.fix_to(pos=self.table.jnts[-1]['gl_posq']+self.offset,
                            rotmat=self.table.jnts[-1]['gl_rotmatq'])
        else:
            raise ValueError("The given component name is not available!")

    def gen_meshmodel(self,
                      tcp_jntid=None,
                      tcp_loc_pos=None,
                      tcp_loc_rotmat=None,
                      toggle_tcpcs=False,
                      toggle_jntscs=False,
                      rgba=None,
                      name='fr5_meshmodel'):
        meshmodel = mc.ModelCollection(name=name)
        self.table.gen_meshmodel(tcp_loc_pos=None,
                                 tcp_loc_rotmat=None,
                                 toggle_tcpcs=False,
                                 toggle_jntscs=toggle_jntscs,
                                 rgba=rgba).attach_to(meshmodel)
        self.arm.gen_meshmodel(tcp_jntid=tcp_jntid,
                               tcp_loc_pos=tcp_loc_pos,
                               tcp_loc_rotmat=tcp_loc_rotmat,
                               toggle_tcpcs=toggle_tcpcs,
                               toggle_jntscs=toggle_jntscs,
                               rgba=rgba).attach_to(meshmodel)
        if self.showhnd:
            self.hnd.gen_meshmodel(toggle_tcpcs=False,
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
                       name='fr5_stickmodel'):
        stickmodel = mc.ModelCollection(name=name)
        self.table.gen_stickmodel(tcp_loc_pos=None,
                                  tcp_loc_rotmat=None,
                                  toggle_tcpcs=False,
                                  toggle_jntscs=toggle_jntscs).attach_to(stickmodel)
        self.arm.gen_stickmodel(tcp_jntid=tcp_jntid,
                                tcp_loc_pos=tcp_loc_pos,
                                tcp_loc_rotmat=tcp_loc_rotmat,
                                toggle_tcpcs=toggle_tcpcs,
                                toggle_jntscs=toggle_jntscs,
                                toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        if self.showhnd:
            self.hnd.gen_stickmodel(toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        return stickmodel


if __name__ == '__main__':
    import time
    import visualization.panda.world as wd
    import modeling.geometric_model as gm

    base = wd.World(cam_pos=[2, 2, 1], lookat_pos=[0, 0, 0], w=960, h=720)
    gm.gen_frame().attach_to(base)
    fr5 = FR5_robot()
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
    # pos1 = np.array([-0.03,  0.31,  0.42])
    # rotmat1 = np.array([[0,  0.866, -0.5],
    #                     [0,  0.5,   0.866],
    #                     [1,  0,     0]])
    # conf1 = fr5.ik(component_name="arm", tgt_pos=pos1, tgt_rotmat=rotmat1)
    # fr5.fk(component_name="fr5_to_table", jnt_values=np.array([math.pi/3.0]))
    conf1 = np.array([0, -45/180*math.pi, 0, 0, 0, 0])
    fr5.fk(component_name="arm", jnt_values=conf1)
    fr5.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    print(fr5.get_gl_tcp())

    # pos2 = np.array([-0.03, 0.51, 0.42])
    # rotmat2 = np.array([[0, 0.866, -0.5],
    #                     [0, 0.5, 0.866],
    #                     [1, 0, 0]])
    # # conf2 = fr5.ik(component_name="arm", tgt_pos=pos2, tgt_rotmat=rotmat2)
    conf2 = np.array([90/180*math.pi, -90/180*math.pi, -90/180*math.pi, -90/180*math.pi, 90/180*math.pi, 0/180*math.pi])
    fr5.fk(component_name="arm", jnt_values=conf2)
    fr5.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    print(fr5.get_gl_tcp())

    # fr5.fix_to(np.ones(3), np.eye(3))
    # fr5.gen_meshmodel().attach_to(base)
    # jnt_val = fr5.get_jnt_values()
    # print(jnt_val)
    # tcp = fr5.get_gl_tcp(manipulator_name="arm")
    # print(tcp)
    # gm.gen_sphere(tcp[0], radius=.02, rgba=[0.0, 0.0, 1.0, 1.0]).attach_to(base)

    # fr5.show_cdprimit()   # show the collision model
    # fr5.gen_stickmodel().attach_to(base)
    # tic = time.time()
    # print(fr5.is_collided())
    # toc = time.time()
    # print(toc - tic)

    base.run()