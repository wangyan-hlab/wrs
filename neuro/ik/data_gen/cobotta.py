import math
import time
import itertools
import numpy as np
import pandas as pd
import basis.robot_math as rm
import modeling.geometric_model as gm
import visualization.panda.world as world
import robot_sim.robots.cobotta.cobotta as cbt_s
import time
from threading import Thread

# file size: pandas (string) > pickle (binary) = torch.save > numpy, 20211216

def gen_data(rbt_s, num, component_name='arm', granularity=math.pi / 8, save_name='cobotta_ik.csv'):
    n_jnts = rbt_s.manipulator_dict[component_name].ndof
    all_ranges = []
    for jnt_id in range(1, n_jnts + 1):
        r0, r1 = rbt_s.manipulator_dict[component_name].jnts[jnt_id]['motion_rng']
        all_ranges.append(np.arange(r0, r1, granularity))
        # print(granularity, all_ranges[-1])
    all_data = list(itertools.product(*all_ranges))
    avg_len = int(len(all_data)/8)
    n_data = 1
    for rngs in all_ranges:
        n_data = n_data * len(rngs)
    data_set = []
    in_data_npy = np.empty((0, 6))
    for i, data in enumerate(all_data[avg_len*num:avg_len*(num+1)]):
        print(i, n_data)
        rbt_s.fk(component_name=component_name, jnt_values=np.array(data))
        xyz, rotmat = rbt_s.get_gl_tcp(manipulator_name=component_name)
        rpy = rm.rotmat_to_euler(rotmat)
        in_data = (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
        # diff = np.sum(np.abs(np.array(in_data) - in_data_npy), 1)
        # if np.any(diff < 1e-4):
        #     print(diff)
        #     input("Press Enter to continue...")
        in_data_npy = np.vstack((in_data_npy, np.array(in_data)))
        out_data = data
        data_set.append([in_data, out_data])
    # df = pd.DataFrame(data_set, columns=['xyzrpy', 'jnt_values'])
    # df.to_csv(save_name)
    np.save(save_name+"_min_max", np.array([np.min(in_data_npy, 0), np.max(in_data_npy, 0)]))
    np.save(save_name, np.array(data_set))
    print("part finished!")
    time.sleep(0.1)

if __name__ == '__main__':
    base = world.World(cam_pos=np.array([1.5, 1, .7]))
    gm.gen_frame().attach_to(base)
    rbt_s = cbt_s.Cobotta()
    rbt_s.gen_meshmodel(toggle_tcpcs=True).attach_to(base)
    # gen_data(rbt_s, num=0, granularity=math.pi / 8, save_name='cobotta_ik')
    # gen_data(rbt_s, num=0, granularity=math.pi / 8, save_name='cobotta_ik_test')

    granularity = math.pi/6
    thread_01 = Thread(target=gen_data,
                       args=(rbt_s, 0, 'arm', granularity, 'cobotta_ik_jnt1'))
    thread_01.start()
    thread_02 = Thread(target=gen_data,
                       args=(rbt_s, 1, 'arm', granularity, 'cobotta_ik_jnt2'))
    thread_02.start()
    thread_03 = Thread(target=gen_data,
                       args=(rbt_s, 2, 'arm', granularity, 'cobotta_ik_jnt3'))
    thread_03.start()
    thread_04 = Thread(target=gen_data,
                       args=(rbt_s, 3, 'arm', granularity, 'cobotta_ik_jnt4'))
    thread_04.start()
    thread_05 = Thread(target=gen_data,
                       args=(rbt_s, 4, 'arm', granularity, 'cobotta_ik_jnt5'))
    thread_05.start()
    thread_06 = Thread(target=gen_data,
                       args=(rbt_s, 5, 'arm', granularity, 'cobotta_ik_jnt6'))
    thread_06.start()
    thread_07 = Thread(target=gen_data,
                       args=(rbt_s, 6, 'arm', granularity, 'cobotta_ik_jnt7'))
    thread_07.start()
    thread_08 = Thread(target=gen_data,
                       args=(rbt_s, 7, 'arm', granularity, 'cobotta_ik_jnt8'))
    thread_08.start()

    base.run()
