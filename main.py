import os
import argparse
import numpy as np

import sys
sys.path.append('common')
from BVH import load, loadtop, loadbottom, loadMinusTop, loadMinusBottom, loadTwo, save
from Visualize import visualize_anim


def main(src_path1, src_path2, dst_path):
    os.makedirs(dst_path, exist_ok=True)
    # load only top of src1 and only bottom of src2
    # anim1, joint_names1, frame_time1, order1 = loadMinusTop(src_path1)
    # anim2, joint_names2, frame_time2, order2 = loadMinusBottom(src_path2)

    # combine src1 top and src2 bottom into one file
    anim, joint_names, frame_time, order = loadTwo(src_path1, src_path2)
    

    # Save anim to bvh file
    save(os.path.join(dst_path, 'output.bvh'), anim, joint_names, frame_time, order)
    
    # Visualize anim and save as mp4
    name = os.path.basename(src_path1).split('.')[0]
    visualize_anim(anim, title=None, img_dir=os.path.join(dst_path, name), multi_view=False,
                   video_path=os.path.join(dst_path, name + ".mp4"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--src_path1', type=str, default='data/Walking_02.bvh')
    parser.add_argument('--src_path2', type=str, default='data/Samba_Dancing.bvh')
    parser.add_argument('--dst_path', type=str, default='output/')
    args = parser.parse_args()
    main(**vars(args))
