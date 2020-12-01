#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import pcl
import pcl.pcl_visualization
from os import listdir
from os.path import join

# not useful

def main():
    folder = "/home/yufeng/Temp/Scanning/"
    files = []
    final_cloud = None
    while True:
        pc_files = listdir(folder)
        newcloud = False
        cloud = None
        for f in pc_files:
            print(f)
            # new point cloud
            if f not in files:
                newpc = pcl.load(join(folder,f));
                new = process(newpc)
                if cloud == None:
                    cloud = new
                else:
                    cloud = cloud + new
                files.append(f)
                newcloud = True

        if newcloud == True:
            if final_cloud == None:
                final_cloud = cloud
            else:
                final_cloud = final_cloud + cloud
            visual.pcl.pcl_visualization.CloudViewing()
            visual.ShowMonochromeCloud(final_cloud, b'cloud')

def process(cloud):
    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.0)
    return fil.filter()

if __name__ == "__main__":
    main()
