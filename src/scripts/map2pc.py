import numpy as np
from numpy import trunc,pi,ones_like

import sensor_msgs.point_cloud2 as pc2
import nav_msgs.msg as nav_msgs

def occupancy2pointcloud(grid, position, range=10.0):
    grid_np = np.array(grid.data, dtype='u1').reshape((grid.info.height,grid.info.width))
    origin = np.array((grid.info.origin.position.x,
                       grid.info.origin.position.y,
                       grid.info.origin.position.z))
    world2map = lambda x:trunc((x-origin)/grid.info.resolution)
    ll = world2map(np.r_[position[0]-range, position[1]-range, 0])
    ll = np.clip(ll, zeros((3,)), np.r_[grid.info.height-1,
        grid.info.width-1,0])
    ur = world2map(np.r_[position[0]+range, position[1]+range, 0])
    ur = np.clip(ur, zeros((3,)), np.r_[grid.info.height-1,
        grid.info.width-1,0])

    submap = grid_np[ll[1]:ur[1],ll[0]:ur[0]]
    mappts = np.c_[np.where(submap==100)]
    map2world = lambda x:(x+0.5)*og.info.resolution+origin[:2]
    wpts = np.array([map2world(x) for x in mappts])
    pcpts=[]
    for z in linspace(0,1,11):
        pcpts.append(np.c_[wpts,z*ones_like(wpts[:,0])])
    pcpts = np.vstack(pcpts)
    hdr = grid.header
    pc = pc2.create_cloud_xyz32(hdr, pcpts)
    return pc


