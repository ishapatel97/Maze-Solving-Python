#!/usr/bin/env python3
import rospy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
import numpy as np

class MapCoverage:
    map_grid=[]

    def mapCB(self,data):
        self.map_grid = np.array(data.data)
        # print(data.info)
        self.map_grid = self.map_grid.reshape(data.info.width,data.info.height,order='F')
        
        grid_covered = 0
        total_grids=384*384
        i, j = 1, 1
        side = 7
        # int(math.floor((max(2, 3) / self.resolution) / 2))
        for s_i in range(i,384):
            for s_j in range(j, 384):
                cell = self.map_grid[s_i][s_j]
                if cell != -1:
                    grid_covered+=1
        percentage = (grid_covered/total_grids)*100
	    
        self.pub.publish(percentage)
        print(percentage,"%")

    def __init__(self):

        rospy.init_node('map_listner', anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.mapCB)
        self.pub = rospy.Publisher('map_coverage_percentage', Float64,queue_size=10)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    map_cover = MapCoverage()
    # getMapCoverage()
