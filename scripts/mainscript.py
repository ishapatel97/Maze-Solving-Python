#!/usr/bin/env python3

import rospy
from autoMove import TurtlebotDriving
import time
from explore import Explore

def main():

    name = "Wall Following"
    algorithm = Explore(speed=0.2, distance_wall=0.5, side="left")
    path, length, timetaken, completed = algorithm.run()

    if completed:
        print("Done")
        print("Time taken :",timetaken,"s\n")

if __name__ == '__main__':
    main()