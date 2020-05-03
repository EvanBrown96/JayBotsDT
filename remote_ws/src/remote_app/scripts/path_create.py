#!/usr/bin/env python
from std_msgs.msg import Header
from nav_srvs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import rospy
import copy
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

path_pub = None
goal_pub = None

goal = None
path = None

def create_path(occupancy_map, init_pos, final_pos):
    map_width = occupancy_map.info.width
    map_height = occupancy_map.info.height
    map_res = occupancy_map.info.resolution

    map_data = []
    for i in range(map_height):
        map_data.append(occupancy_map.data[(i*map_width):((i+1)*map_width)])

    # first expand walls to prevent rover from running into them while following path
    new_map = copy.copy(map_data)
    for i, row in enumerate(map_data):
        for j, val in enumerate(row):
            if val == 0:
                new_map[k][l] = 1
            elif val == 100:
                for k in range(max(0, i-5), min(map_width, i+6)):
                    for l in range(max(0, j-5), min(map_height, j+6)):
                        new_map[k][l] = -1

    grid = Grid(matrix=new_map)

    # get indices of start and end positions
    init_coords = grid.node(init_pos.x*map_width*map_res, init_pos.y*map_height*map_res)

    final_coords = grid.node(final_pos.x*map_width*map_res, final_pos.y*map_height*map_res)

    # do A* algorithm
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    p, _ = finder.find_path(init_coords, final_coords, grid)

    # convert path into PoseStamped
    pose_path = []
    h = Header()
    #h.stamp = rospy.Time.now()

    return pose_path

def set_goal(data):
    global goal, path
    goal = data.point

    if current is not None:
        rospy.wait_for_service('dynamic_map')
        get_map = rospy.ServiceProxy('dynamic_map', GetMap)
        try:
            map = get_map()
            path = create_path(map, current, goal)
        except rospy.ServiceException as e:
            rospy.logerror("Get map call failed: " + str(e))

def set_current(data):
    global current
    current = data.pose.position

def setup_pathfinding():
    global path_pub, goal_pub

    rospy.init_node('pathfinding')
    rospy.loginfo("starting pathfinding")
    path_pub = rospy.Publisher('path', Path, queue_size=1)
    rospy.Subscriber('/clicked_point', IDK, set_goal)
    rospy.loginfo("subscribing to /clicked_point")
    rospy.Subscriber('slam_out_pose', PoseStamped, set_current)
    rospy.loginfo("subscribing to slam_out_pose")
    #goal_pub = rospy.Publisher('goal', IDK, queue_size=1)
    #rospy.loginfo("publishing to goal")

if __name__ == "__main__":
    setup_pathfinding()
