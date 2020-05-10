#!/usr/bin/env python
from std_msgs.msg import Header
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, PointStamped
import rospy
import copy
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from tf.transformations import quaternion_from_euler
import math

path_pub = None
goal_pub = None

goal = None
current = None
path = None

def create_path(occupancy_map, init_pos, final_pos):
    map_width = occupancy_map.info.width
    map_height = occupancy_map.info.height
    map_res = occupancy_map.info.resolution

    rospy.loginfo("making map 2D")
    map_data = []
    for i in range(map_height):
        map_data.append(occupancy_map.data[(i*map_width):((i+1)*map_width)])

    rospy.loginfo("expanding walls")
    # first expand walls to prevent rover from running into them while following path
    new_map = [[-1 for _ in range(map_width)] for _ in range(map_height)]
    for i, row in enumerate(map_data):
        for j, val in enumerate(row):
            if val == 0:
                new_map[i][j] = 1
            elif val == 100:
                #new_map[i][j] = -1
                for k in range(max(0, i-6), min(map_width, i+7)):
                    for l in range(max(0, j-6), min(map_height, j+7)):
                        new_map[k][l] = -1

    rospy.loginfo("creating grid")
    grid = Grid(matrix=new_map)

    rospy.loginfo("running A*")
    # get indices of start and end positions
    init_x = int(round((init_pos.x/map_res)+(map_width/2)))
    init_y = int(round((init_pos.y/map_res)+(map_height/2)))
    final_x = int(round((final_pos.x/map_res)+(map_width/2)))
    final_y = int(round((final_pos.y/map_res)+(map_height/2)))
    init_coords = grid.node(init_x, init_y)
    final_coords = grid.node(final_x, final_y)
    final_coords = grid.node(int((final_pos.x/map_res)+(map_width/2)), int((final_pos.y/map_res)+(map_height/2)))

    # do A* algorithm
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    p, _ = finder.find_path(init_coords, final_coords, grid)
    rospy.loginfo(p)
    rospy.loginfo("transforming to poses")
    # convert path into PoseStamped
    pose_path = []
    direction = (p[1][0]-p[0][0], p[1][1]-p[0][1])
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = occupancy_map.header.frame_id
    for i in range(1, len(p)):
        cur_dir = (p[i][0]-p[i-1][0], p[i][1]-p[i-1][1])
        if direction == cur_dir and i < len(p)-1:
            continue # don't add point if it's moving in the same direction
        direction = cur_dir

        x = map_res*(p[i][0]-(map_width/2))
        y = map_res*(p[i][1]-(map_height/2))
        point = Point(x, y, 0)
        pose = Pose(point, Quaternion(0, 0, 0, 0))
        pose_stamped = PoseStamped(h, pose)
        pose_path.append(pose_stamped)

    return Path(h, pose_path)

def set_goal(data):
    global goal, path
    rospy.loginfo("set goal")
    goal = data.point

    if current is not None:
        rospy.wait_for_service('dynamic_map')
        get_map = rospy.ServiceProxy('dynamic_map', GetMap)
        try:
            map_result = get_map()
            path = create_path(map_result.map, current, goal)
            path_pub.publish(path)
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
    rospy.Subscriber('/clicked_point', PointStamped, set_goal)
    rospy.loginfo("subscribing to /clicked_point")
    rospy.Subscriber('slam_out_pose', PoseStamped, set_current)
    rospy.loginfo("subscribing to slam_out_pose")
    rospy.spin()
    rospy.loginfo("stopping pathfinding")

if __name__ == "__main__":
    setup_pathfinding()
