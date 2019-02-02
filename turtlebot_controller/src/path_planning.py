#!/usr/bin/env python

from rospy import get_param, ServiceProxy
from time import time
from ogmpp_communications.srv import OgmppPathPlanningSrv
from ogmpp_communications.srv import OgmppSetMapSrv
from ogmpp_communications.srv import OgmppPathPlanningSrvRequest
from ogmpp_communications.srv import OgmppPathPlanningSrvResponse
from ogmpp_communications.srv import OgmppSetMapSrvRequest

from geometry_msgs.msg import Point

# Class that implements path planning via A*
class PathPlanning:

    # Constructor
    def __init__(self):
        self.debug = get_param('debug')
        self.ogmpp_pp = ServiceProxy('/ogmpp_path_planners/plan', OgmppPathPlanningSrv)
        self.ogmpp_map = ServiceProxy('/ogmpp_path_planners/set_map', OgmppSetMapSrv)

    def setMap(self, ogm):
        # Set the map
        map_req = OgmppSetMapSrvRequest()
        map_req.map = ogm
        # Get OGM Service to set the Map!
        self.ogmpp_map(map_req)

    # Function to be called from navigation
    # Need to make it faster !!!
    # It's just too slow !!!
    def createPath(self, robot_pose, target_pose, resolution):

        # Ask for path
        resp = OgmppPathPlanningSrvResponse()
        req = OgmppPathPlanningSrvRequest()
        req.data.begin = Point()
        req.data.end = Point()

        req.data.begin.x = robot_pose[0] * resolution
        req.data.begin.y = robot_pose[1] * resolution
        req.data.end.x = target_pose[0] * resolution
        req.data.end.y = target_pose[1] * resolution
        req.method = "uniform_prm"

        # Get OGM Service to get the Path!
        start = time()
        resp = self.ogmpp_pp(req)
        if self.debug:
            print('\x1b[35;1m' + str('Path Planning in  ')
                  + str(time() - start) + str(' seconds.') + '\x1b[0m')

        path = []
        for p in resp.path.poses:
            path.append([p.pose.position.x / resolution, p.pose.position.y / resolution])
        return path

