import geom2d

def find_goal_in_sector(sector):
    """
    Find a goal point in the center of an open sector w/ no obstacles
    detected. Points pt1 and pt2 have been found at distances d1 & d2
    (from robot) on each side of the open sector.
    """
    d1 = sector.get('first_dist') / 1000  # convert from mm to m
    pose1 = sector.get('first_pose')  # robot pose when pt1 detected
    pt1 = geom2d.pt_coords(pose1, d1, 0)
    d2 = sector.get('last_dist') / 1000  # convert from mm to m
    pose2 = sector.get('last_pose')  # robot pose when pt2 detected
    pt2 = geom2d.pt_coords(pose2, d2, 0)

    goal_point = geom2d.midpoint(pt1, pt2)

    # make sure it's wide enough to drive from robot location (pt0)
    # pt0 found by avg of 2 poses (it wiggles during turn-in-place)
    p0x = (pose1[0] + pose2[0]) / 2
    p0y = (pose1[1] + pose2[1]) / 2
    pt0 = (p0x, p0y)

    # line between robot location and goal_point
    line = geom2d.cnvrt_2pts_to_coef(pt0, goal_point)

    # choose the closer point to pt0
    closer_pnt = geom2d.closer(pt0, pt1, pt2)

    # and project it onto line (pt3)
    pt3 = geom2d.proj_pt_on_line(line, closer_pnt)

    # distance between closer_pnt and pt3
    half_width = geom2d.p2p_dist(closer_pnt, pt3)
    return goal_point, half_width    
