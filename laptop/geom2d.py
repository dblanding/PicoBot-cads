# 2D geometry helper functions

from math import atan2, sin, cos, pi, sqrt

def p2r(r, theta):
    """Convert polar coords (r, theta) to rectangular coords (x,y)"""
    x = r * cos(theta)
    y = r * sin(theta)
    return (x, y)

def r2p(x, y):
    """Convert rect coords (x, y) to polar (r, theta)"""
    r = sqrt(x*x + y*y)
    theta = atan2(y, x)
    return (r, theta)

def cnvrt_2pts_to_coef(pt1, pt2):
    """Return (a,b,c) coefficients of line defined by 2 (x,y) pts."""
    x1, y1 = pt1
    x2, y2 = pt2
    # equation of a line: ax + by + c = 0
    a = y2 - y1
    b = x1 - x2
    c = x2*y1-x1*y2
    return (a, b, c)

def proj_pt_on_line(line, pt):
    """Return point which is the projection of pt on line."""
    a, b, c = line
    x, y = pt
    denom = a**2 + b**2
    if not denom:
        return pt
    xp = (b**2*x - a*b*y -a*c)/denom
    yp = (a**2*y - a*b*x -b*c)/denom
    return (xp, yp)

def midpoint(p1, p2, f=.5):
    """Return point part way (f=.5 by def) between points p1 and p2."""
    return (((p2[0]-p1[0])*f)+p1[0], ((p2[1]-p1[1])*f)+p1[1])

def p2p_dist(p1, p2):
    """Return the distance between two points"""
    x, y = p1
    u, v = p2
    return sqrt((x-u)**2 + (y-v)**2)

def closer(p0, p1, p2):
    """Return closer of p1 or p2 to point p0."""
    d1 = (p1[0] - p0[0])**2 + (p1[1] - p0[1])**2
    d2 = (p2[0] - p0[0])**2 + (p2[1] - p0[1])**2
    if d1 < d2: return p1
    else: return p2

def pt_coords(pose, dist, rel_ang):
    """Representing vectors as complex numbers, calculate &
    return (x, y) coords of point detected by robot at pose,
    measured distance, rel_ang (radians)'
    """
    xr, yr, ar = pose

    # convert robot location from rect coords to polar coords
    a, alpha = r2p(xr, yr)

    b = dist  # measured dist from robot to point

    beta = ar + rel_ang

    delta = atan2(a * sin(alpha) + b * sin(beta),
                  a * cos(alpha) + b * cos(beta))
    d = sqrt((a * cos(alpha) + b * cos(beta))**2 +\
             (a * sin(alpha) + b * sin(beta))**2)

    return p2r(d, delta)

if __name__ == "__main__":
    pose = (2, 2, pi/4)
    dist = 2 * sqrt(2)
    print("point on right at coords", pt_coords(pose, dist, -pi/2))
    print("point on left at coords", pt_coords(pose, dist, pi/2))
    print("point in front at coords", pt_coords(pose, dist, 0))
