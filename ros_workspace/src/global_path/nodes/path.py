import Image
import ImageDraw
import math
import sys
import numpy as np
import threading
import time
import os
from collections import namedtuple
from matplotlib import pyplot
import matplotlib.cm as cm
from utils import angle_between, intersect, simpoly


class Point(object):
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
        
    def __repr__(self):
        return 'Point(%d, %d)' % (self.x, self.y)
    
    def __add__(self, vector):
        """Return new Point created by adding vector, where vector is a 2-tuple"""
        return Point(self.x+vector[0], self.y+vector[1])
    
    def __sub__(self, vector):
        """Return new Point created by subtracting vector, where vector is a 2-tuple"""
        return Point(self.x-vector[0], self.y-vector[1])
    
    def vect(self, other):
        """Return 2-tuple vector from self to other"""
        return [other.x - self.x, other.y - self.y]
    
    def dist(self, other):
        """Calc dist between this point and another"""
        return math.sqrt(math.pow(self.x-other.x, 2) + math.pow(self.y-other.y, 2))
        
def hit_test(fence_map, point, proximity):
    """See if there is a fence point with in proximity of point"""
    y_min = max(point.y-proximity, 0)
    y_max = min(point.y+proximity, len(fence_map))
    x_min = max(point.x-proximity, 0)
    x_max = min(point.x+proximity, len(fence_map[0]))
    subset = fence_map[y_min:y_max, x_min:x_max]
    for y, row in enumerate(subset):
        for x, column in enumerate(row):
            true_x = x + x_min
            true_y = y + y_min
            if subset[y][x] and math.sqrt(math.pow(point.x - true_x, 2) + math.pow(point.y - true_y, 2)) < proximity:
                return True
                #pyplot.scatter(true_x, true_y)
    return False

def find_nearest_fence_point(fence_map, point):
    winner = None
    shortest_dist = sys.maxint
    window = 30 #TODO fallback if this fucks up
    y_min = max(point.y-window, 0)
    y_max = min(point.y+window, len(fence_map))
    x_min = max(point.x-window, 0)
    x_max = min(point.x+window, len(fence_map[0]))
    subset = fence_map[y_min:y_max, x_min:x_max]
    for y, row in enumerate(subset):
        for x, column in enumerate(row):
            true_x = x + x_min
            true_y = y + y_min
            if subset[y][x]:
                dist = math.sqrt(math.pow(point.x - true_x, 2) + math.pow(point.y - true_y, 2))
                if dist < shortest_dist:
                    winner = Point(true_x, true_y)
                    shortest_dist = dist
    return winner

def generate_candidates(point, radius, map_width, map_height):
    """Generate candidate waypoints in a circle around the current position"""
    candidates = []
    # cardinals
    candidates.extend([
        Point(point.x-radius, point.y),
        Point(point.x, point.y-radius),
        Point(point.x+radius, point.y),
        Point(point.x, point.y+radius)
        ])
    
    # 45
    offset = math.sqrt(math.pow(radius, 2) / 2)
    candidates.extend([
        Point(point.x+offset, point.y+offset),
        Point(point.x-offset, point.y+offset),
        Point(point.x+offset, point.y-offset),
        Point(point.x-offset, point.y-offset)
    ])
    
    # boundary check at end
    return [p for p in candidates if 0 < p.x < map_width and 0 < p.y < map_height]

def angle_between_clock(vec1, vec2):
    """Finds the angle between two vectors in a clockwise direction (can be > 180)"""
    angle = angle_between(vec1, vec2)
    cross = np.cross(vec1, vec2)
    if cross < 0:
        angle = (2*math.pi) - angle
    return angle

def sweep_from_fence(fence_point, robot_position, candidate_point):
    fence_vec = [robot_position.x-fence_point.x, robot_position.y-fence_point.y]
    cand_vec = [robot_position.x-candidate_point.x, robot_position.y-candidate_point.y]
    return angle_between_clock(fence_vec, cand_vec)
    
def get_next_waypoint(robot_position, fence_map):
    """Returns a tuple of the next waypoint and a boolean indicating whether it thinks it is
    alongside a fence (True), or trying to reach a fence (False) to be used for loop detection"""
    
    STEP = 10
    MIN_FENCE_OFFSET = 10
    MAX_FENCE_OFFSET = 20
    fence_map_height = len(fence_map)
    fence_map_width = len(fence_map[0])
    candidates = generate_candidates(robot_position, STEP, fence_map_width, fence_map_height)
    
    # Eliminate candidates too close or far from fence
    bounded_candidates = []
    for p in candidates:
        if hit_test(fence_map, p, MIN_FENCE_OFFSET):
            pass #pyplot.scatter(p.x, p.y, c='r')
        elif not hit_test(fence_map, p, MAX_FENCE_OFFSET):
            pass #pyplot.scatter(p.x, p.y, c='r')
        else:
            #pyplot.scatter(p.x, p.y, c='g')
            bounded_candidates.append(p)
    
    # We're nowhere near the fence - just go a consistent direction until you hit fence.
    if len(bounded_candidates) == 0:
        return (candidates[0], False)
    
    # TODO: Check for past-fence
    # TODO: Optimize for target fence distance if multiple candidates left
    
    # Select first candidate found in clockwise sweep
    fence_pt = find_nearest_fence_point(fence_map, robot_position)
    #pyplot.scatter(fence_pt.x, fence_pt.y, c='y')
    sweep = 2 * math.pi
    winner = None
    for b in bounded_candidates:
        angle = sweep_from_fence(fence_pt, robot_position, b)
        #pyplot.annotate(str(angle), (b.x, b.y))
        if angle < sweep:
            winner = b
            sweep = angle
    return (winner, True)

def loop(fence_map, start):
    LOOP_END_DIST = 10 # How close we get to start pt
    robot = start
    path = [robot]
    while True:
        waypoint, alongFence = get_next_waypoint(robot, fence_map)
        #print waypoint
        path.append(waypoint)
        robot = waypoint
        # Hacky loop detection
        if len(path) > 10 and start.dist(robot) < LOOP_END_DIST:
            path.append(start)
            break
        elif len(path) > 300: # TODO DEBUG ONLY REMOVE
            print "I think I'm infinite looping aaaaaagh break"
            break
    return path

def shrink_poly(path):
    OFFSET = 20
    new_path = []
    for index in range(1, len(path)-1):
        vec_1 = [path[index-1].x - path[index].x, path[index-1].y - path[index].y]
        vec_2 = [path[index+1].x - path[index].x, path[index+1].y - path[index].y]
        
        # TODO check duplicate vectors?
        # normalize each vector
        vec_1_norm = 1 / np.linalg.norm(vec_1)
        vec_2_norm = 1 / np.linalg.norm(vec_2)
        
        normal = [vec_1[1]*vec_1_norm - vec_2[1]*vec_2_norm,
                  -vec_1[0]*vec_1_norm + vec_2[0]*vec_2_norm]
        #normalize the normal to offset
        mag = np.linalg.norm(normal)
        if mag:
            factor = OFFSET / mag
            normal = [normal[0]*factor, normal[1]*factor]
        #pyplot.plot([path[index].x, path[index].x+normal[0]],[path[index].y,path[index].y+normal[1]], c='w')
        new_path.append(Point(path[index].x+normal[0], path[index].y+normal[1]))

    new_path.append(new_path[0]) # Complete loop
    return new_path

def remove_loops(path):
    """Prune loops from path. This is barfy and n^2 but doesn't happen often
    We try to avoid pruning the main body by always pruning the smaller loop"""
    i = 0
    delete_me = [] # TODO set?
    while i < len(path):
        for j in range(i+2, len(path)-1):
            if intersect(path[i], path[i+1], path[j], path[j+1]):
                #print "intersect between %d, %d" % (i, j)
                # TODO how to detect large loops we should keep? (barbell field case)
                if j-i > len(path) / 2:
                    delete_me.extend(range(0, i+1))
                    delete_me.extend(range(j+i, len(path)))
                else:
                    delete_me.extend(range(i+1, j+1))
        i += 1
    new_path = []
    for i, point in enumerate(path):
        if i not in delete_me:
            new_path.append(point)
    # close path
    if new_path[0] != new_path[-1]:
        new_path.append(new_path[0])
    return new_path

def explore(fence_map, start):
    AREA_CUTOFF = 3000
    area = float('inf')
    
    outer_path = loop(fence_map, start)
    pyplot.plot([p.x for p in outer_path], [p.y for p in outer_path], c='b')
    while area > AREA_CUTOFF:
        shrink = shrink_poly(outer_path)
        #pyplot.scatter([p.x for p in shrink], [p.y for p in shrink], c='w')
        #pyplot.plot([p.x for p in shrink], [p.y for p in shrink], ':')
        pruned = remove_loops(shrink)
        pyplot.plot([p.x for p in pruned], [p.y for p in pruned])
        outer_path = pruned
        area = simpoly([p.x for p in pruned], [p.y for p in pruned])
        print "area:", area
        
def main():
    image = Image.open(os.path.join('..','test_data','corridor.png')).convert("L")
    image = np.asarray(image)

    start = Point(x=166, y=229) #corridor.png
    
    pyplot.imshow(image, origin='lower', cmap = cm.Greys_r)
    pyplot.gca().autoscale(False)
    
    explore(image, start)

    pyplot.show()
    
if __name__ == '__main__':
    main()

    
    
    
