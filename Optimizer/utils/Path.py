from utils.LineSegment2d import LineSegment2d
from utils.GridSearch import GridSearch
from utils.Vec2d import Vec2d
from math import cos, sin

kMathEpsilon = 1e-6

class Path:
    def __init__(self, AStarPath):
        self.segments = []
        self.accumulated_s = []
        s = 0
        for i in range(len(AStarPath)):
            self.accumulated_s.append(s)
            if i == len(AStarPath)-1:
                heading = AStarPath[i] - AStarPath[i-1]
            else:
                self.segments.append(LineSegment2d(AStarPath[i], 
                                     AStarPath[i+1]))
                heading = AStarPath[i+1] - AStarPath[i]
                s += heading.length
                        
    def XYToSL(self, point):
        min_dist = float('inf')
        min_index = -1
        for i, seg in enumerate(self.segments):
            dist = seg.distanceToPoint(point)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        nearest_seg = self.segments[min_index]
        proj = nearest_seg.unit_direction.innerProd(point - nearest_seg.start)
        prod = nearest_seg.unit_direction.outProd(point - nearest_seg.start)
        if min_index == 0:
            s = min(proj, nearest_seg.length)
            if proj < 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0 else -1) * min_dist
        elif min_index == len(self.segments) - 1:
            s = self.accumulated_s[min_index] + max(0, proj)
            if proj > 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0 else -1) * min_dist
        else:
            s = self.accumulated_s[min_index] + max(0, min(proj, nearest_seg.length))
            lateral = (1 if prod > 0 else -1) * min_dist
        
        if abs(lateral) < kMathEpsilon:
            lateral = 0
        return s, lateral
    
    def SLToXY(self, sl_point, segment_index=None):
        s, l = sl_point
        if segment_index is None:
            segment_index = 0
            for i in range(1, len(self.accumulated_s)):
                if s <= self.accumulated_s[i]:
                    segment_index = i - 1
                    break        
        delta_s = s - self.accumulated_s[segment_index]
        curr_segment = self.segments[segment_index]
        if l != 0:
            A, B, _ = curr_segment.lineCoeffs()
            normal = Vec2d(A, B).normalize() * l
            prod = curr_segment.unit_direction.outProd(normal)
            if prod *  l < 0:
                normal = -1 * normal
        else:
            normal = Vec2d(0., 0.)
        res = Vec2d(delta_s * cos(curr_segment.heading), delta_s * sin(curr_segment.heading)) \
        + normal + curr_segment.start
        return res
    
    def generateBoundaries(self, numKnots, obstaclesVertices, L):
        delta_s = self.accumulated_s[-1] / (numKnots - 1)
        curr_s = 0
        curr_segment_index = 0
        s_l_boundaries = []
        for i in range(numKnots):
            while curr_s != self.accumulated_s[-1] and curr_s >= self.accumulated_s[curr_segment_index]:
                curr_segment_index += 1
            max_L = -L
            max_U = L
            point1 = self.SLToXY((curr_s, 0.))
            point2 = self.SLToXY((curr_s, 1))
            for obstacle in GridSearch.getObstaclesSegments(obstaclesVertices):
                for obstacle_seg in obstacle:
                    intersection_point = obstacle_seg.intersectionPoint(LineSegment2d(point1, point2))
                    if intersection_point is not None and obstacle_seg.hasIn(intersection_point):
                        outProd = self.segments[curr_segment_index-1].unit_direction.outProd(intersection_point-point1)
                        if outProd < 0:
                            max_L = max(max_L, -(intersection_point - point1).length)
                        else:
                            max_U = min(max_U, (intersection_point - point1).length)
            s_l_boundaries.append([max_L, max_U])
            curr_s += delta_s
            if abs(curr_s - self.accumulated_s[curr_segment_index]) < kMathEpsilon:
                curr_s = self.accumulated_s[curr_segment_index]
        return s_l_boundaries