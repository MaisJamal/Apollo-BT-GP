from math import atan2

from .Vec2d import Vec2d

kmathEpsilon = 1e-6

class LineSegment2d:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        
        delta = end - start
        self.length = delta.length
        self.heading = atan2(delta.y, delta.x)
        self.unit_direction = delta.normalize()
        
    def distanceToPoint(self, point, getDistantPoint=False):
        delta = point - self.start
        proj = delta.innerProd(self.unit_direction)
        res = None
        if proj < 0:
            res = delta.length
        elif proj >= self.length:
            res = (point - self.end).length
        else:
            res = abs(delta.outProd(self.unit_direction))
        if getDistantPoint:
            if proj < 0:
                return res, self.start
            elif proj > self.length:
                return res, self.end
            return res, self.start + self.unit_direction * proj
        return res
        
    def hasIn(self, point):
        vec = point - self.start
        proj = vec.innerProd(self.unit_direction)
        prod = vec.outProd(self.unit_direction)
        return abs(prod) < kmathEpsilon and (0 <= proj <= self.length or abs(proj) < kmathEpsilon)
    
    def lineCoeffs(self):
        delta_x = self.end.x - self.start.x
        delta_y = self.end.y - self.start.y
        
        if delta_x == 0:
            B = 0
            A = 1
        elif delta_y == 0:
            A = 0
            B = 1
        else:
            A = 1
            B = -delta_y / delta_x
        C = - A * self.start.x - B * self.start.y
        return A, B, C
    
    def intersectionPoint(self, other):
        A1, B1, C1 = self.lineCoeffs()
        A2, B2, C2 = other.lineCoeffs()
        
        det = A1*B2 - A2*B1
        if det == 0:
            return None
        y0 = (A2*C1-A1*C2) / det
        if A1 == 0:
            x0 = (-C2 - B2*y0) / A2
        else:
            x0 = (-C1 - B1*y0) / A1
        
        return Vec2d(x0, y0)
    
    def isIntersect(self, other):
        intersectionPoint = self.intersectionPoint(other)
        if intersectionPoint is not None and self.hasIn(intersectionPoint) and other.hasIn(intersectionPoint):
            return True
        return False
        
    def __str__(self):
        return f'LineSegment2d(start:{self.start}, end: {self.end})'
    
    def __repr__(self):
        return str(self)