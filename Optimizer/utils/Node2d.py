class Node2d:
    def __init__(self, x, y, xy_res, XYBounds):
        self.x = x
        self.y = y
        self.grid_x = (x - XYBounds[0]) // xy_res
        self.grid_y = (y - XYBounds[2]) // xy_res  
        self.index = f'{self.grid_x}_{self.grid_y}'
        self.path_cost = 0
        self.heu_cost = 0
        self.pre_node = None
        
    def setPathCost(self, path_cost):
        self.path_cost = path_cost
    
    def setHeuCost(self, heu_cost):
        self.heu_cost = heu_cost
        
    def getCost(self):
        return self.path_cost + self.heu_cost
    
    def __eq__(self, other):
        return self.index == other.index
    
    def __str__(self):
        return f'Node2d(x:{self.x}, y:{self.y}, cost:{self.getCost()})'
    
    def __repr__(self):
        return str(self)