'''

This is the python code for dynamic goal - Real Time RRT Star (RT-RRT*) Algorithm

This sampling based path planning method can be used in real time scenarios where there are dynamic obstacles in the environment(map).

This code is written to show the dynamic RT RRT* path planning in 2D

An goal value input is given by the user. 

After every step, the updated path to the goal will be displayed (please close the matplotlib map display window to resume)

As the map size is big and many nodes are sampled, the code takes time to completly exeute. To address this issue, a smaller map can be used. 

'''

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
import math
import cv2

class RTRRTStar():

    def __init__(self):
        
        ## Declaring parameters used in the code and initializing class attributes
        self.current_x = 0
        self.current_y = 0
        self.current_angle = 0
        self.robot_radius = 220
        self.clearance = 20
        self.offset = self.robot_radius + self.clearance
        self.map_dimension = (6000,2000)
        self.num_iterations=10000
        self.step_length = 100
        self.circle_offset = []
        self.rec1_offset = []
        self.rec2_offset = [] 
        self.left_offset = []
        self.top_offset = []
        self.right_offset = []
        self.bottom_offset = []
        self.circle_boundary = []
        self.rec1_boundary = []
        self.rec2_boundary = []
        self.dynamic_rect_boundary = []
        self.rectangle_offset=[]
        self.rectangle_boundary=[]
        self.rec_edge_list = []
        self.dynamic_obs_list=[]
        self.vertex=[]
        self.obs_index=0
        self.plot_path=False
        k=0
        threshold_neighbour_radius = 25
        threshold_neighbours_number = 5
        self.neighbourhood_radius = self.step_length + 50
        
        ## A opencv video object for storing the obtained plots as a video
        fps = 10
        frame_size = (640, 480)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('plot_video.avi', fourcc, fps, frame_size)    
        
        ## Declaring initial point
        self.initial_x=(0+500)
        self.initial_y=(0+1000)

        ## Getting goal information from the user
        ## Recommended goal inputs: goal_x = 5000 and goal_y = 0
        while True:
            skip_while = False
            self.goal_x = int(input("Enter the goal x : "))
            self.goal_y = int(input("Enter the goal y : "))
            self.goal_x=(self.goal_x+500)
            self.goal_y=(self.goal_y+1000)
            
            if self.goal_x>=self.map_dimension[0] or self.goal_x<0 or self.goal_y<0 or self.goal_y>=self.map_dimension[1]:
                print("Please enter valid goal node value")
                skip_while=True
            
            self.goal_node = Graph_Node((self.goal_x,self.goal_y))

            if skip_while==True:
                continue
            else:    
                break
        
        ## Updating and getting the obstacle details
        self.update_obs()
        
        ## Creating initial node object 
        self.initial_node = Graph_Node((self.initial_x,self.initial_y))

        ## Adding initial node to the vertex
        self.vertex.append(self.initial_node)
        
        ## Loop to generate random nodes and connect them (Initial planning loop)
        while k<=self.num_iterations:
            
            ## creating flag variables to check for node density and distribution
            too_close=False
            too_many=False
            
            ## Sampling a random node
            self.random_node = Graph_Node((np.random.uniform( 0 + self.offset, self.map_dimension[0] - self.offset ), np.random.uniform( 0 + self.offset, self.map_dimension[1] - self.offset )))
            
            ## Finding the closest neighbour to the sampled random node
            self.closest_neighbor = self.vertex[int(np.argmin([math.hypot(node.x - self.random_node.x, node.y - self.random_node.y) for node in self.vertex]))]
            
            ## Finding the distance between the sampled node and the closest node. 
            dx = self.random_node.x - self.closest_neighbor.x
            dy = self.random_node.y - self.closest_neighbor.y
            
            ## Trimming the node with respect to step length if the sampled node is greater than the step length
            distance = min(self.step_length, math.hypot(dx, dy))
            angle = math.atan2(dy, dx)
            
            ## Creating a new node based on the calculated distance and angle
            self.new_node = Graph_Node((self.closest_neighbor.x + distance * math.cos(angle),self.closest_neighbor.y + distance * math.sin(angle)))
            
            ## Initially the parent node will be the closest node
            self.new_node.parent = self.closest_neighbor
            
            ## The new node is considred for optimization and rewiring
            ## If the new node is valid and there is no collision when connecting the node with the closest node, it is considred for further processing
            if self.new_node and not self.check_collision(self.closest_neighbor, self.new_node):
                self.neighbor_distance_list = []
                self.neighbor_distance_index = []
                
                ## distance between the new node and all the other nodes are calculated
                for node in self.vertex:
                    distance = math.hypot(node.x - self.new_node.x, node.y - self.new_node.y)
                    
                    ## if the node is too close to another node, it is ignored
                    if distance<=threshold_neighbour_radius:
                        too_close=True
                    
                    ## all the nodes which are close to the new node (within specified distance) are considered as its neighbours
                    if distance <= self.neighbourhood_radius and not self.check_collision(self.new_node, node):
                        self.neighbor_distance_list.append(distance)
                        ## The index of the neighbor node in the vertex list is stored
                        self.neighbor_distance_index.append(self.vertex.index(node))
               
                ## If there are many neighbours, the new node is ignored
                if len(self.neighbor_distance_index)>threshold_neighbours_number:
                    too_many=True
               
                ## Only when the new node is a certain minimum distance from its closest neighbour and it has a limited number of neighbours, it is considered and added to the list
                ## This is called Sampling Density Rejection and it reduces the time complexity of rewiring the nodes in real time rrt
                if too_close==False and too_many==False:
                    self.vertex.append(self.new_node)
                else:
                    continue
                
                ## reconnecting the new node to the nighbouring node which ensures lowest cost 
                if len(self.neighbor_distance_index)>0:
                    ## calculating the cost if the new node is connected with neighbouring nodes
                    cost_list = []
                    for i in self.neighbor_distance_index:
                        cost = self.get_new_cost(self.vertex[i], self.new_node)
                        cost_list.append(cost)
                    
                    ## getting the node with results in minimum cost and making it the parent of the new node
                    minimun_cost_node_index = self.neighbor_distance_index[int(np.argmin(cost_list))]
                    self.new_node.parent = self.vertex[minimun_cost_node_index]
                    
                    ## rewiring
                    ## the cost after making the new node the parent of other neighbouring nodes is considered. 
                    ## If for a particular node, the resulting cost is less than the the previous cost, the new node is made as the parent of this node.   
                    for i in self.neighbor_distance_index:
                        neighboring_node = self.vertex[i]
                        node = neighboring_node
                        cost = 0.0
                        while node.parent:
                            cost = cost + math.hypot(node.x - node.parent.x, node.y - node.parent.y)
                            node = node.parent
                        if cost > self.get_new_cost(self.new_node, neighboring_node):
                            neighboring_node.parent = self.new_node
            k=k+1
        
        ## Real time path updation loop. This loop runs till the goal is reached.
        while True:
            ## The path from current point to the goal point is computed
            self.get_path(self.goal_node)
            self.solution = self.path[::-1]
            
            ## Extract the root, which is the next point in the path
            [new_root_x,new_root_y] = self.solution[1]
            
            ## this is a condition statement to check if the goal is reached
            if new_root_x==self.final_node.x and new_root_y==self.final_node.y:
                break
            
            print(self.obs_index)
            
            ## increase the obs_index by 1 to simulate the dynamic obstacle
            self.obs_index=self.obs_index + 1
            
            ## The new path is generated from the root. It takes into account the motion of the dynamic obstacle
            self.rewire_around_root(Graph_Node((new_root_x,new_root_y)))
        
        self.out.release()  
    
    ## Function used for plotting the tree and the path in a matplotlib canvas
    def plot(self):
        fig, ax = plt.subplots()
        ax.axis("equal")
        ax.set_xlim(0, self.map_dimension[0])
        ax.set_ylim(0, self.map_dimension[1])
        
        ax.clear()

        ## Adding obstacles and offsets
        for rectangle in self.rectangle_offset:
            (brx, bry, w, h) = rectangle
            ax.add_patch(patches.Rectangle((brx, bry), w, h, edgecolor='none', facecolor='lightgrey', fill=True))
                
        for rectangle in self.rectangle_boundary:
            (brx, bry, w, h) = rectangle
            ax.add_patch(patches.Rectangle((brx, bry), w, h, edgecolor='none', facecolor='red', fill=True)) 
        
        for rectangle in self.dynamic_obs_list:
            (brx, bry, w, h) = rectangle
            ax.add_patch(patches.Rectangle((brx, bry), w, h, edgecolor='none', facecolor='black', fill=True)) 
                    
        xcir, ycir, rcir = self.circle_offset
        ax.add_patch(patches.Circle((xcir, ycir), rcir, edgecolor='none', facecolor='lightgrey', fill=True))

        xcir, ycir, rcir = self.circle_boundary
        ax.add_patch(patches.Circle((xcir, ycir), rcir, edgecolor='none', facecolor='red', fill=True))
        
        ## Plotiing the tree
        for node in self.vertex:
            if node.parent:
                ax.plot([node.parent.x, node.x], [node.parent.y, node.y], 'b-', linewidth=0.5)
            ax.plot(node.x, node.y, 'bo', markersize=1)
        
        ## Plotting the initial and goal points   
        ax.plot(self.initial_node.x, self.initial_node.y, "bs", linewidth=3)
        ax.plot(self.goal_node.x, self.goal_node.y, "gs", linewidth=3)
        
        ## Plotting the path
        if self.plot_path:
            for i in range(len(self.path) - 1):
                pt1 = self.path[i]
                pt2 = self.path[i + 1]
                ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'g-', linewidth=3)

        fig.canvas.draw()
        
        ## Convert the image to BGR format
        frame = np.array(fig.canvas.renderer.buffer_rgba())[:, :, :3]
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        
        ## adding the frame to the video object
        self.out.write(frame)
        plt.show()
        plt.close(fig)
    
    def rewire_around_root(self, new_root):
        
        ## The cost is stored in a dictionery for comparison
        cost = {new_root: 0}
        
        ## The open list created and the root node is added to it
        open = [new_root]
        
        ## The root node has no parent
        new_root.parent = None

        ## While the open node is not empty
        while open:
            ## the open node acts as a queue where the node added firt is popped first
            current_node = open.pop(0)
            
            ## The neighbours are identified for the considered node
            neighbors = [node for node in self.vertex if math.hypot(node.x - current_node.x, node.y - current_node.y) <= self.neighbourhood_radius]
            
            for node in neighbors:
                
                ## If the node cannot be joined with its neighbour due to collision, it is ignored
                if self.check_collision(current_node, node):
                    continue
                if (node.x == current_node.x and node.y == current_node.y):
                    continue
                
                ## New cost is computed based on adding the current cost of the parent node and the distance from the node
                get_new_cost = cost[current_node] + math.hypot(current_node.x - node.x, current_node.y - node.y)
                
                ## If the new cost is less than the current cost, its parent is updated (rewired)
                if node not in cost or get_new_cost < cost[node]:
                    node.parent = current_node
                    cost[node] = get_new_cost
                    open.append(node)
            
    ## Function to get the path to the goal      
    def get_path(self,goal): 
        
        goal_index = -1
        
        ## if the goal node is part of the vertex list, it is consired
        for i in range(len(self.vertex)):
            node = self.vertex[i]
            if node.x == goal.x and node.y == goal.y:
                goal_index=i
                break
        
        ## If the goal node is not part of the vertex list, The nearest neighbouring node to the goal node is consired as the end point
        if goal_index==-1:    
            goal_distances = [math.hypot(node.x - goal.x, node.y - goal.y) for node in self.vertex]
            goal_neighbour_indices = [i for i, distance in enumerate(goal_distances) if distance <= self.step_length]
            if goal_neighbour_indices:
                goal_index = goal_neighbour_indices[int(np.argmin([goal_distances[i] for i in goal_neighbour_indices]))]
        
        ## The path is computed by backtracking
        if goal_index!=-1:
            self.final_node = self.vertex[goal_index]
            end_node = self.vertex[goal_index]
            self.path = [[end_node.x, end_node.y]]
            while end_node.parent is not None:
                self.path.append([end_node.x, end_node.y])
                end_node = end_node.parent
            self.path.append([end_node.x, end_node.y])
            self.plot_path=True
            self.plot()
            
        ## If no node is near the goal node, no path is found
        else:
            print("No path")
            exit()  
    
    ## Function to update the obstacle list (this is helful if there are dynamic obstacles)
    def update_obs(self):
        ## The map obstacles are defined
        ## For a circle, the center of the circle and its radius is defined
        ## For rectangles, the coordinate of one of the corner and the relative width and height is defined
        ## For ease of processing, the offset space around the obstacles and from the walls are also defined as obstacles. 
        self.circle_offset = [4200, 1200, 600 + self.offset]
        self.rec1_offset = [1500-self.offset, 1000-self.offset, 250+(2*self.offset), 1000+self.offset]
        self.rec2_offset = [2500-self.offset, 0, 250+(2*self.offset), 1000+self.offset] 
        self.left_offset = [0,0,self.offset,2000]
        self.top_offset = [0,2000-self.offset,6000,self.offset]
        self.right_offset = [6000-self.offset,0,self.offset,2000]
        self.bottom_offset = [0,0,6000,self.offset]
        self.circle_boundary = [4200, 1200, 600]
        self.rec1_boundary = [1500, 1000, 250, 1000]
        self.rec2_boundary = [2500, 0, 250, 1000]
        
        ## The dynamic obstacle is defined as a rectangle and its position is aupdated after each step
        self.dynamic_rect_boundary = [1700, 500+(40*(self.obs_index%10)), 100, 100]
        
        ## The obstacles are added to a list for future use
        self.rectangle_offset=[self.rec1_offset,self.rec2_offset,self.left_offset,self.top_offset,self.right_offset,self.bottom_offset,self.dynamic_rect_boundary]
        self.rectangle_boundary=[self.rec1_boundary,self.rec2_boundary]
        self.dynamic_obs_list=[self.dynamic_rect_boundary]
        
        ## The corners of the rectangle obstacles are added to a list for future use
        self.rec_edge_list = []
        for rectangle in self.rectangle_offset:
                (brx, bry, w, h) = rectangle
                vertex_list = [[brx , bry ],[brx + w , bry],[brx + w , bry + h],[brx, bry + h]]
                self.rec_edge_list.append(vertex_list) 
        
        
    ## Function to check for collisions  
    def check_collision(self,initial,final):
        
        ## The obstacle list is updated
        self.update_obs()  
        
        ## A line is considered from initial to final point
        diff_x = final.x - initial.x 
        diff_y = final.y - initial.y
        
        ## If either the initial or goal node is inside the offset or obstacle, collision is retured as true
        for rectangle in self.rectangle_offset:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            if (initial.x - bottom_left_x) >= 0 and (initial.x - bottom_left_x) <= width:
                if (initial.y - bottom_left_y) >= 0 and (initial.y - bottom_left_y) <= height :
                    return True
            if (final.x - bottom_left_x) >= 0 and (final.x - bottom_left_x) <= width:
                if (final.y - bottom_left_y) >= 0 and (final.y - bottom_left_y) <= height :
                    return True
        for rectangle in self.rectangle_boundary:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            if (initial.x - bottom_left_x) >= 0 and (initial.x - bottom_left_x) <= width:
                if (initial.y - bottom_left_y) >= 0 and (initial.y - bottom_left_y) <= height :
                    return True
            if (final.x - bottom_left_x) >= 0 and (final.x - bottom_left_x) <= width:
                if (final.y - bottom_left_y) >= 0 and (final.y - bottom_left_y) <= height :
                    return True
        (x_center, y_center, radius) = self.circle_boundary
        if math.hypot(initial.x - x_center, initial.y - y_center) <= radius:
            return True 
        if math.hypot(final.x - x_center, final.y - y_center) <= radius:
            return True       
        (x_center, y_center, radius) = self.circle_offset
        if math.hypot(initial.x - x_center, initial.y - y_center) <= radius:
            return True 
        if math.hypot(final.x - x_center, final.y - y_center) <= radius:
            return True 
        
        ## If the line connecting the initial and final point, intersects with the edges of the obstacle offset, then collision is returned as true
        for vertices in self.rec_edge_list:
            vertex_1, vertex_2, vertex_3, vertex_4 = vertices
            if self.check_intersection_rectangle(initial, final, initial.x,initial.y, diff_x,  diff_y, vertex_1, vertex_2):
                return True
            if self.check_intersection_rectangle(initial, final, initial.x,initial.y, diff_x,  diff_y, vertex_2, vertex_3):
                return True
            if self.check_intersection_rectangle(initial, final, initial.x,initial.y, diff_x,  diff_y, vertex_3, vertex_4):
                return True
            if self.check_intersection_rectangle(initial, final, initial.x,initial.y, diff_x,  diff_y, vertex_4, vertex_1):
                return True
        if self.check_intersection_circle(initial.x,initial.y, diff_x,  diff_y, [x_center, y_center], radius):
            return True
        
        return False
    
    
    ## Function to check if the path is intersecting with rectangular obstacles
    def check_intersection_rectangle(self,initial,final, start_x, start_y, diff_x, diff_y, corner1, corner2):
            
            # vector from corner1 to the starting point of the line segment
            vec_1 = [start_x - corner1[0], start_y - corner1[1]]
            # vector from corner1 to corner2 (diagonal)
            vec_2 = [corner2[0] - corner1[0], corner2[1] - corner1[1]]
            # The line vector
            vec_3 = [-diff_y, diff_x]
            
            ## Checking if the line is parallel to the rectangle edge
            dot = np.dot(vec_2, vec_3)
            ## If parallel, no intersection
            if dot == 0:
                return False
            ## Checking for intersection
            a = np.linalg.norm(np.cross(vec_2, vec_1)) / dot
            b = np.dot(vec_1, vec_3) / dot
            if a >= 0 and b <= 1 and b >=0:
                distance_to_obstacle = math.hypot(start_x + a * diff_x - initial.x, start_y + b * diff_y - initial.y)
                distance_segment = math.hypot(final.x - initial.x, final.y - initial.y)
                if distance_to_obstacle <= distance_segment:
                    return True 
            return False       
           
    
    ## Function to check if the path is intersecting with circular obstacles
    def check_intersection_circle(self,start_x, start_y, diff_x, diff_y, center, radius):
        ## getting square of the magnitude of the direction vector.
        diff = [diff_x, diff_y]
        diff_sq = np.dot(diff, diff)

        ## no intersection if it has no magnitude
        if diff_sq == 0:
            return False
        
        ## Checking for intersection
        a = np.dot([center[0] - start_x, center[1] - start_y], diff) / diff_sq
        if 0 <= a <= 1:
            if math.hypot(start_x + a * diff_x - center[0], start_y + a * diff_y - center[1]) <= radius:
                return True
        return False
    
    
    ## Function to find the new cost based on the parents cost and distance to the parent     
    def get_new_cost(self,node_initial, node_final):
        ## Getting x and y difference
        dx = node_final.x - node_initial.x
        dy = node_final.y - node_initial.y
        ## Calculating distance
        dist = math.hypot(dx, dy)
        ## Computing cost
        node = node_initial
        cost = 0.0
        while node.parent:
            cost = cost + math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent
        return cost + dist

## Class representing sampled nodes   
class Graph_Node:
    def __init__(self, node):
        self.x = node[0]
        self.y = node[1]
        self.parent = None        
        
        
rt_RTRRTStar = RTRRTStar()