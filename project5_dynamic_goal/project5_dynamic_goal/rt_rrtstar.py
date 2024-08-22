'''

This is the ROS2 code for dynamic goal - Real Time RRT Star (RT-RRT*) Algorithm

This sampling based path planning method can be used in real time scenarios where the goal position changes with respect to time on the map.

This code is written for making a turtlebot3 waffle robot catch up to the frequently changing goal

An initial goal value input is given by the user but the goal changes by + 200mm in the y direction for every 25 steps the robot takes (where 1 step is the movement between 2 connected nodes)

After every 25 steps, the updated path to the new goal will be displayed (please close the matplotlib map display window to resume the simulation)

A PID controller is implemented to take the robot to the goal. The robot might take a longer time to move to the goal but it accurately traverses the waypoints obtained from the planning algorithm.

'''

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d import euler
from rclpy.qos import qos_profile_sensor_data

class RTRRTStar(Node):

    def __init__(self):

        super().__init__('tbot_RTRRTStar')
        
        ## Declaring parameters used in the code
        self.robot_radius = 220
        self.clearance = 20
        self.offset = self.robot_radius + self.clearance
        self.map_dimension = (6000,2000)
        self.num_iterations=10000
        self.step_length = 100
        threshold_neighbour_radius = 25
        threshold_neighbours_number = 5
        self.neighbourhood_radius = self.step_length
        self.linear=0.1
        self.waypoint_threshold=10
        
        ## Declaring initial point
        self.initial_x=(0+500)
        self.initial_y=(0+1000)
        
        ## Initializing class variables
        self.current_x = 0
        self.current_y = 0
        self.current_angle = 0
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
        self.rec_cor_list = []
        self.dynamic_obs_list=[]
        self.vertex=[]
        self.next_goal_x = 0.0
        self.next_goal_y = 0.0
        self.plot_path=False
        k=0
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.next_error = 0
        self.max_next_error = 5
        self.min_next_error = -5
        self.angular=0.0
        self.index=0
        self.lock1 = False
        self.lock2 = False
        self.final_node = None
        
        ## creating publishers and subscribers
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,qos_profile=qos_profile_sensor_data) 
        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_msg = Twist()
        
        ## Updating and getting the obstacle details
        self.update_obs()
        
                ## Getting goal information from the user
        ## Recommended goal inputs: goal_x = 5000 and goal_y = -500
        
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
            
            for rectangle in self.rectangle_offset:
                (brx, bry, w, h) = rectangle
                if 0 <= self.goal_node.x - (brx) <= w and 0 <= self.goal_node.y - bry <= h:
                    print("Please enter valid goal node value")
                    skip_while=True

            (x, y, r) = self.circle_offset
            if math.hypot(self.goal_node.x- x, self.goal_node.y - y) <= r:
                skip_while=True

            if skip_while==True:
                continue
            else:    
                break
        
        ## Creating initial and final node object 
        self.initial_node = Graph_Node((self.initial_x,self.initial_y))
        self.goal_node = Graph_Node((self.goal_x,self.goal_y))

        ## Adding initial node to the vertex
        self.vertex.append(self.initial_node)

        ## Loop to generate random nodes and connect them (Initial planning loop)
        while k <= self.num_iterations:
            
            ## creating flag variables to check for node density and distribution
            too_close = False
            too_many = False
            
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
        
        ## After the random sampling is done, the path from initial/current point to the goal point is computed
        self.get_path(self.goal_node)
        
        print("Starting Tbot Simulation")
        
        ## Creating a timer callback
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def timer_callback(self):
        
        ## The 2 flag variables are used to ensure smooth linear flow of the code
        if self.lock1==False and self.lock2==False:
            
            ## The path is inverted (as initial path is from goal to initial point)
            self.solution = self.path[::-1]
            
            ## The waypoint is accessed by indexing the solution
            try:
                self.next_goal=self.solution[self.index]
            except:
                print("Solution Reached")
                self.pub_msg.angular.z = 0.0
                self.pub_msg.linear.x = 0.0
                self.move_publisher.publish(self.pub_msg)
                exit()
                
            ## The next goal is the immediate next node that the robot should reach. This is not to be confused with the final node. 
            [self.next_goal_x , self.next_goal_y] = self.next_goal
            
            ## map coordinate is converted to gazebo coordinate
            self.next_goal_x = self.next_goal_x - 500
            self.next_goal_y = self.next_goal_y - 1000
            self.next_goal_x = self.next_goal_x / 1000
            self.next_goal_y = self.next_goal_y / 1000
            
            ## Condition statement to check if the robot is close to its waypoint 
            if (np.abs(self.current_x-self.next_goal_x)*1000) < self.waypoint_threshold and (np.abs(self.current_y-self.next_goal_y)*1000) < self.waypoint_threshold:
                
                ## Index is changed to access the next waypoint
                self.index = self.index+1
                
                ## if the index value is more than 25, the goal point is changed
                
                ## This simulates a dynamic scenario where the goal changes and the robot replans its path from its current position to the new goal
                if self.index>25:
                    
                    ## The robot is stopped to allow it to replan the path
                    self.pub_msg.angular.z = 0.0
                    self.pub_msg.linear.x = 0.0
                    self.move_publisher.publish(self.pub_msg)
                    
                    ## The goal is changed
                    ## In this implementation, the goal position is moved up by 100mm after every 25 steps the robot takes.
                    self.update_goal()
                    
                    ## The current position of the robot in map frame is obtained
                    [curr_node_x,curr_node_y] = self.solution[self.index-1]
                    self.curr_node = Graph_Node((curr_node_x,curr_node_y))
                    
                    ## Index value is reinitialized to 0
                    self.index = 0
                    self.lock1 = True
                    self.lock2 = True
                    
                    ## The nodes are rewired from the current position of the robot.
                    ## This ensures that the robot calculates the path to the new goal from the current position.
                    self.rewire_around_root(self.curr_node)
                    
                    ## The updated path is obtained after rewiring
                    self.get_path(self.goal_node)

            ## The angle from the current position to the next waypoint is calculated
            self.x_diff = self.next_goal_x - self.current_x
            self.y_diff = self.next_goal_y - self.current_y
            self.desired_angle = math.atan2(self.y_diff,self.x_diff)

            ## The error in robots heading is calculated by considering the current orientation of the robot
            self.error = self.shortest_angular_difference(self.current_angle,self.desired_angle)
            
            ## The gain values for the PID controller is defined
            self.Kp = 1
            self.Ki = 0.02
            self.Kd = 0.3
            
            ## The angular velocity is calculated from the error and gain values
            self.angular = (self.Kp * self.error) + (self.Kd*(self.error-self.prev_error)) + (self.Ki*(self.next_error))
            
            ## storing the error values for I and D components of the PID controller.
            self.prev_error = self.error
            self.next_error = self.next_error + self.error
            if self.next_error > self.max_next_error:
                self.next_error = self.max_next_error
            elif self.next_error < self.min_next_error:
                self.next_error = self.min_next_error
            
            ## Unless the heading is within 0.01 radians, the robot is oriented by controlling the angular velocity    
            if np.abs(self.error)>0.01:
                self.pub_msg.angular.z = self.angular
                self.pub_msg.linear.x = 0.0
            ## After a desired heading is obtained, the robot is given linear velocity to reach the waypoint  
            else:
                self.pub_msg.angular.z = 0.0
                self.pub_msg.linear.x = self.linear
                
        # self.pub_msg.angular.z = self.angular
        # self.pub_msg.linear.x = self.linear
        
        ## The robot is stopped to allow for rewiring and generating new paths
        else:
            self.pub_msg.angular.z = 0.0
            self.pub_msg.linear.x = 0.0
        self.move_publisher.publish(self.pub_msg)

    ## Function which dynamically updates the goal values when callled
    def update_goal(self):
        current_goal_x=self.goal_node.x
        current_goal_y=self.goal_node.y
        new_goal_y=current_goal_y+200
        ## Goal nodes gets updated
        self.goal_node=Graph_Node((current_goal_x,new_goal_y))
    
    ## Odom subscription callback function 
    def odom_callback(self,sub_msg):
        
        ## The pose of the robot is stored 
        self.current_x = sub_msg.pose.pose.position.x
        self.current_y = sub_msg.pose.pose.position.y
        quat_w = sub_msg.pose.pose.orientation.w
        quat_x = sub_msg.pose.pose.orientation.x
        quat_y = sub_msg.pose.pose.orientation.y
        quat_z = sub_msg.pose.pose.orientation.z
        [_,_,self.current_angle] = euler.quat2euler([quat_w,quat_x,quat_y,quat_z])
        
    ## Function used to calculate the error based on current orientation and desired orientation
    def shortest_angular_difference(self,current_angle, desired_angle):
        diff = desired_angle - current_angle
        if diff > math.pi:
            diff = diff - 2 * math.pi
        if diff < -math.pi:
            diff = diff + 2 * math.pi
        return diff    
        
    ## Function used for plotting the tree and the path in a matplotlib canvas
    def plot(self):
        fig, ax = plt.subplots()
        ax.axis("equal")
        ax.set_xlim(0, self.map_dimension[0])
        ax.set_ylim(0, self.map_dimension[1])
        
        ax.clear()

        # Adding obstacles and offsets
        for rectangle in self.rectangle_offset:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            ax.add_patch(patches.Rectangle((bottom_left_x, bottom_left_y), width, height, edgecolor='none', facecolor='lightgrey', fill=True))  
        for rectangle in self.rectangle_boundary:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            ax.add_patch(patches.Rectangle((bottom_left_x, bottom_left_y), width, height, edgecolor='none', facecolor='red', fill=True))              
        x_center, y_center, radius = self.circle_offset
        ax.add_patch(patches.Circle((x_center, y_center), radius, edgecolor='none', facecolor='lightgrey', fill=True))
        x_center, y_center, radius = self.circle_boundary
        ax.add_patch(patches.Circle((x_center, y_center), radius, edgecolor='none', facecolor='red', fill=True))
        
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
        
        plt.show()
        
        self.lock2=False
    
    ## Function to dynamically rewire the nodes around a root node
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
                new_cost = cost[current_node] + math.hypot(current_node.x - node.x, current_node.y - node.y)
                
                ## If the new cost is less than the current cost, its parent is updated (rewired)
                if node not in cost or new_cost < cost[node]:
                    node.parent = current_node
                    cost[node] = new_cost
                    open.append(node)
        self.lock1=False
            
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

        ## The obstacles are added to a list for future use
        self.rectangle_offset=[self.rec1_offset,self.rec2_offset,self.left_offset,self.top_offset,self.right_offset,self.bottom_offset]
        self.rectangle_boundary=[self.rec1_boundary,self.rec2_boundary]        
        
        ## The corners of the rectangle obstacles are added to a list for future use
        self.rec_cor_list = []
        for rectangle in self.rectangle_offset:
                (bottom_left_x, bottom_left_y, width, height) = rectangle
                vertex_list = [[bottom_left_x , bottom_left_y ],[bottom_left_x + width , bottom_left_y],[bottom_left_x + width , bottom_left_y + height],[bottom_left_x, bottom_left_y + height]]
                self.rec_cor_list.append(vertex_list)

  
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
        for vertices in self.rec_cor_list:
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
        
def main(args=None):

    rclpy.init(args=args)

    control_RTRRTStar = RTRRTStar()
    
    rclpy.spin(control_RTRRTStar)

    control_RTRRTStar.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
