import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d import euler
from rclpy.qos import qos_profile_sensor_data

class RRTStar(Node):

    def __init__(self):

        super().__init__('tbot_RRTStar')
        
        ## Declaring parameters used in the code
        self.robot_radius = 220
        self.clearance = 20
        self.offset = self.robot_radius + self.clearance
        self.map_dimension = (2000,2000)
        self.num_iterations=2500
        self.step_length = 100
        threshold_neighbour_radius = 50
        threshold_neighbours_number = 15
        self.neighbourhood_radius = self.step_length + 50
        self.linear=0.05
        self.waypoint_threshold=5
        self.odom_subt = self.create_subscription(Odometry,'/burger/odom',self.odom_callbackb,qos_profile=qos_profile_sensor_data) 
        
        ## Declaring initial and goal points
        self.initial_x=(0+275)
        self.initial_y=(0+1000)
        
        while True:
            skip_while = False
            self.goal_x = int(input("Enter the goal x : "))
            self.goal_y = int(input("Enter the goal y : "))
            self.goal_x=(self.goal_x+275)
            self.goal_y=(self.goal_y+1000)

            if self.goal_x>=self.map_dimension[0] or self.goal_x<0 or self.goal_y<0 or self.goal_y>=self.map_dimension[1]:
                print("Please enter valid goal node value")
                skip_while=True
        
            self.goal_node = Graph_Node((self.goal_x,self.goal_y))

            if skip_while==True:
                continue
            else:    
                break
        
        ## Initializing class variables
        self.current_x = 0
        self.current_y = 0
        self.current_angle = 0
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
        self.vertex=[]
        self.next_goal_x = 0.0
        self.next_goal_y = 0.0
        self.plot_path=False
        self.dynamic_obs_x=1
        self.dynamic_obs_y=-0.8
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
        self.index = 1
        self.lock1 = False
        self.lock2 = False
        self.final_node = None
        self.linearb=0.02
        
        ## creating publishers and subscribers
        self.odom_subw = self.create_subscription(Odometry,'/waffle/odom',self.odom_callbackw,qos_profile=qos_profile_sensor_data) 
        self.move_publisher = self.create_publisher(Twist, '/waffle/cmd_vel', 10)
        self.move_publisherb = self.create_publisher(Twist, '/burger/cmd_vel', 10)
        self.pub_msg = Twist()
        self.pub_msgb = Twist()
        self.include_dynamic = False
        
        self.update_obs()
        
        ## Creating initial and final node object 
        self.initial_node = Graph_Node((self.initial_x,self.initial_y))
        self.goal_node = Graph_Node((self.goal_x,self.goal_y))

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
            ## If the distance is more than step length, then the distance is trimmed and a new node is created at step length distance
            dx = self.random_node.x - self.closest_neighbor.x
            dy = self.random_node.y - self.closest_neighbor.y
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
                        cost = self.new_cost(self.vertex[i], self.new_node)
                        cost_list.append(cost)
                        
                    ## getting the node with results in minimum cost and making it the parent of the new node
                    minimun_cost_node_index = self.neighbor_distance_index[int(np.argmin(cost_list))]
                    self.new_node.parent = self.vertex[minimun_cost_node_index]
                    
                    ## rewiring
                    ## the cost after making the new node the parent of other neighbouring nodes is considered. 
                    ## If for a particular node, the resulting cost is less than the the previous cost, the new node is made as the parent  of this node.
                    for i in self.neighbor_distance_index:
                        neighboring_node = self.vertex[i]
                        node = neighboring_node
                        cost = 0.0
                        while node.parent:
                            cost = cost + math.hypot(node.x - node.parent.x, node.y - node.parent.y)
                            node = node.parent
                        if cost > self.new_cost(self.new_node, neighboring_node):
                            neighboring_node.parent = self.new_node
            k=k+1
        
        ## After the random sampling is done, the path from innitial/current point to the goal point is computed
        self.get_path(self.goal_node)
        
        print("Starting Tbot Simulation")
        
        self.include_dynamic = True
        
        # Creating a timer callback
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def timer_callback(self):
        
        ## The 2 flag variables are used to ensure smooth linear flow of the code
        if self.lock1==False and self.lock2==False:
            
            ## The path is inverted (as initial path is from goal to initial point)
            self.solution = self.path[::-1]
            
            ## The waypoint is accessed by indexing the solution
            self.next_goal=self.solution[self.index]
            [self.next_goal_x , self.next_goal_y] = self.next_goal
            
            ## map coordinate is converted to gazebo coordinate
            self.next_goal_x = self.next_goal_x - 275
            self.next_goal_y = self.next_goal_y - 1000
            self.next_goal_x = self.next_goal_x / 1000
            self.next_goal_y = self.next_goal_y / 1000
            
            ## Condition statement to check if the robot is close to its waypoint 
            if (np.abs(self.current_x-self.next_goal_x)*1000) < self.waypoint_threshold and (np.abs(self.current_y-self.next_goal_y)*1000) < self.waypoint_threshold:
                
                ## Index is changed to access the next waypoint
                self.index = self.index + 1
                    
                ## The robot is stopped to allow it to replan the path
                self.pub_msg.angular.z = 0.0
                self.pub_msg.linear.x = 0.0
                self.move_publisher.publish(self.pub_msg)
                self.pub_msgb.angular.z = 0.0
                self.pub_msgb.linear.x = 0.0
                self.move_publisherb.publish(self.pub_msgb)
                
                if len(self.solution)<4:
                    print("Solution Reached")
                    exit()

                ## The current position of the robot in map frame is obtained
                [curr_node_x,curr_node_y] = self.solution[self.index-1]
                self.curr_node = Graph_Node((curr_node_x,curr_node_y))
                
                ## Index value is reinitialized to 0
                self.index=1
                self.lock1=True
                self.lock2=True
                
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
            
            self.pub_msgb.angular.z = 0.0
            self.pub_msgb.linear.x = self.linearb
                
        # self.pub_msg.angular.z = self.angular
        # self.pub_msg.linear.x = self.linear
        
        ## The robot is stopped to allow for rewiring and generating new paths
        else:
            self.pub_msg.angular.z = 0.0
            self.pub_msg.linear.x = 0.0
            self.pub_msgb.angular.z = 0.0
            self.pub_msgb.linear.x = 0.0
        self.move_publisher.publish(self.pub_msg)
        self.move_publisherb.publish(self.pub_msgb)
    
    ## Odom subscription callback function 
    def odom_callbackw(self,sub_msg):
        
        ## The pose of the robot is stored 
        self.current_x = sub_msg.pose.pose.position.x
        self.current_y = sub_msg.pose.pose.position.y
        quat_w = sub_msg.pose.pose.orientation.w
        quat_x = sub_msg.pose.pose.orientation.x
        quat_y = sub_msg.pose.pose.orientation.y
        quat_z = sub_msg.pose.pose.orientation.z
        [_,_,self.current_angle] = euler.quat2euler([quat_w,quat_x,quat_y,quat_z])
        
    def odom_callbackb(self,sub_msg):
        
        ## The pose of the robot is stored 
        self.dynamic_obs_x = sub_msg.pose.pose.position.x
        self.dynamic_obs_y = sub_msg.pose.pose.position.y
        
        if self.dynamic_obs_y>-0.3:
            self.linearb=0.0
        
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
            (brx, bry, w, h) = rectangle
            ax.add_patch(patches.Rectangle((brx, bry), w, h, edgecolor='none', facecolor='lightgrey', fill=True))  
        for rectangle in self.rectangle_boundary:
            (brx, bry, w, h) = rectangle
            ax.add_patch(patches.Rectangle((brx, bry), w, h, edgecolor='none', facecolor='red', fill=True))              
        if self.include_dynamic:
            xcir, ycir, rcir = self.dynamic_obs
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
    
    ## Function to check for collisions  
    def check_collision(self,initial,final):
        
        ## The obstacle list is updated
        self.update_obs()  
        
        ## A line is considered from initial to final point
        start = [initial.x, initial.y]
        diff = [final.x - initial.x, final.y - initial.y]
        
        ## If either the initial or goal node is inside the offset or obstacle, collision is retured as true
        for rectangle in self.rectangle_offset:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            if 0 <= initial.x - (bottom_left_x) <= width and 0 <= initial.y - bottom_left_y <= height:
                return True
            if 0 <= final.x - bottom_left_x <= width and 0 <= final.y - bottom_left_y <= height:
                return True
        for rectangle in self.rectangle_boundary:
            (bottom_left_x, bottom_left_y, width, height) = rectangle
            if 0 <= initial.x - (bottom_left_x) <= width and 0 <= initial.y - bottom_left_y <= height:
                return True
            if 0 <= final.x - bottom_left_x <= width and 0 <= final.y - bottom_left_y <= height:
                return True
        
        ## If the line connecting the initial and final point, intersects with the edges of the obstacle offset, then collision is returned as true
        for vertices in self.rec_edge_list:
            vertex1, vertex2, vertex3, vertex4 = vertices
            if self.intersect_rec(initial, final, start, diff, vertex1, vertex2):
                return True
            if self.intersect_rec(initial, final, start, diff, vertex2, vertex3):
                return True
            if self.intersect_rec(initial, final, start, diff, vertex3, vertex4):
                return True
            if self.intersect_rec(initial, final, start, diff, vertex4, vertex1):
                return True
        
        ## The dynamic obstacle aspect is included only after planning the initial path  
        if self.include_dynamic==True:
            (x, y, r) = self.dynamic_obs
            if math.hypot(initial.x - x, initial.y - y) <= r:
                return True 
            if math.hypot(final.x - x, final.y - y) <= r:
                return True 
                
            (x, y, r) = self.dynamic_obs
            if self.intersect_circle(start, diff, [x, y], r):
                return True
        
        return False
    
    ## Function to update the obstacle list (this is helful if there are dynamic obstacles)
    def update_obs(self):
        ## The map obstacles are defined
        ## For a circle, the center of the circle and its radius is defined
        ## For rectangles, the coordinate of one of the corner and the relative width and height is defined
        ## For ease of processing, the offset space around the obstacles and from the walls are also defined as obstacles. 
        self.rec1_offset = [915-self.offset, 0, 60+(2*self.offset), 330+self.offset]
        self.rec2_offset = [915-self.offset, 970-self.offset, 210+(2*self.offset), 60+(2*self.offset)] 
        self.rec3_offset = [915-self.offset, 1670-self.offset, 60+(2*self.offset), 330+self.offset] 
        self.left_offset = [0,0,self.offset,2000]
        self.top_offset = [0,2000-self.offset,2000,self.offset]
        self.right_offset = [2000-self.offset,0,self.offset,2000]
        self.bottom_offset = [0,0,2000,self.offset]
        self.rec1_boundary = [915, 0, 60, 330]
        self.rec2_boundary = [915, 970, 210, 60]
        self.rec3_boundary = [915, 1670, 60, 330]
        self.dynamic_obs = [(self.dynamic_obs_x*1000)+275, (self.dynamic_obs_y*1000)+1000, 330]
        
        
        ## The obstacles are added to a list for future use
        self.rectangle_offset=[self.rec1_offset,self.rec2_offset,self.rec3_offset,self.left_offset,self.top_offset,self.right_offset,self.bottom_offset]
        self.rectangle_boundary=[self.rec1_boundary,self.rec2_boundary,self.rec3_boundary]        
        
        self.rec_edge_list = []
        
        ## The corners of the rectangle obstacles are added to a list for future use
        for rectangle in self.rectangle_offset:
                (brx, bry, w, h) = rectangle
                vertex_list = [[brx , bry ],[brx + w , bry],[brx + w , bry + h],[brx, bry + h]]
                self.rec_edge_list.append(vertex_list)
    
    ## Function to find the new cost based on the parents cost and diatance to the parent     
    def new_cost(self,node_initial, node_final):
            ## Getting x and y difference
            dx = node_final.x - node_initial.x
            dy = node_final.y - node_initial.y
            ## Calculating distance
            dist = math.hypot(dx, dy)
            node=node_initial
            ## Computing cost
            cost=0.0
            while node.parent:
                cost = cost + math.hypot(node.x - node.parent.x, node.y - node.parent.y)
                node = node.parent
            return cost + dist
    
    def intersect_rec(self,initial, final, obstacle, direction, point_a, point_b):
            
            # vector from corner1 to the starting point of the line segment
            vec_1 = [obstacle[0] - point_a[0], obstacle[1] - point_a[1]]
            
            # vector from corner1 to corner2 (diagonal)
            vec_2 = [point_b[0] - point_a[0], point_b[1] - point_a[1]]
            
            # The line vector
            vec_3 = [-direction[1], direction[0]]
            
            ## Checking if the line is parallel to the rectangle edge
            dot = np.dot(vec_2, vec_3)
            
            ## If parallel, no intersection
            if dot == 0:
                return False
            
            ## Checking for intersection
            a = np.linalg.norm(np.cross(vec_2, vec_1)) / dot
            b = np.dot(vec_1, vec_3) / dot
            if a >= 0 and b <= 1 and b >=0:
                intersection_point = (obstacle[0] + a * direction[0], obstacle[1] + a * direction[1])
                distance_to_obstacle = math.hypot(intersection_point[0] - initial.x, intersection_point[1] - initial.y)
                distance_segment = math.hypot(final.x - initial.x, final.y - initial.y)
                if distance_to_obstacle <= distance_segment:
                    return True 
            return False    
    
    def intersect_circle(self,origin, direction, center, radius):
        ## getting square of the magnitude of the direction vector.
        diff_sq  = np.dot(direction, direction)
        
        ## no intersection if it has no magnitude
        if diff_sq  == 0:
            return False
        
        a = np.dot([center[0] - origin[0], center[1] - origin[1]], direction) / diff_sq 
        
        ## Checking for intersection
        if 0 <= a <= 1:
            intersection_point = (origin[0] + a * direction[0], origin[1] + a * direction[1])
            if math.hypot(intersection_point[0] - center[0], intersection_point[1] - center[1]) <= radius:
                return True
        return False
    
## Class representing sampled nodes
class Graph_Node:
    def __init__(self, node):
        self.x = node[0]
        self.y = node[1]
        self.parent = None        
        

def main(args=None):

    rclpy.init(args=args)

    control_RRTStar = RRTStar()
    
    rclpy.spin(control_RRTStar)

    control_RRTStar.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()