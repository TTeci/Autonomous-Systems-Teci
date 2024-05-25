import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry




class Bug0(Node):
    #Initialization
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        #novo
        self.start_x=None
        self.start_y=None
        self.cmd_vel_msg = Twist()
        #dodano
        self.obstacle_threshold = 0.55  
        self.state = 'MOVE_TO_GOAL'
        self.angle_diff=None
        self.calc_y=None
        self.b=0       #ograničava m liniju na početku  
        self.a=0  
        self.c=0  

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range
        
    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range

    #GLAVNI KOD-----------------------------------------------------------------------------------------

    def bug_algorithm_callback(self):
        
        
            
        if self.goal_x is None or self.goal_y is None or self.current_x is None or self.current_y is None:
            return    
        #jednom se izvrti
        if self.a==0:
            self.start_x=self.current_x
            self.start_y=self.current_y
            self.a=1
        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)
        print("STARTX",self.start_x)
        print("STARTY",self.start_y)
        
        goal_distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)


        #self.calc_y=(self.goal_y/self.goal_x)*self.current_x    #jednadžba za m-liniju ako je početak u  (0,0)

        self.calc_y=((self.goal_y-self.start_y)/(self.goal_x-self.start_x))*(self.current_x-self.start_x)+self.start_y     #jednadžba m-linije

        self.angle_diff = self.normalize_angle(goal_angle - self.current_theta)


        #-------------------------------------------------------------------
        if goal_distance < 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            print("Goal reached!")
            self.state = 'MOVE_TO_GOAL'

            #da pamti poziciju krajnju kao početnu za dalje
            self.start_x=self.goal_x-0.01
            self.start_y=self.goal_y-0.01 

            self.b=0
            self.c=0   
            return
        #---------------------------------------------------------
        if self.state == 'MOVE_TO_GOAL':
            if (self.fl_sensor_value < self.obstacle_threshold or self.fr_sensor_value < self.obstacle_threshold) and abs(self.angle_diff) <0.1:
               
                self.state = 'FOLLOW_OBSTACLE'
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_pub.publish(self.cmd_vel_msg)
            else:
               self.move_to_goal(goal_angle)
        #-------------------------------------------------------------
        elif self.state == 'FOLLOW_OBSTACLE':

            if abs(self.calc_y-self.current_y)<0.1 and (self.b>200 or self.c==1):     #tu pogađa m_liniju   treba oko 20 sec da se odmakne od prve linije da ga ne buni
                self.state = 'MOVE_TO_GOAL'
                
            else:
                self.follow_obstacle()
        #-------------------------------------------------------
        elif self.state=='CORNER':
            self.c=1
            if abs(self.calc_y-self.current_y)<0.1:     
                self.state = 'MOVE_TO_GOAL'
                
            if self.fl_sensor_value >= 0.9:    
                self.cmd_vel_msg.linear.x = 0.45 
                self.cmd_vel_msg.angular.z = 0.9 
            if self.fl_sensor_value<0.9:
                self.cmd_vel_msg.linear.x = 0.0     
                self.cmd_vel_msg.angular.z = 0.0
                if abs(self.calc_y-self.current_y)>0.2:          
                    self.state='FOLLOW_OBSTACLE'

            self.cmd_pub.publish(self.cmd_vel_msg)
#--------------------------------------------------------------------------------------------

    def move_to_goal(self, goal_angle):
        
        
        if abs(self.angle_diff) > 0.1:
            self.cmd_vel_msg.linear.x = 0.0
            if self.angle_diff >0:
                self.cmd_vel_msg.angular.z = 0.25
            else:
                self.cmd_vel_msg.angular.z = -0.25
            
        else: 
                self.cmd_vel_msg.linear.x = 0.25       
                self.cmd_vel_msg.angular.z = 0.0
                self.b=0
        self.cmd_pub.publish(self.cmd_vel_msg)
#----------------------------------------------------------------------------------------------
    def follow_obstacle(self):
            self.b+=1
            
            if self.fl_sensor_value >self.obstacle_threshold and self.fr_sensor_value <0.8 :    
                self.cmd_vel_msg.linear.x = 0.05     
                self.cmd_vel_msg.angular.z = -0.55     

            if self.fl_sensor_value > 0.35 and self.fl_sensor_value <= 0.45:
                self.cmd_vel_msg.angular.z = 0.0 
                self.cmd_vel_msg.linear.x = 0.05    
                                            

            if self.fl_sensor_value <= 0.35:
                
                self.cmd_vel_msg.linear.x = 0.0       
                self.cmd_vel_msg.angular.z = -0.25               

            if self.fl_sensor_value >0.45 and self.fr_sensor_value >0.8:
                self.cmd_vel_msg.linear.x = 0.1         
                self.cmd_vel_msg.angular.z = 0.25
                if self.fl_sensor_value >0.8:     
                    self.state='CORNER'
                    self.cmd_vel_msg.linear.x = 0.0       
                    self.cmd_vel_msg.angular.z = 0.0                 
            self.cmd_pub.publish(self.cmd_vel_msg)
#----------------------------------------------------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
            
def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug0()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()