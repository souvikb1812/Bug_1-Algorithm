#!/usr/bin/env python3

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt,pi
import math

 
# angle and distant difference constraints
# you can adjust the values for better performance
angle_eps = 0.03
dis_eps = 0.05

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if self.theta < 0:
            self.theta += 2*pi

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
    2: 'go back to closest point',
}
# define initial scenario
state = 0


def main():
    global pub
    global state

    # initialize ROS node
    rospy.init_node("bug_1")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(500)

    # Set the goal point 
    goal = Point()
    goal.x = 0.0#-1.2
    goal.y = 4.0#0.0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 
    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()

    count = 0
    hit_count = 0
    d=0.4   
    c=0
    m=0
    s=0
    q=0
    j=0
    p=0

    while not rospy.is_shutdown():
        # Decide what to do for the robot in each of these states:

        # the x,y distance to the goal from current position
        inc_x = goal.x - odom.x
        inc_y = goal.y - odom.y

        # the angle of the goal point wrt global frame
        angle_to_goal = atan2(inc_y, inc_x)
        if (angle_to_goal<0):
                angle_to_goal+=2*pi

        # the distance to the goal from current location
        dist_diff = sqrt(inc_x**2 + inc_y**2)

        # find the heading angle difference
        angle_diff = angle_to_goal - odom.theta

        if state == 0:
            # go to goal state. 
            '''
            Hint: 
                Here robot should go towards a the goal unless it encounters an obstacle.
                When it encounters the wall it should change the state to 
                "circumnavigate obstacle".

                It's an updated version of the "go_to_point.py"
            '''
            # TODO:

            if (angle_diff>0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=0.3 # rotating anti-clockwise

            elif (angle_diff<-0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=-0.3 # rotating clockwise

            elif(j==0):
                
                # Move towards wall if distance from wall is more than 0.3
                if((dist_diff>=0.2) and scan.region['front'] > 0.3):

                    speed.linear.x=0.11 # setting linear speed
                    speed.angular.z=0.0 # stopping rotation

                # Stop moving towards goal if distance between wall and 
                # turtlebot is less than or equal to 0.3
                elif ((dist_diff>=0.2) and scan.region['front'] <= 0.3):

                    #storing the current location as hit point
                    hit_point.x=odom.x 
                    hit_point.y=odom.y
                    state=1
            

            elif(j==2):

                if(dist_diff>=0.2):

                    speed.linear.x=0.11 # setting linear speed
                    speed.angular.z=0.0 # stopping rotation

                else:

                    speed.linear.x = 0.0 # stopping linear movement
                    speed.angular.z = 0.0 # stopping rotation
                    p=1
                    break

            if(p==1):
                break



            print ("current state: ", state_dict[state])

        elif state == 1:
            # circumnavigate obstacle. 
            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, circumnavigate the obstacle using wall following
                algorithm from previous project. While in this state, record closest point to goal where you can head towards goal.
                This state terminates when you reach the same point when you hit the obstacle.

                Finally, do not forget to change the state!

                It's an updated version of the "follow_wall.py"
            '''
            # TODO:
            # Turn left by rotating the turtlebot anti-clockwise if near wall
            if s==0:

                if ((scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] > d)or
            (scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] <= d )or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] > d ) or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] <= d )):

                    speed.linear.x = 0.01 # setting linear speed
                    speed.angular.z=0.3 # rotating anti-clockwise

                    #checking for closest point
                    if(sqrt(((odom.x-goal.x)**2)+((odom.y-goal.y)**2))<=sqrt(((closest_point.x-goal.x)**2)+((closest_point.y-goal.y)**2))):
                        closest_point.x=odom.x
                        closest_point.y=odom.y

                    #checking if turtlebot is away from the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        m+=1

                    #checking if the turtlebot is near the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2):
                        c+=1
                    #checking if the turtlebot has encountered the hit point for the second time
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2 and m>0 and c>1):
                        state=2
                        #break
                    s=1

                else:
                    s=1


            elif s==1:

                # If wall is to the right move along the wall
                if (scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] < d ):
                
                    speed.linear.x=0.11 # setting linear speed
                    speed.angular.z=0.0 # stopping rotation

                    #checking for closest point
                    if(sqrt(((odom.x-goal.x)**2)+((odom.y-goal.y)**2))<=sqrt(((closest_point.x-goal.x)**2)+((closest_point.y-goal.y)**2))):
                        closest_point.x=odom.x
                        closest_point.y=odom.y

                    #checking if turtlebot is away from the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        m+=1

                    #checking if the turtlebot is near the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2):
                        c+=1

                    #checking if the turtlebot has encountered the hit point for the second time
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2 and m>0 and c>1):
                        state=2
                        #break

                # If the wall is too far to the right then move ahead while turning right by rotating clockwise
                elif ((scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] >= d )or
                (scan.region['front'] > d and scan.region['fleft'] <= d and scan.region['fright'] >= d )):
                    
                    speed.linear.x=0.06 # setting linear speed
                    speed.angular.z=-0.4 # rotating clockwise

                    #checking for closest point
                    if(sqrt(((odom.x-goal.x)**2)+((odom.y-goal.y)**2))<=sqrt(((closest_point.x-goal.x)**2)+((closest_point.y-goal.y)**2))):
                        closest_point.x=odom.x
                        closest_point.y=odom.y

                    #checking if turtlebot is away from the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))>1):
                        m+=1

                    #checking if the turtlebot is near the hit point
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2):
                        c+=1

                    #checking if the turtlebot has encountered the hit point for the second time
                    if(sqrt(((odom.x-hit_point.x)**2)+((odom.y-hit_point.y)**2))<=0.2 and m>0 and c>1):
                        state=2
                        
                    s=0

                else:
                    s=0
            else:
                state=2

                

            print ("current state: ", state_dict[state])


        elif state == 2:
            # go back to closest point
            '''
            Hint: 
                Here robot should go back to closest point encountered in state 1. 
                Once you reach that point, change the state to "go to goal".

                It's an updated version of the "follow_wall.py"
            ''' 
            # TODO:

            d_diff=sqrt(((odom.x-closest_point.x)**2)+((odom.y-closest_point.y)**2))

            if q==0:

                if ((scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] > d)or
            (scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] <= d )or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] > d ) or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] <= d )):

                    speed.linear.x = 0.01 # setting linear speed
                    speed.angular.z=0.3 # rotating anti-clockwise
                    
                    # checking if the turtlebot is near the closes_point
                    if (sqrt(((odom.x-closest_point.x)**2)+((odom.y-closest_point.y)**2))<=0.2):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # stopping angular speed
                        j=2
                        state=0
                    q=1

                else:
                    q=1

            elif q==1:

                if (scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] < d ):
                
                    speed.linear.x=0.11 # setting linear speed
                    speed.angular.z=0.0 # stopping rotation
    
                    # checking if the turtlebot is near the closes_point
                    if (sqrt(((odom.x-closest_point.x)**2)+((odom.y-closest_point.y)**2))<=0.2):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # stopping rotation
                        j=2
                        state=0

                # If the wall is too far to the right then move ahead while turning right by rotating clockwise
                elif ((scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] >= d )or
                (scan.region['front'] > d and scan.region['fleft'] <= d and scan.region['fright'] >= d )):
                        
                    speed.linear.x=0.06 # setting linear speed
                    speed.angular.z=-0.4 # rotating clockwise
                    
                    # checking if the turtlebot is near the closes_point
                    if (sqrt(((odom.x-closest_point.x)**2)+((odom.y-closest_point.y)**2))<=0.2):
                        speed.linear.x = 0.0 # stopping linear speed
                        speed.angular.z=0.0 # stopping rotation
                        j=2
                        state=0
                    q=0

                else:
                    q=0

            print ("current state: ", state_dict[state])



        print (scan.region)
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
