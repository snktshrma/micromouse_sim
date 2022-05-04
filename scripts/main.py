# Micromouse main script
# Sanket Sharma , Asif Khan
import rospy
from geometry_msgs.msg import Twist
# from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
import tf
from nav_msgs.msg import Odometry

MAZE_SIZE = 16  # 16x16 
ANGULAR_SPEED = 0.39
LINEAR_SPEED = 0.39
ANGLE_THRESHOLD = 0.1 # angle precision in radian
TURNING_THRESHOLD = 1.4
step = 1
START_X = 15
START_Y = 0

class controller():
    # constructor here
    def __init__(self):
        # node initialized
        rospy.init_node('controller_node',anonymous=True)
        self.msg=Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
        self.flag = 0
        # self.cells  = file.read
        # self.flood = file.read
        # self.check = file.read()
         # if self.check == 0:
        # This contains configuration of walls
        self.cells = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]

        self.flood = [[14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14],
                     [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
                     [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
                     [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
                     [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
                     [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
                     [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
                     [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
                     [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
                     [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
                     [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
                     [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
                     [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
                     [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
                     [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
                     [14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14]]

        # Required variables:
        # These variables can be changed according to the needs 
        self.final_cells = [[7,7],[7,8],[8,7],[8,8]] # final cell position
        self.direction = "north"  #initial direction
        self.angular_speed = ANGULAR_SPEED #0.14 # change this to change linear speed
        self.linear_speed = LINEAR_SPEED # change this to change angular speed
        self.orient = 0 # North -> 0 ; South -> 2 ; East -> 1 ; West -> 3 // Initial orientation
        self.xy = [START_X,START_Y] # initial position in the maze  
        self.maze_width = 2.892 # given on the portal .. https://techfest.org/2021/competitons/Micromouse.pdf
        self.angleThreshold = ANGLE_THRESHOLD
        

        
        # Other variables (Memory of the Mouse)
        self.one_cell_width = self.maze_width/MAZE_SIZE
        self.half_cell_width = self.one_cell_width/2
        self.half_maze_width = self.maze_width/2
        self.leftwall_distance = 0
        self.rightwall_distance = 0
        self.forwardwall_distance = 0
        self.odom = Odometry()
        self.coordinates = [0,0] # first x then y ( Temporary coordinates)
        self.angle =  0
        self.WallLeft = True
        self.WallRight = True
        self.WallForward = False
        self.xprev=0
        self.yprev=0

        # Specific variables for PD controller
        self.error = 0
        self.perror = 0.01
        self.p = 1.0 # last commit 7  # 2.7
        self.d = 14 # last commit 250 # 16
        self.max = 0.5
        self.kp = 1.2
        self.kd = 1

        # Publishers here
        self.velocity_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        # Subscriptions here
        rospy.Subscriber("/my_mm_robot/laser/scan",LaserScan,self.laser_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)
    
    # FUNCTIONS::
    #-----------------------------------------------------  
    def laser_callback(self,msg):
        #self.laser = msg
        self.leftwall_distance = msg.ranges[359]
        self.rightwall_distance = msg.ranges[0]
        self.forwardwall_distance =msg.ranges[180]
        if(self.leftwall_distance<=0.14):
            self.WallLeft = True
        else:
            self.WallLeft = False
        if(self.rightwall_distance<=0.14):
            self.WallRight = True
        else:
            self.WallRight = False
        if(self.forwardwall_distance<=0.14):
            self.WallForward = True
        else:
            self.WallForward = False

    
    #-----------------------------------------------------
    def odom_callback(self,msg):
        self.odom = msg
        
        # These are temporary coordinates ( not represent real one )
        self.coordinates[0] = int((self.odom.pose.pose.position.x + self.half_maze_width + self.half_cell_width )/(self.one_cell_width)) - 1 # orientation 0.08 0.005
        self.coordinates[1] = (int((-self.odom.pose.pose.position.y + self.half_maze_width  +self.half_cell_width)/(self.one_cell_width)) - 1)
        #print("real x is ",self.coordinates[0]),
        #print("real y is ",self.coordinates[1])
        if self.orient == 0:
            self.error = self.odom.pose.pose.position.x + ((self.xy[0]-7)*(self.one_cell_width) - self.half_cell_width)
            #print("error is",self.error)
        elif self.orient == 2:
            self.error = -(self.odom.pose.pose.position.x + ((self.xy[0]-7)*(self.one_cell_width) - self.half_cell_width))
            #print("error is",self.error)
        elif self.orient == 3:
            self.error = self.odom.pose.pose.position.y - ((8-self.xy[1])*(self.one_cell_width) - self.half_cell_width)
            #print("error is",self.error)
        elif self.orient == 1:
            self.error = -(self.odom.pose.pose.position.y - ((8-self.xy[1])*(self.one_cell_width) - self.half_cell_width))
            #print("error is",self.error)
    
        if(self.coordinates[0] == -1):
            self.coordinates[0] = 0
        if(self.coordinates[1] == -1):
            self.coordinates[1] = 0
        #print("x="+str(self.coordinates[0] )),
        #print("y="+str(self.coordinates[1] ))
        #print("\n")
        (x,y,self.angle) = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                                                     self.odom.pose.pose.orientation.y,
                                                                     self.odom.pose.pose.orientation.z,
                                                                     self.odom.pose.pose.orientation.w])
        # pass
        #print(self.angle)
    
    #-----------------------------------------------------
    def GetDirection(self): 
        current_angle = self.angle
        if(abs(1.57-current_angle)<ANGLE_THRESHOLD):
            return str("west")
        elif(abs(0-current_angle)<ANGLE_THRESHOLD):
            return str("north")
        elif(abs(-1.57-current_angle)<ANGLE_THRESHOLD):
            return str("east")
        elif(abs(3.14-current_angle)<ANGLE_THRESHOLD):
            return str("south")
        elif(abs(-3.14-current_angle)<ANGLE_THRESHOLD):
            return str("south")
        else:
            return str("inbetween")
    
    #-----------------------------------------------------
    def orientation(self,orient,turning):
        if (turning== 'L'):
            orient-=1
            if (orient==-1):
                orient=3
        elif(turning== 'R'):
            orient+=1
            if (orient==4):
                orient=0
        elif(turning== 'B'):
            if (orient==0):
                orient=2
            elif (orient==1):
                orient=3
            elif (orient==2):
                orient=0
            elif (orient==3):
                orient=1
        
        return(orient)
    
    #-----------------------------------------------------
    # This is actually a simple PD controller. 
    def PID(self,error):
        if(self.orient == 0):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = 0-self.angle
            angle_error = angle_having - angle_setpoint
            #print(angle_error)
            diff = angle_error - self.perror
            self.msg.angular.z = self.kp*angle_error - self.kd*diff
            self.perror = angle_error
        
        elif(self.orient == 2):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            if self.angle < 3.14159 and self.angle > 0:
                angle_having = 3.14159-self.angle
            else:
                angle_having = -3.14159-self.angle
            angle_error = angle_having - angle_setpoint
            diff = angle_error - self.perror
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error - self.kd*diff
            self.perror = angle_error

        elif(self.orient == 1):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = -1.57-self.angle
            angle_error = angle_having - angle_setpoint
            diff = angle_error - self.perror
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error - self.kd*diff
            self.perror = angle_error

        else:
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = 1.57-self.angle
            angle_error = angle_having - angle_setpoint
            diff = angle_error - self.perror
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error - self.kd*diff
            self.perror = angle_error
        
    
    #-----------------------------------------------------
    # A single PD controller for Turning (don't change tuned values)
    def TurnPID(self):
        self.msg.angular.x = 0
        orient = self.orient
        if orient == 0:
            now = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec()-now < TURNING_THRESHOLD):
                angle_setpoint = 0
                #print(angle_setpoint)
                angle_having = 0-self.angle
                angle_error = angle_having - angle_setpoint
                #print(angle_error)
                diff = angle_error - self.perror
                self.msg.angular.z = self.kp*4*angle_error - self.kd*4*diff
                self.perror = angle_error  
                self.velocity_pub.publish(self.msg) 
        elif orient == 2:
            now = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec()-now < TURNING_THRESHOLD):
                angle_setpoint = 0
                #print(angle_setpoint)
                if self.angle < 3.14159 and self.angle > 0:
                    angle_having = 3.14159-self.angle
                else:
                    angle_having = -3.14159-self.angle
                angle_error = angle_having - angle_setpoint
                #print(angle_error)
                diff = angle_error - self.perror
                self.msg.angular.z = self.kp*4*angle_error - self.kd*4*diff
                self.perror = angle_error 
                self.velocity_pub.publish(self.msg)
        elif orient == 1:
            now = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec()-now < TURNING_THRESHOLD):
                angle_setpoint = 0
                #print(angle_setpoint)
                angle_having = -1.57-self.angle
                angle_error = angle_having - angle_setpoint
                #print(angle_error)
                diff = angle_error - self.perror
                self.msg.angular.z = self.kp*4*angle_error - self.kd*4*diff
                self.perror = angle_error  
                self.velocity_pub.publish(self.msg)
        else :
            now = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec()-now < TURNING_THRESHOLD):
                angle_setpoint = 0
                #print(angle_setpoint)
                angle_having = 1.57-self.angle
                angle_error = angle_having - angle_setpoint
                #print(angle_error)
                diff = angle_error - self.perror
                self.msg.angular.z = self.kp*4*angle_error - self.kd*4*diff
                self.perror = angle_error  
                self.velocity_pub.publish(self.msg)

        self.msg.angular.z = 0
        self.msg.angular.x = 0
        self.velocity_pub.publish(self.msg)

 
    #-----------------------------------------------------
    def GoLeft(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        current_direction = self.GetDirection()
        self.now = rospy.Time.now().to_sec() 
        while(rospy.Time.now().to_sec()-self.now < 0.7  ): 
            self.msg.angular.z = ANGULAR_SPEED
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            #print(current_direction)
        current_direction = self.GetDirection()
        while(current_direction=="inbetween"  ):
            self.msg.angular.z = ANGULAR_SPEED
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  
        
        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

    
    #-----------------------------------------------------
    def GoForward(self):
        if self.orient == 0:
            self.error = self.odom.pose.pose.position.x + ((self.xy[0]-7)*(self.one_cell_width) - self.half_cell_width)
            #print("error is",self.error)
        elif self.orient == 2:
            self.error = -(self.odom.pose.pose.position.x + ((self.xy[0]-7)*(self.one_cell_width) - self.half_cell_width))
            #print("error is",self.error)
        elif self.orient == 3:
            self.error = self.odom.pose.pose.position.y - ((8-self.xy[1])*(self.one_cell_width) - self.half_cell_width)
            #print("error is",self.error)
        elif self.orient == 1:
            self.error = -(self.odom.pose.pose.position.y - ((8-self.xy[1])*(self.one_cell_width) - self.half_cell_width))
            #print("error is",self.error)
        self.msg.linear.x = LINEAR_SPEED
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0 
        self.msg.angular.z = 0
        skip = 400 # 500 was working # 250 not
        if(self.orient == 0 or self.orient == 2):
            i = 0
            self.now = rospy.Time.now().to_sec()
            while(abs(self.now-rospy.Time.now().to_sec())<0.025):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
            current_y = self.coordinates[1]
            i = 0
            while(self.coordinates[1]==current_y):
                self.PID(self.error)
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
        else:
            i = 0
            self.now = rospy.Time.now().to_sec() 
            while(abs(self.now-rospy.Time.now().to_sec())<0.025):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
            current_x = self.coordinates[0]
            i=0
            while(self.coordinates[0]==current_x):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1

        # Stopping
        self.perror = 0
        self.error = 0
        self.msg.angular.z = 0
        #self.msg.linear.x = -40
        #self.velocity_pub.publish(self.msg)
        self.msg.linear.x = 0
        self.velocity_pub.publish(self.msg)

    
    #-----------------------------------------------------
    def GoRight(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        current_direction = self.GetDirection()
        self.now = rospy.Time.now().to_sec() 
        #self.angular_speed

        while(rospy.Time.now().to_sec()-self.now < 0.7 ): # current_direction!="inbetween"
            self.msg.angular.z = -ANGULAR_SPEED
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            #print(current_direction)
        current_direction = self.GetDirection()
        while(current_direction=="inbetween"  ):
            self.msg.angular.z = -ANGULAR_SPEED
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)


    #-----------------------------------------------------
    def isReachable(self,x,y,x1,y1):
        if (x==x1):
            if(y>y1):
                if(self.cells[y][x]==4 or self.cells[y][x]==12 or self.cells[y][x]==6 or self.cells[y][x]==5 or self.cells[y][x]==14 or self.cells[y][x]==7 or self.cells[y][x]==13 ):
                    return (False)
                else:
                    return(True)
            else:
                if(self.cells[y][x]==1 or self.cells[y][x]==3 or self.cells[y][x]==9 or self.cells[y][x]==5 or self.cells[y][x]==7 or self.cells[y][x]==11 or self.cells[y][x]==13 ):
                    return (False)
                else:
                    return(True)
                
        elif (y==y1):
            if(x>x1):
                if(self.cells[y][x]==8 or self.cells[y][x]==12 or self.cells[y][x]==9 or self.cells[y][x]==10 or self.cells[y][x]==14 or self.cells[y][x]==11 or self.cells[y][x]==13 ):
                    return (False)
                else:
                    return (True)
            else:
                if(self.cells[y][x]==2 or self.cells[y][x]==6 or self.cells[y][x]==3 or self.cells[y][x]==10 or self.cells[y][x]==14 or self.cells[y][x]==7 or self.cells[y][x]==11 ):
                    return (False)
                else:
                    return (True) 

    #-----------------------------------------------------
    def neighborCoordinates(self,x,y):
        x3= x-1
        y3=y
        x0=x
        y0=y+1
        x1=x+1
        y1=y
        x2=x
        y2=y-1
        if(x1>=16):
            x1=-1
        if(y0>=16):
            y0=-1
        return (x0,y0,x1,y1,x2,y2,x3,y3) 

    #----------------------------------------------------- 
    def accoradantOrNot(self,x,y):
        x0,y0,x1,y1,x2,y2,x3,y3 = self.neighborCoordinates(x,y)
        val= self.flood[y][x]
        minimums=[-1,-1,-1,-1]

        if (x0>=0 and y0>=0):
            if (self.isReachable(x,y,x0,y0)):
                minimums[0]=self.flood[y0][x0]
        if (x1>=0 and y1>=0):
            if (self.isReachable(x,y,x1,y1)):
                minimums[1]=self.flood[y1][x1]
        if (x2>=0 and y2>=0):
            if (self.isReachable(x,y,x2,y2)):
                minimums[2]=self.flood[y2][x2]
        if (x3>=0 and y3>=0):
            if (self.isReachable(x,y,x3,y3)):
                minimums[3]=self.flood[y3][x3]

        minCount=0
        for i in range(4):
            if minimums[i]== -1:
                pass
            elif minimums[i]== val+1 :
                pass
            elif minimums[i]== val-1 :
                minCount+=1
                pass

        #minVal= min(minimums)

        #return(minVal)
        
        if (minCount>0):
            return (True)
        else:
            return (False)

    #-----------------------------------------------------
    def fixCell(self,x,y):
        x0,y0,x1,y1,x2,y2,x3,y3 = self.neighborCoordinates(x,y)

        val= self.flood[y][x]
        minimums=[-1,-1,-1,-1]
        if (x0>=0 and y0>=0):
            if (self.isReachable(x,y,x0,y0)):
                minimums[0]=self.flood[y0][x0]

        if (x1>=0 and y1>=0):
            if (self.isReachable(x,y,x1,y1)):
                minimums[1]=self.flood[y1][x1]

        if (x2>=0 and y2>=0):
            if (self.isReachable(x,y,x2,y2)):
                minimums[2]=self.flood[y2][x2]

        if (x3>=0 and y3>=0):
            if (self.isReachable(x,y,x3,y3)):
                minimums[3]=self.flood[y3][x3]

        for i in range(4):
            if minimums[i]== -1:
                minimums[i]= 1000

        minVal= min(minimums)
        # Danger !!!!!
        self.flood[y][x]= minVal+1   

    #-----------------------------------------------------
    def isCentre(self,x,y):
        if((x==7 and y==8) or(x==7 and y == 7) or (x==8 and y==7) or(x==8 and y==8) ):
            return True
        else:
            return False

    #-----------------------------------------------------
    def floodFill(self,x,y,xprev,yprev):
        # flood fill algorithm
        if not self.accoradantOrNot(x,y):
            self.flood[y][x]= self.flood[yprev][xprev]+1

        stack=[]
        stack.append(x)
        stack.append(y)
        x0,y0,x1,y1,x2,y2,x3,y3= self.neighborCoordinates(x,y)
        if(x0>=0 and y0>=0):
            if (self.isReachable(x,y,x0,y0)):
                stack.append(x0)
                stack.append(y0)
        if(x1>=0 and y1>=0):
            if (self.isReachable(x,y,x1,y1)):
                stack.append(x1)
                stack.append(y1)
        if(x2>=0 and y2>=0):
            if (self.isReachable(x,y,x2,y2)):
                stack.append(x2)
                stack.append(y2)
        if(x3>=0 and y3>=0):
            if (self.isReachable(x,y,x3,y3)):
                stack.append(x3)
                stack.append(y3)

        while (len(stack)!= 0):
            yrun= stack.pop()
            xrun= stack.pop()

            if self.accoradantOrNot(xrun,yrun):
                pass
            else:
                self.fixCell(xrun,yrun)
                stack.append(xrun)
                stack.append(yrun)
                x0,y0,x1,y1,x2,y2,x3,y3= self.neighborCoordinates(xrun,yrun)
                if(x0>=0 and y0>=0):
                    if (self.isReachable(xrun,yrun,x0,y0)):
                        stack.append(x0)
                        stack.append(y0)
                if(x1>=0 and y1>=0):
                    if (self.isReachable(xrun,yrun,x1,y1)):
                        stack.append(x1)
                        stack.append(y1)
                if(x2>=0 and y2>=0):
                    if (self.isReachable(xrun,yrun,x2,y2)):
                        stack.append(x2)
                        stack.append(y2)
                if(x3>=0 and y3>=0):
                    if (self.isReachable(xrun,yrun,x3,y3)):
                        stack.append(x3)
                        stack.append(y3)
            #break
    def neighborCells(self,x,y,orient):
        if orient == 0:
            forward = [x , y + 1]
            backward = [x , y - 1]
            leftward = [x - 1, y]
            rightward = [x + 1, y]
        elif orient == 2:
            forward = [x , y - 1]
            backward = [x , y + 1]
            leftward = [x + 1, y]
            rightward = [x - 1, y]      
        elif orient == 1:
            forward = [x + 1 , y ]
            backward = [x - 1 , y]
            leftward = [x , y + 1]
            rightward = [x, y -1]
        else:
            forward = [x - 1 , y ]
            backward = [x + 1 , y]
            leftward = [x , y - 1]
            rightward = [x, y + 1]

        if forward[0] > 15:
            forward[0] = -1
        if forward[1] > 15:
            forward[1] = -1
        if rightward[0] > 15:
            rightward[0] = -1
        if rightward[1] > 15:
            rightward[1] = -1  
        if backward[0] > 15:
            backward[0] = -1
        if backward[1] > 15:
            backward[1] = -1 
        if leftward[0] > 15:
            leftward[0] = -1
        if leftward[1] > 15:
            leftward[1] = -1     
        return (forward, rightward, backward, leftward)
    

    #----------------------------------------------------
    def updateCellArraySimple(self,x,y,orient,L,F,R):
        if(L and R and F):
            if (orient==0):
                self.cells[y][x]= 11|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 7|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 14|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 13|self.cells[y][x]

        elif (L and R and not F):
            if (orient==0 or orient== 2):
                self.cells[y][x]= 10|self.cells[y][x]
            elif (orient==1 or orient==3):
                self.cells[y][x]= 5|self.cells[y][x]

        elif (L and F and not R):
            if (orient==0):
                self.cells[y][x]= 9|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 3|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 6|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 12|self.cells[y][x]

        elif (R and F and not L):
            if (orient==0):
                self.cells[y][x]= 3|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 6|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 12|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 9|self.cells[y][x]

        elif(F):
            if (orient==0):
                self.cells[y][x]= 1|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 2|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 4|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 8|self.cells[y][x]

        elif(L):
            if (orient==0):
                self.cells[y][x]= 8|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 1|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 2|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 4|self.cells[y][x]

        elif(R):
            if (orient==0):
                self.cells[y][x]= 2|self.cells[y][x]
            elif (orient==1):
                self.cells[y][x]= 4|self.cells[y][x]
            elif (orient==2):
                self.cells[y][x]= 8|self.cells[y][x]
            elif (orient==3):
                self.cells[y][x]= 1|self.cells[y][x]

        else:
            self.cells[y][x]= 0|self.cells[y][x]

    #-----------------------------------------------------
    def updateCellArray(self,x,y,orient,L,F,R):
        if(L and R and F):
            if (orient==0):
                self.cells[y][x]= 11
            elif (orient==1):
                self.cells[y][x]= 7
            elif (orient==2):
                self.cells[y][x]= 14
            elif (orient==3):
                self.cells[y][x]= 13

        elif (L and R and not F):
            if (orient==0 or orient== 2):
                self.cells[y][x]= 10
            elif (orient==1 or orient==3):
                self.cells[y][x]= 5

        elif (L and F and not R):
            if (orient==0):
                self.cells[y][x]= 9
            elif (orient==1):
                self.cells[y][x]= 3
            elif (orient==2):
                self.cells[y][x]= 6
            elif (orient==3):
                self.cells[y][x]= 12

        elif (R and F and not L):
            if (orient==0):
                self.cells[y][x]= 3
            elif (orient==1):
                self.cells[y][x]= 6
            elif (orient==2):
                self.cells[y][x]= 12
            elif (orient==3):
                self.cells[y][x]= 9

        elif(F):
            if (orient==0):
                self.cells[y][x]= 1
            elif (orient==1):
                self.cells[y][x]= 2
            elif (orient==2):
                self.cells[y][x]= 4
            elif (orient==3):
                self.cells[y][x]= 8

        elif(L):
            if (orient==0):
                self.cells[y][x]= 8
            elif (orient==1):
                self.cells[y][x]= 1
            elif (orient==2):
                self.cells[y][x]= 2
            elif (orient==3):
                self.cells[y][x]= 4

        elif(R):
            if (orient==0):
                self.cells[y][x]= 2
            elif (orient==1):
                self.cells[y][x]= 4
            elif (orient==2):
                self.cells[y][x]= 8
            elif (orient==3):
                self.cells[y][x]= 1

        else:
            self.cells[y][x]= 0

        forward, rightward, backward, leftward = self.neighborCells(x,y,orient)
        if rightward[0] >=0 and rightward[1]>=0:
            x = rightward[0]
            y = rightward[1]
            if R:
                self.updateCellArraySimple(x,y,orient,True,False,False)

        if leftward[0] >=0 and leftward[1]>=0:
            x = leftward[0]
            y = leftward[1]
            if L:
                self.updateCellArraySimple(x,y,orient,False,False,True)


        if orient == 0:
            if forward[0] >=0 and forward[1]>=0:
                x = forward[0]
                y = forward[1]
                if F:
                    self.cells[y][x] = self.cells[y][x]|4
  
        elif orient == 2:
            if forward[0] >=0 and forward[1]>=0:
                x = forward[0]
                y = forward[1]
                if F:
                    self.cells[y][x] = 1|self.cells[y][x]

        elif orient == 1:
            if forward[0] >=0 and forward[1]>=0:
                x = forward[0]
                y = forward[1]
                if F:
                    self.cells[y][x] = 8|self.cells[y][x]

        elif orient == 3:
            if forward[0] >=0 and forward[1]>=0:
                x = forward[0]
                y = forward[1]
                if F:
                    self.cells[y][x] = 2|self.cells[y][x]


                        
                
                
                
        
        '''if self.leftwall_distance > 0.16 and self.leftwall_distance < 0.96:
            temp = int(self.leftwall_distance/0.18075)
            if (orient==0):
                self.cells[y][x-temp] = 1
            elif (orient==2):
                self.cells[y][x+temp] = 3
            elif (orient==1):
                self.cells[y+temp][x] = 2
            else:
                self.cells[y-temp][x] = 4
        if self.rightwall_distance > 0.16 and self.rightwall_distance < 0.96:
            temp = int(self.rightwall_distance/0.18075)
            if (orient==0):
                self.cells[y][x+temp] = 3
            elif (orient==2):
                self.cells[y][x-temp] = 1
            elif (orient==1):
                self.cells[y-temp][x] = 4
            else:
                self.cells[y+temp][x] = 2
        if self.forwardwall_distance > 0.16 and self.forwardwall_distance < 0.96:
            temp = int(self.forwardwall_distance/0.18075)
            if (orient==0):
                self.cells[y+temp][x] = 2
            elif (orient==2):
                self.cells[y-temp][x] = 4
            elif (orient==1):
                self.cells[y][x+temp] = 3
            else:
                self.cells[y][x-temp] = 1'''
        


    #---------------------------------------------------------------
    
    '''def where_to_go(self):
        global maze_width
        cost_f = 0
        cost_l = 0
        cost_r = 0
        cost_b = 0
        if(self.orient==0):
            if(self.xy[1]==maze_width-1):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]][self.xy[1]+1]
            
            if(self.xy[1]==0):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]][self.xy[1]-1]

            if(self.xy[0]==0):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]-1][self.xy[1]]

            if(self.xy[0]==maze_width-1):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]+1][self.xy[1]]
        #-----------------------------------------------------------
        if(self.orient==2):
            if(self.xy[1]==0):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]][self.xy[1]-1]
            
            if(self.xy[1]==maze_width-1):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]][self.xy[1]+1]
                
            if(self.xy[0]==maze_width-1):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]+1][self.xy[1]]

            if(self.xy[0]==0):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]-1][self.xy[1]]
        #----------------------------------------------------------------------
        if(self.orient==3):
            if(self.xy[0]==maze_width-1):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]+1][self.xy[1]]
            
            if(self.xy[0]==0):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]-1][self.xy[1]]

            if(self.xy[1]==maze_width-1):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]][self.xy[1]+1]

            if(self.xy[1]==0):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]][self.xy[1]-1]
        #---------------------------------------------------------------------
        if(self.orient==1):
            if(self.xy[0]==0):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]-1][self.xy[1]]
            
            if(self.xy[0]==maze_width-1):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]+1][self.xy[1]]

            if(self.xy[1]==0):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]][self.xy[1]-1]

            if(self.xy[1]==maze_width-1):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]][self.xy[1]+1]
        #-------------------------------------------------------------------------
        if(self.WallForward):
            cost_f = 1001
        if(self.WallLeft):
            cost_l = 1001
        if(self.WallRight):
            cost_r = 1001 
        if(cost_f<=cost_r and cost_f<=cost_l and cost_f<=cost_b):
            return "F"
        elif(cost_l<=cost_r and cost_l<=cost_f and cost_l<=cost_b):
            return "L"
        elif(cost_r<=cost_f and cost_r<=cost_l and cost_r<=cost_b):
            return "R"
        elif(cost_b<=cost_f and cost_b<=cost_l and cost_b<=cost_r):
            return "B"
        else:
            return "F"'''

    #------------------------------------------------------------
    def isDestination(self,x,y):
        if [x,y] in self.final_cells:
            return True
        else:
            return False


    #------------------------------------------------------------
    def where_to_go(self,x,y,xprev,yprev,orient):

        x0,y0,x1,y1,x2,y2,x3,y3 = self.neighborCoordinates(x,y)
        
        prev=0
        minimums=[1000,1000,1000,1000]

        if (self.isReachable(x,y,x0,y0)):
            if (x0==xprev and y0==yprev):
                prev=0
            minimums[0]= self.flood[y0][x0]
            if self.isDestination(x0,y0):
                minimums[0] = 0

        if (self.isReachable(x,y,x1,y1)):
            if (x1==xprev and y1==yprev):
                prev=1
            minimums[1]= self.flood[y1][x1]
            if self.isDestination(x1,y1):
                minimums[1]= 0

        if (self.isReachable(x,y,x2,y2)):
            if (x2==xprev and y2==yprev):
                prev=2
            minimums[2]= self.flood[y2][x2]
            if self.isDestination(x2,y2):
                minimums[2]= 0

        if (self.isReachable(x,y,x3,y3)):
            if (x3==xprev and y3==yprev):
                prev=3
            minimums[3]= self.flood[y3][x3]
            if self.isDestination(x3,y3):
                minimums[3]= 0

        minVal=minimums[0]
        minCell=0
        noMovements=0
        for i in minimums:
            if (i!=1000):
                noMovements+=1

        for i in range(4):
            if (minimums[i]<minVal):
                if (noMovements==1):
                    minVal= minimums[i]
                    minCell= i
                else:
                    if(i==prev):
                        pass
                    else:
                        minVal= minimums[i]
                        minCell= i

        if (minCell==orient):
            return ('F')
        elif((minCell==orient-1) or (minCell== orient+3)):
            return('L')
        elif ((minCell==orient+1) or (minCell== orient-3)):
            return('R')
        else:
            return('B')

    #------------------------------------------------------------------  
    def updatePos(self):
        if (self.orient==0):
            self.xy[1]+=1
        elif (self.orient==1):
            self.xy[0]+=1
        elif (self.orient==2):
            self.xy[1]-=1
        elif (self.orient==3):
            self.xy[0]-=1
    
    #----------------------------------------------------------------
    def showFlood(self):
        for i in range(MAZE_SIZE):
            for j in range(MAZE_SIZE):
                j = 15 - j
                if self.xy[0] == j and self.xy[1] == i:
                    if self.flood[i][j] < 10 :
                        print("%#" + " "),
                    else:
                        print("%#" + " "),
                else:
                    if self.flood[i][j] < 10 :
                        print("0" + str(self.flood[i][j]) + " "),
                    else:
                        print(str(self.flood[i][j]) + " "),                        
            print("\n")
    #-----------------------------------------------------------------
    def showCell(self):
        for i in range(MAZE_SIZE):
            for j in range(MAZE_SIZE):
                j = 15 - j
                if self.cells[i][j] < 10 :
                    print("0" + str(self.cells[i][j]) + " "),
                else:
                    print(str(self.cells[i][j]) + " "),                        
            print("\n")

    #----------------------------------------------------------------
    def changeDestination(self,destinationx, destinationy):
        for j in range(16):
            for i in range(16):
                self.flood[i][j]=255

        queue=[]
        self.flood[destinationy][destinationx]=0

        queue.append(destinationy)
        queue.append(destinationx)

        
        while (len(queue)!=0):
            yrun=queue.pop(0)
            xrun=queue.pop(0)

            x0,y0,x1,y1,x2,y2,x3,y3= self.neighborCoordinates(xrun,yrun)
            if(x0>=0 and y0>=0 ):
                if (self.flood[y0][x0]==255):
                    self.flood[y0][x0]=self.flood[yrun][xrun]+1
                    queue.append(y0)
                    queue.append(x0)
            if(x1>=0 and y1>=0 ):
                if (self.flood[y1][x1]==255):
                    self.flood[y1][x1]=self.flood[yrun][xrun]+1
                    queue.append(y1)
                    queue.append(x1)
            if(x2>=0 and y2>=0 ):
                if (self.flood[y2][x2]==255):
                    self.flood[y2][x2]=self.flood[yrun][xrun]+1
                    queue.append(y2)
                    queue.append(x2)
            if(x3>=0 and y3>=0 ):
                if (self.flood[y3][x3]==255):
                    self.flood[y3][x3]=self.flood[yrun][xrun]+1
                    queue.append(y3)
                    queue.append(x3)
    #-----------------------------------------------------------------
    def floodFill3(self):
        queue = []
        for i in range(16):
            for j in range(16):
                self.flood[i][j]=255

        self.flood[7][7]=0
        self.flood[8][7]=0
        self.flood[7][8]=0
        self.flood[8][8]=0

        queue.append(7)
        queue.append(7)
        queue.append(8)
        queue.append(7)
        queue.append(7)
        queue.append(8)
        queue.append(8)
        queue.append(8)

        while (len(queue)!=0):
            yrun=queue.pop(0)
            xrun=queue.pop(0)

            x0,y0,x1,y1,x2,y2,x3,y3= self.neighborCoordinates(xrun,yrun)
            if(x0>=0 and y0>=0 ):
                if (self.flood[y0][x0]==255):
                    if (self.isReachable(xrun,yrun,x0,y0)):
                        self.flood[y0][x0]=self.flood[yrun][xrun]+1
                        queue.append(y0)
                        queue.append(x0)
            if(x1>=0 and y1>=0):
                if (self.flood[y1][x1]==255):
                    if (self.isReachable(xrun,yrun,x1,y1)):
                        self.flood[y1][x1]=self.flood[yrun][xrun]+1
                        queue.append(y1)
                        queue.append(x1)
            if(x2>=0 and y2>=0 ):
                if (self.flood[y2][x2]==255):
                    if (self.isReachable(xrun,yrun,x2,y2)):
                        self.flood[y2][x2]=self.flood[yrun][xrun]+1
                        queue.append(y2)
                        queue.append(x2)
            if(x3>=0 and y3>=0 ):
                if (self.flood[y3][x3]==255):
                    if (self.isReachable(xrun,yrun,x3,y3)):
                        self.flood[y3][x3]=self.flood[yrun][xrun]+1
                        queue.append(y3)
                        queue.append(x3)
    #-----------------------------------------------------------------
    def floodFill2(self):
        for j in range(16):
            for i in range(16):
                self.flood[i][j]=255

        queue=[]
        self.flood[START_Y][START_X]=0

        queue.append(START_Y)
        queue.append(START_X)

        
        while (len(queue)!=0):
            yrun=queue.pop(0)
            xrun=queue.pop(0)

            x0,y0,x1,y1,x2,y2,x3,y3= self.neighborCoordinates(xrun,yrun)
            if(x0>=0 and y0>=0 ): # and self.cells[y0][x0]!=0
                if (self.flood[y0][x0]==255):
                    if (self.isReachable(xrun,yrun,x0,y0)):
                        self.flood[y0][x0]=self.flood[yrun][xrun]+1
                        queue.append(y0)
                        queue.append(x0)
            if(x1>=0 and y1>=0 ):
                if (self.flood[y1][x1]==255):
                    if (self.isReachable(xrun,yrun,x1,y1)):
                        self.flood[y1][x1]=self.flood[yrun][xrun]+1
                        queue.append(y1)
                        queue.append(x1)
            if(x2>=0 and y2>=0):
                if (self.flood[y2][x2]==255):
                    if (self.isReachable(xrun,yrun,x2,y2)):
                        self.flood[y2][x2]=self.flood[yrun][xrun]+1
                        queue.append(y2)
                        queue.append(x2)
            if(x3>=0 and y3>=0 ):
                if (self.flood[y3][x3]==255):
                    if (self.isReachable(xrun,yrun,x3,y3)):
                        self.flood[y3][x3]=self.flood[yrun][xrun]+1
                        queue.append(y3)
                        queue.append(x3)

    #-----------------------------------------------------------------          
    def run(self):
        global step
        #print(step)
        #print(self.GetDirection())
        # Just for debugging purpose to stop the bot at first 
        '''flag  = 0
        while(flag == 0):
            self.GoLeft()
            flag = 1
        while(flag == 1):
            print(self.GetDirection())
            #print("bounded")'''
        #print(self.xy)
        #self.showCell()
        self.updateCellArray(self.xy[0],self.xy[1],self.orient,self.WallLeft,self.WallForward,self.WallRight)
        self.showFlood()
        #print(self.xy)
        
        if (not (self.xy in self.final_cells)): #not (self.xy in self.final_cells)
            if step == 1 or step == 3:
                self.floodFill3()
            elif step == 2:
                self.floodFill2()
            
            #self.floodFill(self.xy[0],self.xy[1],self.xprev,self.yprev)
        # Printing the flood array just for debugging purpose
            #self.showFlood()
        else:
            if step == 1:
                #self.changeDestination(15,0)
                self.floodFill2()
                #self.showFlood()
                self.final_cells = [[START_X,START_Y],[START_X,START_Y],[START_X,START_Y],[START_X,START_Y]]
                step = 2
                
            elif step == 2: 
                self.floodFill3()   
                self.showFlood()
                rospy.sleep(5)
                self.final_cells = [[7,7],[7,8],[8,7],[8,8]]
                step = 3

            elif step == 3:
                print("DONE!!!")
                rospy.sleep(1000) # just to stop wherever we are 


            #self.showFlood()
            #rospy.sleep(8)
            self.floodFill(self.xy[0],self.xy[1],self.xprev,self.yprev)

        where_to_go = self.where_to_go(self.xy[0],self.xy[1],self.xprev,self.yprev,self.orient)   #self.where_to_go()

        
        if(where_to_go == "L"):
            self.GoLeft()
            self.orient = self.orientation(self.orient,"L")
            self.TurnPID()
        elif(where_to_go == "R"):
            self.GoRight()
            self.orient = self.orientation(self.orient,"R")
            self.TurnPID()
        elif(where_to_go == "B"):
            self.GoLeft()
            self.GoLeft()
            self.orient = self.orientation(self.orient,"B")
            self.TurnPID()
    
        self.GoForward()
        self.xprev = self.xy[0]
        self.yprev = self.xy[1]
        
        # update position of the bot after a forward is done
        self.updatePos()
        
        # update cell array with wall configuration
        #self.updateCellArray(self.xy[0],self.xy[1],self.orient,self.WallLeft,self.WallForward,self.WallRight)  

        # if we are in the middle coordinates than stop there   
        '''if self.xy in self.final_cells:
            for i in range(MAZE_SIZE):
                for j in range(MAZE_SIZE):
                    print(str(self.flood[i][j]) + " "),
            print("\n")
            rospy.spin()'''
        

        #rospy.sleep(0.025)
        #os.system('clear')



if __name__ == '__main__':
    bot = controller()
    # wait so that time module initializes properly
    rospy.sleep(5)
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        bot.run()
        r.sleep()
