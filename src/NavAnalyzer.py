#!/usr/bin/env python3 
import rospy
from std_srvs.srv import Empty
from ROS_SplinePathMotionProfiling.srv import GetNavCriteriaPlan, GetNavSimTest
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import handler

class NavAnalyzer: 

    def __init__(self):
        # initialize node
        rospy.init_node("nav_analyzer", anonymous=True)
        self.node_rate = rospy.Rate(10)
        
        # absolute maximum centripetal acceleration given simulation physics
        coeff_static_friction = 1.0
        centripetal_acceleration = coeff_static_friction * 9.81 # [m/sec^2]
        # percentage of centripetal acceleration to consider when specifying max centripetal acceleration for spline path driving
        scaler = 0.8
        
        # other motion profiling constraints:
        self.ACCELERATION = 0.2 # [m/sec^2]
        self.MAX_ANGULAR_SPEED = 0.6 # [radians/sec]
        self.MAX_LINEAR_SPEED = 0.75 # [m/sec]
        self.MAX_CENTRIPETAL_ACCELERATION = centripetal_acceleration * scaler # [m/sec^2]
        # initializer flags for auto-tuning pid tests; used upon initializing by evaluating PID coefficients in PIDTuner the first time by a callable error supplier on performance error
        self.init_pid_tuning = True

        self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.152344, 0.000083, 15.357032
        self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656

    def configureNavMotionProfilingCriteria(self, acceleration: float, max_lin_speed: float, max_ang_speed: float, max_centripetal_acceleration: float): 
        """
        Configures motion profiling criteria for spline path driving navigation given a fixed acceleration rate and upper bounded motion constraints. 
        param: acceleration [float] The specified fixed acceleration in [m/sec^2]
        param: max_lin_speed [float] The specified max linear speed in [m/sec]
        param: max_ang_speed [float] The specified max angular speed in [radians/sec]
        param: max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] 
        """
        rospy.loginfo("NavAnalyzer.py: Requesting motion profiling criteria config from \'/navigation/set_motion_criteria\' service")
        rospy.wait_for_service("/navigation/set_motion_criteria")
        try: 
            client = rospy.ServiceProxy("/navigation/set_motion_criteria", GetNavCriteriaPlan)
            response = client(acceleration=acceleration, max_linear_speed=max_lin_speed, max_angular_speed=max_ang_speed, max_centripetal_acceleration=max_centripetal_acceleration)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to configure motion profiling criteria; service may have failed")
                exit()
            rospy.loginfo("NavAnalyzer.py: motion profiling criteria config service ended successfully")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def requestNavSimTest(self, lin_kp: float, lin_ki: float, lin_kd: float, ang_kp: float, ang_ki: float, ang_kd: float):
        """
        Requests a navigation simulation test for spline driving to be run given specified navigation feedback PID coefficients to return the position and heading error
        by comparing the recorded positioning of the robot against the spline path being followed. 
        param: lin_kp [float] The specified proportional coefficient for linear speed feedback control
        param: lin_ki [float] The specified integral coefficient for linear speed feedback control
        param: lin_kd [float] The specified derivative coefficient for linear speed feedback control
        param: ang_kp [float] The specified proportional coefficient for angular speed feedback control
        param: ang_ki [float] The specified integral coefficient for angular speed feedback control
        param: ang_kd [float] The specified derivative coefficient for angular speed feedback control
        returns: The position error and heading error of the overall path driven compared to spline path followed
        """
        rospy.loginfo("NavAnalyzer.py: Requesting navigation simulation test from \'/navigation/sim_test\' service")
        rospy.wait_for_service("/navigation/sim_test")
        try:
            client = rospy.ServiceProxy("/navigation/sim_test", GetNavSimTest)
            response = client(linear_kp=lin_kp, linear_ki=lin_ki, linear_kd=lin_kd, angular_kp=ang_kp, angular_ki=ang_ki, angular_kd=ang_kd)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to retrieve successful test results")
                exit()
            rospy.loginfo("NavAnalyzer.py: nav sim test service ended; returning test results")
            return (response.x_errors, response.y_errors, response.heading_errors, response.lin_speed_adjs, response.ang_speed_adjs)
        except rospy.ServiceException as e: 
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())
        return None
            
    def resetRobot(self):
        """
        Requests a reset of the robot in Gazebo World Simulation.
        """
        rospy.loginfo("NavAnalyzer.py: Requesting Robot Reset from \'/gazebo/set_model_state\' service")
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            state_msg = ModelState()
            state_msg.model_name = "turtlebot3_burger"
            state_msg.reference_frame = "map"
            state_msg.pose.position.x = 0
            state_msg.pose.position.y = 0
            state_msg.pose.orientation = handler.get_orientation(0)
            response = client(state_msg)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to reset robot on service response")
                exit()
            rospy.loginfo("NavAnalyzer.py: robot reset service ended successfuly")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def logData(self, file_dir: str, data): 
        """
        Logs data to a CSV file. 
        """
        with open(file_dir, "a") as file:
            for i in range(0, len(data)):
                record, delim = "", ""
                for token in data[i]:
                    record = record + delim + token
                    delim = ", "
                file.write(record + "\n")

    def analyzePerformance(self, epochs: int):
        """
        Analyzes position and heading error and collects both per epoch as a record in the error file specified as pair of error distributions.
        param: epochs [int] The specified number of data points to collect from this number of test simulations to run
        """
        # performance error loggers
        error_log = [] # log vector elements (delta_x, delta_y, delta_angle)
        speed_adj_log = [] # log vector elements (linear_speed_adjustment, angular_speed_adjustment)
        for epoch in range(0, epochs):
            # initialize, reset, and wait
            self.resetRobot()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzer.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            # simulate and test for averaged errors
            x_errors, y_errors, heading_errors, lin_adjs, ang_adjs = self.requestNavSimTest(lin_kp=self.LIN_KP, lin_ki=self.LIN_KI, lin_kd=self.LIN_KD, ang_kp=self.ANG_KP, ang_ki=self.ANG_KI, ang_kd=self.ANG_KD)
            N = min(len(x_errors), min(len(y_errors), min(len(heading_errors), min(len(lin_adjs), len(ang_adjs)))))
            # log errors
            for i in range(0, N):
                error_log.append((x_errors[i], y_errors[i], heading_errors[i]))
                speed_adj_log.append(lin_adjs[i], ang_adjs[i])

            # output status with moving averages
            status = "NavAnalyzer.py: Performance Analysis => epoch=" + str(epoch + 1) + " of " + str(epochs) 
            rospy.loginfo(f"{handler.str_bold_start}{status}{handler.str_bold_end}")

        return (error_log, speed_adj_log)

    def run(self):
        # initialize profiling criteria
        self.configureNavMotionProfilingCriteria(self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)
        # analyze performance:   
        error_log, speed_adj_log = self.analyzePerformance(3) 
        # set error logging file
        root_dir = handler.get_ros_package_root("ROS_SplinePathMotionProfiling")
        self.logData(root_dir + "/datasets/error_data.csv", error_log)
        self.logData(root_dir + "/datasets/speed_adjustment_data.csv", speed_adj_log)
        rospy.spin()

if __name__ == "__main__":
    NavAnalyzer().run()

