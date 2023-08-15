#!/usr/bin/env python3

#---------------Import Libraries and msgs---------------:

import rospy
import time
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion
from collections import Counter
from control_lib import UR_Controller


#---------------Initialise---------------:

print("\nPlease Wait While System Starts Up...", end="")
rospy.init_node("solution", anonymous=False)
ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
time.sleep(2)
print("System Started")


#---------------Main Code---------------:

class TrajectoryFinder():

    def __init__(self):

        self.ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
        self.ur_con = UR_Controller()
        self.home_waypoint = [-1, -1.5, 1.5, -1.5, 1.5, 1.5]
        self.distanceThreshold = 0.05
        self.orientationThreshold = 5
        self.cartesianAxes = ["X", "Y", "Z"]
        self.rotationalForces = ["Roll", "Pitch", "Yaw"]

    def calculateSearchTime(self, beginning, end, delay):
        '''
        This method is responsible for taking the timestamps for the beginning and end of the search cycle, along with the delay
        between each search (distance and roation) as arguments and finding out the interval between the beginning and end minus the delay period (4 seconds)
        params:
            beginning: beginning of search cycle
            end: end of search cylce
            delay: delay interval (4 seconds) between loops to solve each error (distance and rotation)
        returns:
            interval: formatted interval describing search cycle duration
        '''
        
        # Calculate interval
        interval = (end - beginning) - delay

        # Convert interval into time format minutes & seconds
        minutes, seconds = divmod(interval, 60)

        # Format interval into text form
        interval = "{:02.0f} minutes: {:02.0f} seconds".format(minutes,seconds)

        return interval

    def moveLAndFindError(self, axis, increment):
        '''
        This method is responsible for taking a cartesian axis (axis) and increment value (increment) and updating the position of the cartesian axis of the robot by the incrment.
        After which, the method calculates the error between the desired position and new (current) position to then return in particular the new-found distance error.
        params:
            axis: the cartesian axis in which movement occurs
            increment: the sum of directional movement which occurs
        returns:
            distanceError: the new-found distance error
        '''
        
        # Get robots current pose
        pose = self.ur_con.get_pose()

        # If statement query to find out which cartesian axis it is, and add increment to the axis cartesian value
        if axis == "X":

            pose.position.x += increment

        elif axis == "Y":

            pose.position.y += increment

        else:
            pose.position.z += increment

        # Defining the command message
        #command = self.ur_con.generate_move_j(pose, sequence = False, pose_msg = True)
        command = self.ur_con.generate_move_l(pose)

        # Publish movement message of movement with updated waypoint values
        self.ur_script.publish(command)

        time.sleep(2)

        # Get robots current pose
        pose = self.ur_con.get_pose()

        # Get errors (distance between desired positioning - distance, and rotation)
        errors = self.ur_con.check_errors(pose)

        # Assign variable for distance error
        distanceError = errors.ee_trans_error

        return distanceError

    def rotateEndEffectorFindError(self, force, deviation):
        '''
        This method is responsible for taking a rotational force, and angular deivation value and using them to call the rotate_tool function to rotate the end effector on an axis
        in correspondence with the rotational force (force) provided, by the deviation. After which, the method calculates the error between the desired position and new (current) 
        position to then return in particular the new-found rotation error.
        params:
            force: the rotational force which determines the manner of rotation (roll = , pitch = , yaw = )
            deviation: the sum of the angular deviation that occurs (degrees)
        returns:
            rotationError: the new-found rotation error
        '''
        
        # Convert angular deviation from degrees to radians
        deviation = deviation * np.pi/180

        #  If statement to query rotational force, finding out which force it is.
        if force == "Roll":

            # Call function to perform roll movement
            command = self.ur_con.rotate_tool(deviation, 0, 0)

        elif force == "Pitch":

            # Call function to perform pitch movement
            command = self.ur_con.rotate_tool(0, deviation, 0)

        else:

            # Call function to perform yaw movement
            command = self.ur_con.rotate_tool(0, 0, deviation)

        # Publish movement message of movement with updated waypoint values
        self.ur_script.publish(command)

        time.sleep(2)

        # Get robots current pose
        pose = self.ur_con.get_pose()  

        # Get errors (distance between desired positioning - distance, and rotation)
        errors = self.ur_con.check_errors(pose)

        # Assign variable for rotation error
        rotationError = errors.ee_rot_error

        return rotationError

    def findTrajectory(self):
        '''
        This method is responsible for finding the trajectory of the ball
        '''
        
        # Notify user the manipulator robot is going to move to its home position
        print("\nNow moving the robot to its home position...")

        # Write command for moving the manipulator robot to home position
        command = self.ur_con.generate_move_j(self.home_waypoint) 

        # Publish command to move manipulator robot
        self.ur_script.publish(command)

        # 15 second delay to allow manipulator robot to move to home position
        time.sleep(15)

        # Notify user the manipulator robot has reached its home position
        print("The robot has successfully reached its home position")

        # Get pose of manipulator robot
        pose = self.ur_con.get_pose()

        # Get errors (by querying segments between desired positioning and pose - distance, and rotation) 
        errors = self.ur_con.check_errors(pose)

        # Assign each error to variables for cartesian (distance) & rotaion error, and ball state (caught or not)
        distanceError, rotationError, ballCaught = (errors.ee_trans_error), errors.ee_rot_error, errors.ball_caught

        print("\nDefault errors:")
        print(" - Cartesian error:", (distanceError * 1000), "mm")
        print(" - Orientational error:", rotationError, "degrees")
        print(" - Ball caught:", ballCaught)

        numOfSteps = 0

        beginning = time.monotonic()

        # While loop to run while the ball has not been caught
        while not ballCaught:

            time.sleep(2)
            print("\nNow handling the Cartesian error:\n")

            while distanceError > self.distanceThreshold:

                # For loop for iterating through catesian axis list (cartesianAxes)
                for axis in self.cartesianAxes:
                    
                    # Assign distance error as movement increment
                    increment = distanceError

                    # Call function to update manipulator pose and retrieve new distance error
                    updatedDistanceError = self.moveLAndFindError(axis, increment)

                    # If statement to query if the new movement generated a smaller distance error than prior, if so assign result to distance error variable
                    if updatedDistanceError < distanceError:
                        
                        distanceError = updatedDistanceError

                    else:

                        # Assign inverse increment variable by multiplying the increment by -2 (this cancels out the inital move, back to its origin position, then increasing it by the increment however in the oppisite direction)
                        inverseIncrement = increment * -2

                        # Call function to update manipulator pose and retrieve new distance error
                        updatedDistanceError = self.moveLAndFindError(axis, inverseIncrement)

                        # Query if the new movement generated a smaller distance error than prior, if so assign result to distance error variable
                        if updatedDistanceError < distanceError:
                            
                            distanceError = updatedDistanceError

                        else:

                            # Call function to revert manipulator pose and retrieve initial distance error
                            distanceError = self.moveLAndFindError(axis, increment)

                        print(" - | Queried the {} axis. | The distance error is {} mm. |".format(axis, distanceError))

                        # Add 1 to number of steps for hill climb
                        numOfSteps += 1

                        # Check if the distance error is less than or equal to the distance threshold
                        if distanceError <= self.distanceThreshold:
                            print("\nThe cartesian error has been successfully handled!")
                            break

            time.sleep(2)
            print("\nNow handling the Orientational error:\n")
   
            while rotationError > self.orientationThreshold:

                # For loop for iterating through catesian axis list (cartesianAxes)
                for force in self.rotationalForces:
                    
                    # If statement to query forceName and determine force axis
                    if force == "Roll":
                        forceAxis = "X"
                    elif force == "Pitch":
                        forceAxis = "Y"
                    else:
                        forceAxis = "Z"

                    # Assign rotation error as angular movement deviation
                    deviation = rotationError

                    # Call function to update manipulator pose and retrieve new rotation error
                    updatedRotationError = self.rotateEndEffectorFindError(force, deviation)

                    # If statement to query if the new movement generated a smaller rotation error than prior, if so assign result to rotation error variable
                    if updatedRotationError < rotationError:

                        rotationError = updatedRotationError

                    else:

                        # Assign inverse deviation variable by multiplying the deviation by -2, subsequently moving (rotating) the robot in the opposite direction
                        inverseDeviation = deviation * -2

                        # Call function to update manipulator pose and retrieve new rotation error
                        updatedRotationError = self.rotateEndEffectorFindError(force, inverseDeviation)

                        if updatedRotationError < rotationError:

                            rotationError = updatedRotationError
                        
                        else:

                            # Call function to revert manipulator pose and retrieve initial rotation error
                            rotationError = self.rotateEndEffectorFindError(force, deviation)

                    print(" - | Queried force - {} ({} axis). | The rotation error (angular deviation) is {} degrees |".format(force, forceAxis, rotationError))
                    
                    numOfSteps += 1

                    # Check if the rotation error is less than or equal to the orientational threshold
                    if rotationError <= self.orientationThreshold:
                        print("\nThe orientational error has been successfully handled!")
                        break
            
            # Assign ball state boolean value to true as both errors have been handled
            ballCaught = True

        end = time.monotonic()

        # Call function to calculate search cycle duration time
        duration = self.calculateSearchTime(beginning, end, 4)

        print("\nThe ball has succesfully been caught!")
        print("Number of steps taken: {}".format(numOfSteps))
        print("Search Cycle Duration: {}".format(duration))

        print("\nA new trajectory will now be generated")

        time.sleep(2)


root = TrajectoryFinder()

try:
    while True:
        # Call find trajectory method
        root.findTrajectory()
except KeyboardInterrupt:
    pass
