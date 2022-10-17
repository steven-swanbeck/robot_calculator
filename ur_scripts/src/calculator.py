#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from visualization_msgs.msg import Marker

tau = 2 * 3.1415926535897932384626433832795028841971693993751058

class MoveGroupPythonCalculator(object):
    
    def __init__(self):
        super(MoveGroupPythonCalculator, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_calculator", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.solution = 19.72

    # ========================================================================
    # Calculator Functions
    # ========================================================================
    def isNumeric(self,input):
        try:
            float(input)
            return True
        except ValueError:
            return False

    def process(self,calc_request):
        stop_flag = False
        continue_flag = True
        while stop_flag == False:
            for i in range(0, len(calc_request)):
                # Negative alterations
                if calc_request[i] == '-':
                    # converting - - to +
                    if calc_request[i + 1] == '-':
                        calc_request[i] = '+'
                        calc_request.pop(i + 1)
                        stop_flag = True
                        break
                    # converting - ( to -1 * (
                    if calc_request[i - 1] == '(':
                        if self.isNumeric(calc_request[i + 1]):
                            calc_request[i + 1] = str(-1 * float(calc_request[i + 1]))
                            calc_request.pop(i)
                        else:
                            calc_request[i] = '-1'
                        stop_flag = True
                        break
                    # converting ^ - to ^ ( -  ...)
                    if calc_request[i - 1] == '^':
                        calc_request.insert(i, '(')
                        corresponding_paren_set = False
                        for j in range(i, len(calc_request)):
                            if calc_request[j] == ')':
                                calc_request.insert(j + 1, ')')
                                corresponding_paren_set = True
                        if corresponding_paren_set == False:
                            calc_request.insert(i + 3, ')')
                        stop_flag = True
                        break
                # Accounting for implicit * with ()
                if calc_request[i] == '(' and i >= 1:
                    if self.isNumeric(calc_request[i - 1]) or calc_request[i - 1] == ')':
                        calc_request.insert(i, '*')
                        stop_flag = True
                        break
                else:
                    stop_flag = True
                    if i == len(calc_request) - 1:
                        continue_flag = False
        if continue_flag == True:
            stop_flag = False
            self.process(calc_request)
        return calc_request

    def executeCalculation(self,outputs):
        stop_flag = False
        while stop_flag == False:
            for i in range(len(outputs)):
                if outputs[i] == '+':
                    execute_operation = float(outputs[i - 2]) + float(outputs[i - 1])
                    outputs[i] = str(execute_operation)
                    outputs.pop(i - 1)
                    outputs.pop(i - 2)
                    stop_flag = True
                    break
                elif outputs[i] == '-':
                    try:
                        execute_operation = float(outputs[i - 2]) - float(outputs[i - 1])
                        outputs[i] = str(execute_operation)
                        outputs.pop(i - 1)
                        outputs.pop(i - 2)
                    except ValueError:
                        execute_operation = 0 - float(outputs[i - 1])
                        outputs[i] = str(execute_operation)
                        outputs.pop(i - 1)
                    stop_flag = True
                    break
                elif outputs[i] == '*':
                    execute_operation = float(outputs[i - 2]) * float(outputs[i - 1])
                    outputs[i] = str(execute_operation)
                    outputs.pop(i - 1)
                    outputs.pop(i - 2)
                    stop_flag = True
                    break
                elif outputs[i] == '/':
                    execute_operation = float(outputs[i - 2]) / float(outputs[i - 1])
                    outputs[i] = str(execute_operation)
                    outputs.pop(i - 1)
                    outputs.pop(i - 2)
                    stop_flag = True
                    break
                elif outputs[i] == '^':
                    execute_operation = float(outputs[i - 2]) ** float(outputs[i - 1])
                    outputs[i] = str(execute_operation)
                    outputs.pop(i - 1)
                    outputs.pop(i - 2)
                    stop_flag = True
                    break
        if len(outputs) > 1:
            stop_flag = False
            self.executeCalculation(outputs)
        else:
            self.solution = outputs[0]
            return outputs[0]

    def myCalculator2C(self):
        calc_request = list(map(str,input("Enter expression: ").strip().split(' ')))
        if calc_request[0] != 'q':
            calc_request = self.process(calc_request)
            operator_stack = []
            outputs = []
            operator_precedence = {'+': 1, '-': 1, '*': 2, '/':2, '^': 3, '(': 4, ')': 4}
            for i in range(len(calc_request)):
                if self.isNumeric(calc_request[i]):
                    outputs.append(calc_request[i])
                elif calc_request[i] == '+' or calc_request[i] == '-' or calc_request[i] == '*' or calc_request[i] == '/' or calc_request[i] == '^':# and operator_stack[-1] != '(':
                    operator_stack.append(calc_request[i])
                    while len(operator_stack) > 1 and (operator_stack[-2] != '(') and (operator_precedence[calc_request[i]] <= operator_precedence[operator_stack[-2]]):
                        outputs.append(operator_stack[-2])
                        operator_stack.pop(-2)
                if calc_request[i] == '(':
                    operator_stack.append(calc_request[i])
                if calc_request[i] == ')':
                    while operator_stack[-1] != '(':
                        outputs.append(operator_stack[-1])
                        operator_stack.pop(-1)
                    if operator_stack[-1] == '(':
                        operator_stack.pop(-1)
            while len(operator_stack) > 0:
                outputs.append(operator_stack[-1])
                operator_stack.pop(-1)
            return self.executeCalculation(outputs)
        else:
            self.solution = 'q'
            return calc_request
    # ========================================================================
    # End Calculator Functions
    # ========================================================================

    # ========================================================================
    # Drawing Functions
    # ========================================================================
    def print_result(self):
        out = self.myCalculator2C()
        # print(self.solution)
        if self.solution != 'q':
            out_split = [str(dig) for dig in str(round(float(self.solution),1)).rstrip('0').rstrip('.')]
            # print(out_split)
            for number in out_split:
                if number == '0':
                    cartesian_plan, fraction = self.draw_0()
                    self.execute_plan(cartesian_plan)
                elif number == '1':
                    cartesian_plan, fraction, trace = self.draw_1()
                    self.execute_plan(cartesian_plan)
                elif number == '2':
                    cartesian_plan, fraction = self.draw_2()
                    self.execute_plan(cartesian_plan)
                elif number == '3':
                    cartesian_plan, fraction = self.draw_3()
                    self.execute_plan(cartesian_plan)
                elif number == '4':
                    cartesian_plan, fraction = self.draw_4()
                    self.execute_plan(cartesian_plan)
                elif number == '5':
                    cartesian_plan, fraction = self.draw_5()
                    self.execute_plan(cartesian_plan)
                elif number == '6':
                    cartesian_plan, fraction = self.draw_6()
                    self.execute_plan(cartesian_plan)
                elif number == '7':
                    cartesian_plan, fraction = self.draw_7()
                    self.execute_plan(cartesian_plan)
                elif number == '8':
                    cartesian_plan, fraction = self.draw_8()
                    self.execute_plan(cartesian_plan)
                elif number == '9':
                    cartesian_plan, fraction = self.draw_9()
                    self.execute_plan(cartesian_plan)
                else:
                    cartesian_plan, fraction = self.draw_dot()
                    self.execute_plan(cartesian_plan)
            self.print_result()

    def draw_dot(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.y -= scale * 0.5
        wpose.position.x -= scale * 0.25
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.25
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_0(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top right of box
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke up
        wpose.position.y += scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 0.5
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_1(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []
        trace = []

        wpose = move_group.get_current_pose().pose
        # Starting from center, move to top of box
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Downstroke
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Lift pen off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction, trace

    def draw_2(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # # Move to top left of box
        # wpose.position.x -= scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_3(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top left of box
        # wpose.position.x -= scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Go right and up halfway
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_4(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top left of box
        # wpose.position.x -= scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke up
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_5(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top right of box
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_6(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top right of box
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke up halfway
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        # wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_7(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top left of box
        # wpose.position.x -= scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down and left
        wpose.position.x -= scale * 1.0 * 0.5
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_8(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top right of box
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke up
        wpose.position.y += scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left and down halfway
        wpose.position.x -= scale * 1.0 * 0.5
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction

    def draw_9(self, scale=0.1, zscale=0.1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        # Move to top right of box
        wpose.position.x += scale * 0.5
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Touch pen to paper
        wpose.position.z -= zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down halfway
        wpose.position.y -= scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke right
        wpose.position.x += scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke up halfway
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Stroke down
        wpose.position.y -= scale * 1.0
        waypoints.append(copy.deepcopy(wpose))
        # Stroke left
        wpose.position.x -= scale * 1.0 * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Lift off paper
        wpose.position.z += zscale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Move back to voxel center
        wpose.position.x += scale * 1.0
        wpose.position.y += scale * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # Plan path
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001, 0.0)

        return plan, fraction
    # ========================================================================
    # End Drawing Functions
    # ========================================================================

    # ========================================================================
    # Misc Movement Functions
    # ========================================================================
    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6 

        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def go_to_pose_goal(self):
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def go_to_initial_goal(self):
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 1.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.0
        # pose_goal.position.x = -0.4
        # pose_goal.position.y = 0.2
        # pose_goal.position.z = 0.2
        pose_goal.position.x = -0.05
        pose_goal.position.y = 0.4
        pose_goal.position.z = 0.2

        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
    # ========================================================================
    # End Misc Movement Functions
    # ========================================================================

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("")
        input(
            "============ Press `Enter` to initialize ..."
        )
        tutorial = MoveGroupPythonCalculator()

        input("============ Press `Enter` to assume initial pose goal ...")
        tutorial.go_to_initial_goal()

        input("============ Press `Enter` to begin calculator ...")
        tutorial.print_result()

        print("============ Python calculator complete!")
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()