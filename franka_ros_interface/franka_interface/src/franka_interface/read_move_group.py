

    Main Page
    Namespaces
    Classes
    Files

    File List

move_group.py
Go to the documentation of this file.

00001 # Software License Agreement (BSD License)
00002 #
00003 # Copyright (c) 2012, Willow Garage, Inc.
00004 # All rights reserved.
00005 #
00006 # Redistribution and use in source and binary forms, with or without
00007 # modification, are permitted provided that the following conditions
00008 # are met:
00009 #
00010 #  * Redistributions of source code must retain the above copyright
00011 #    notice, this list of conditions and the following disclaimer.
00012 #  * Redistributions in binary form must reproduce the above
00013 #    copyright notice, this list of conditions and the following
00014 #    disclaimer in the documentation and/or other materials provided
00015 #    with the distribution.
00016 #  * Neither the name of Willow Garage, Inc. nor the names of its
00017 #    contributors may be used to endorse or promote products derived
00018 #    from this software without specific prior written permission.
00019 #
00020 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
00021 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
00022 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
00023 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
00024 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
00025 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
00026 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00027 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
00028 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
00029 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
00030 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00031 # POSSIBILITY OF SUCH DAMAGE.
00032 #
00033 # Author: Ioan Sucan
00034 
00035 from geometry_msgs.msg import Pose, PoseStamped
00036 from moveit_msgs.msg import RobotTrajectory, Grasp, PlaceLocation, Constraints
00037 from sensor_msgs.msg import JointState
00038 import rospy
00039 import tf
00040 from moveit_ros_planning_interface import _moveit_move_group_interface
00041 from exception import MoveItCommanderException
00042 import conversions
00043 
00044 class MoveGroupCommander(object):
00045     """
00046     Execution of simple commands for a particular group
00047     """
00048 
00049     def __init__(self, name, robot_description="robot_description"):
00050         """ Specify the group name for which to construct this commander instance. Throws an exception if there is an initialization error. """
00051         self._g = _moveit_move_group_interface.MoveGroup(name, robot_description)
00052 
00053     def get_name(self):
00054         """ Get the name of the group this instance was initialized for """
00055         return self._g.get_name()
00056 
00057     def stop(self):
00058         """ Stop the current execution, if any """
00059         self._g.stop()
00060 
00061     def get_active_joints(self):
00062         """ Get the active joints of this group """
00063         return self._g.get_active_joints()
00064 
00065     def get_joints(self):
00066         """ Get the joints of this group """
00067         return self._g.get_joints()
00068 
00069     def get_variable_count(self):
00070         """ Return the number of variables used to parameterize a state in this group (larger or equal to number of DOF)"""
00071         return self._g.get_variable_count()
00072 
00073     def has_end_effector_link(self):
00074         """ Check if this group has a link that is considered to be an end effector """
00075         return len(self._g.get_end_effector_link()) > 0
00076 
00077     def get_end_effector_link(self):
00078         """ Get the name of the link that is considered to be an end-effector. Return an empty string if there is no end-effector. """
00079         return self._g.get_end_effector_link()
00080 
00081     def set_end_effector_link(self, link_name):
00082         """ Set the name of the link to be considered as an end effector """
00083         if not self._g.set_end_effector_link(link_name):
00084             raise MoveItCommanderException("Unable to set end efector link")
00085 
00086     def get_pose_reference_frame(self):
00087         """ Get the reference frame assumed for poses of end-effectors """
00088         return self._g.get_pose_reference_frame()
00089 
00090     def set_pose_reference_frame(self, reference_frame):
00091         """ Set the reference frame to assume for poses of end-effectors """
00092         self._g.set_pose_reference_frame(reference_frame)
00093     
00094     def get_planning_frame(self):
00095         """ Get the name of the frame where all planning is performed """
00096         return self._g.get_planning_frame()
00097 
00098     def get_current_joint_values(self):
00099         """ Get the current configuration of the group as a list (these are values published on /joint_states) """
00100         return self._g.get_current_joint_values()
00101 
00102     def get_current_pose(self, end_effector_link = ""):
00103         """ Get the current pose of the end-effector of the group. Throws an exception if there is not end-effector. """
00104         if len(end_effector_link) > 0 or self.has_end_effector_link():
00105             return conversions.list_to_pose_stamped(self._g.get_current_pose(end_effector_link), self.get_planning_frame())
00106         else:
00107             raise MoveItCommanderException("There is no end effector to get the pose of")
00108 
00109     def get_current_rpy(self, end_effector_link = ""):
00110         """ Get a list of 3 elements defining the [roll, pitch, yaw] of the end-effector. Throws an exception if there is not end-effector. """
00111         if len(end_effector_link) > 0 or self.has_end_effector_link():
00112             return self._g.get_current_rpy(end_effector_link)
00113         else:
00114             raise MoveItCommanderException("There is no end effector to get the rpy of")
00115 
00116     def get_random_joint_values(self):
00117         return self._g.get_random_joint_values()
00118 
00119     def get_random_pose(self, end_effector_link = ""):
00120         if len(end_effector_link) > 0 or self.has_end_effector_link():
00121             return conversions.list_to_pose_stamped(self._g.get_random_pose(end_effector_link), self.get_planning_frame())
00122         else:
00123             raise MoveItCommanderException("There is no end effector to get the pose of")
00124 
00125     def set_start_state_to_current_state(self):
00126         self._g.set_start_state_to_current_state()
00127 
00128     def set_start_state(self, msg):
00129         """
00130         Specify a start state for the group.
00131 
00132         Parameters
00133         ----------
00134         msg : moveit_msgs/RobotState
00135 
00136         Examples
00137         --------
00138         >>> from moveit_msgs.msg import RobotState
00139         >>> from sensor_msgs.msg import JointState
00140         >>> joint_state = JointState()
00141         >>> joint_state.header = Header()
00142         >>> joint_state.header.stamp = rospy.Time.now()
00143         >>> joint_state.name = ['joint_a', 'joint_b']
00144         >>> joint_state.position = [0.17, 0.34]
00145         >>> moveit_robot_state = RobotState()
00146         >>> moveit_robot_state.joint_state = joint_state
00147         >>> group.set_start_state(moveit_robot_state)
00148         """
00149         self._g.set_start_state(conversions.msg_to_string(msg))
00150 
00151     def get_joint_value_target(self):
00152         return self._g.get_joint_value_target()
00153 
00154     def set_joint_value_target(self, arg1, arg2 = None, arg3 = None):
00155         """
00156         Specify a target joint configuration for the group.
00157         - if the type of arg1 is one of the following: dict, list, JointState message, then no other arguments should be provided.
00158         The dict should specify pairs of joint variable names and their target values, the list should specify all the variable values
00159         for the group. The JointState message specifies the positions of some single-dof joints. 
00160         - if the type of arg1 is string, then arg2 is expected to be defined and be either a real value or a list of real values. This is
00161         interpreted as setting a particular joint to a particular value.
00162         - if the type of arg1 is Pose or PoseStamped, both arg2 and arg3 could be defined. If arg2 or arg3 are defined, their types must
00163         be either string or bool. The string type argument is interpreted as the end-effector the pose is specified for (default is to use
00164         the default end-effector), and the bool is used to decide whether the pose specified is approximate (default is false). This situation
00165         allows setting the joint target of the group by calling IK. This does not send a pose to the planner and the planner will do no IK.
00166         Instead, one IK solution will be computed first, and that will be sent to the planner. 
00167         """
00168         if type(arg1) is JointState:
00169             if (arg2 != None or arg3 != None):
00170                 raise MoveItCommanderException("Too many arguments specified")
00171             if not self._g.set_joint_value_target_from_joint_state_message(conversions.msg_to_string(arg1)):
00172                 raise MoveItCommanderException("Error setting joint target. Is the target within bounds?")
00173 
00174         elif (type(arg1) is str):
00175             if (arg2 == None):
00176                 raise MoveItCommanderException("Joint value expected when joint name specified")
00177             if (arg3 != None):
00178                 raise MoveItCommanderException("Too many arguments specified")
00179             if not self._g.set_joint_value_target(arg1, arg2):
00180                 raise MoveItCommanderException("Error setting joint target. Is the target within bounds?")
00181 
00182         elif (type(arg1) is PoseStamped) or (type(arg1) is Pose):
00183             approx = False
00184             eef = ""
00185             if (arg2 != None):
00186                 if type(arg2) is str:
00187                     eef = arg2
00188                 else:
00189                     if type(arg2) is bool:
00190                         approx = arg2
00191                     else:
00192                         raise MoveItCommanderException("Unexpected type")
00193             if (arg3 != None):
00194                 if type(arg3) is str:
00195                     eef = arg3
00196                 else:
00197                     if type(arg3) is bool:
00198                         approx = arg3
00199                     else:
00200                         raise MoveItCommanderException("Unexpected type")
00201             r = False
00202             if type(arg1) is PoseStamped:
00203                 r = self._g.set_joint_value_target_from_pose_stamped(conversions.msg_to_string(arg1), eef, approx)
00204             else:
00205                 r = self._g.set_joint_value_target_from_pose(conversions.msg_to_string(arg1), eef, approx)
00206             if not r:
00207                 if approx:
00208                     raise MoveItCommanderException("Error setting joint target. Does your IK solver support approximate IK?")
00209                 else:
00210                     raise MoveItCommanderException("Error setting joint target. Is IK running?")
00211 
00212         elif (hasattr(arg1, '__iter__')):
00213             if (arg2 != None or arg3 != None):
00214                 raise MoveItCommanderException("Too many arguments specified")
00215             if not self._g.set_joint_value_target(arg1):
00216                 raise MoveItCommanderException("Error setting joint target. Is the target within bounds?")
00217 
00218         else:
00219             raise MoveItCommanderException("Unsupported argument of type %s" % type(arg1))
00220 
00221 
00222     def set_rpy_target(self, rpy, end_effector_link = ""):
00223         """ Specify a target orientation for the end-effector. Any position of the end-effector is acceptable."""
00224         if len(end_effector_link) > 0 or self.has_end_effector_link():
00225             if len(rpy) == 3:
00226                 if not self._g.set_rpy_target(rpy[0], rpy[1], rpy[2], end_effector_link):
00227                     raise MoveItCommanderException("Unable to set orientation target")
00228             else:
00229                 raise MoveItCommanderException("Expected [roll, pitch, yaw]")
00230         else:
00231             raise MoveItCommanderException("There is no end effector to set the pose for")
00232 
00233     def set_orientation_target(self, q, end_effector_link = ""):
00234         """ Specify a target orientation for the end-effector. Any position of the end-effector is acceptable."""
00235         if len(end_effector_link) > 0 or self.has_end_effector_link():
00236             if len(q) == 4:
00237                 if not self._g.set_orientation_target(q[0], q[1], q[2], q[3], end_effector_link):
00238                     raise MoveItCommanderException("Unable to set orientation target")
00239             else:
00240                 raise MoveItCommanderException("Expected [qx, qy, qz, qw]")
00241         else:
00242             raise MoveItCommanderException("There is no end effector to set the pose for")
00243 
00244     def set_position_target(self, xyz, end_effector_link = ""):
00245         """ Specify a target position for the end-effector. Any orientation of the end-effector is acceptable."""
00246         if len(end_effector_link) > 0 or self.has_end_effector_link():
00247             if not self._g.set_position_target(xyz[0], xyz[1], xyz[2], end_effector_link):
00248                 raise MoveItCommanderException("Unable to set position target")
00249         else:
00250             raise MoveItCommanderException("There is no end effector to set the pose for")
00251 
00252     def set_pose_target(self, pose, end_effector_link = ""):
00253         """ Set the pose of the end-effector, if one is available. The expected input is a Pose message, a PoseStamped message or a list of 6 floats:"""
00254         """ [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
00255         if len(end_effector_link) > 0 or self.has_end_effector_link():
00256             ok = False
00257             if type(pose) is PoseStamped:
00258                 old = self.get_pose_reference_frame()
00259                 self.set_pose_reference_frame(pose.header.frame_id)
00260                 ok = self._g.set_pose_target(conversions.pose_to_list(pose.pose), end_effector_link)
00261                 self.set_pose_reference_frame(old)
00262             elif type(pose) is Pose:
00263                 ok = self._g.set_pose_target(conversions.pose_to_list(pose), end_effector_link)
00264             else:
00265                 ok = self._g.set_pose_target(pose, end_effector_link)
00266             if not ok:
00267                 raise MoveItCommanderException("Unable to set target pose")
00268         else:
00269             raise MoveItCommanderException("There is no end effector to set the pose for")
00270 
00271     def set_pose_targets(self, poses, end_effector_link = ""):
00272         """ Set the pose of the end-effector, if one is available. The expected input is a list of poses. Each pose can be a Pose message, a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """
00273         if len(end_effector_link) > 0 or self.has_end_effector_link():
00274             if not self._g.set_pose_targets([conversions.pose_to_list(p) if type(p) is Pose else p for p in poses], end_effector_link):
00275                 raise MoveItCommanderException("Unable to set target poses")
00276         else:
00277             raise MoveItCommanderException("There is no end effector to set poses for")
00278 
00279     def shift_pose_target(self, axis, value, end_effector_link = ""):
00280         """ Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target """
00281         if len(end_effector_link) > 0 or self.has_end_effector_link():
00282             pose = self._g.get_current_pose(end_effector_link)
00283             # by default we get orientation as a quaternion list
00284             # if we are updating a rotation axis however, we convert the orientation to RPY
00285             if axis > 2:
00286                 (r, p, y) = tf.transformations.euler_from_quaternion(pose[3:])
00287                 pose = [pose[0], pose[1], pose[2], r, p, y]
00288             if axis >= 0 and axis < 6:
00289                 pose[axis] = pose[axis] + value
00290                 self.set_pose_target(pose, end_effector_link)
00291             else:
00292                 raise MoveItCommanderException("An axis value between 0 and 5 expected")
00293         else:
00294             raise MoveItCommanderException("There is no end effector to set poses for")
00295 
00296     def clear_pose_target(self, end_effector_link):
00297         """ Clear the pose target for a particular end-effector """
00298         self._g.clear_pose_target(end_effector_link)
00299         
00300     def clear_pose_targets(self):
00301         """ Clear all known pose targets """
00302         self._g.clear_pose_targets()
00303 
00304     def set_random_target(self):
00305         """ Set a random joint configuration target """
00306         self._g.set_random_target()
00307 
00308     def set_named_target(self, name):
00309         """ Set a joint configuration by name. The name can be a name previlusy remembered with remember_joint_values() or a configuration specified in the SRDF. """
00310         if not self._g.set_named_target(name):
00311             raise MoveItCommanderException("Unable to set target %s. Is the target within bounds?" % name)
00312 
00313     def remember_joint_values(self, name, values = None):
00314         """ Record the specified joint configuration of the group under the specified name. If no values are specified, the current state of the group is recorded. """
00315         if values == None:
00316             values = self.get_current_joint_values()
00317         self._g.remember_joint_values(name, values)
00318 
00319     def get_remembered_joint_values(self):
00320         """ Get a dictionary that maps names to joint configurations for the group """
00321         return self._g.get_remembered_joint_values()
00322     
00323     def forget_joint_values(self, name):
00324         """ Forget a stored joint configuration """
00325         self._g.forget_joint_values(name)
00326 
00327     def get_goal_tolerance(self):
00328         """ Return a tuple of goal tolerances: joint, position and orientation. """
00329         return (self.get_goal_joint_tolerance(), self.get_goal_position_tolerance(), self.get_goal_orientation_tolerance())
00330 
00331     def get_goal_joint_tolerance(self):
00332         """ Get the tolerance for achieving a joint goal (distance for each joint variable) """
00333         return self._g.get_goal_joint_tolerance()
00334 
00335     def get_goal_position_tolerance(self):
00336         """ When moving to a position goal or to a pose goal, the tolerance for the goal position is specified as the radius a sphere around the target origin of the end-effector """
00337         return self._g.get_goal_position_tolerance()
00338 
00339     def get_goal_orientation_tolerance(self):
00340         """ When moving to an orientation goal or to a pose goal, the tolerance for the goal orientation is specified as the distance (roll, pitch, yaw) to the target origin of the end-effector """
00341         return self._g.get_goal_orientation_tolerance()
00342 
00343     def set_goal_tolerance(self, value):
00344         """ Set the joint, position and orientation goal tolerances simultaneously """
00345         self._g.set_goal_tolerance(value)
00346 
00347     def set_goal_joint_tolerance(self, value):
00348         """ Set the tolerance for a target joint configuration """
00349         self._g.set_goal_joint_tolerance(value)
00350 
00351     def set_goal_position_tolerance(self, value):
00352         """ Set the tolerance for a target end-effector position """
00353         self._g.set_goal_position_tolerance(value)
00354 
00355     def set_goal_orientation_tolerance(self, value):
00356         """ Set the tolerance for a target end-effector orientation """
00357         self._g.set_goal_orientation_tolerance(value)
00358 
00359     def allow_looking(self, value):
00360         """ Enable/disable looking around for motion planning """
00361         self._g.allow_looking(value)
00362 
00363     def allow_replanning(self, value):
00364         """ Enable/disable replanning """
00365         self._g.allow_replanning(value)
00366         
00367     def get_known_constraints(self):
00368         """ Get a list of names for the constraints specific for this group, as read from the warehouse """
00369         return self._g.get_known_constraints()
00370 
00371     def get_path_constraints(self):
00372         """ Get the acutal path constraints in form of a moveit_msgs.msgs.Constraints """
00373         c = Constraints()
00374         c_str = self._g.get_path_constraints()
00375         conversions.msg_from_string(c,c_str)
00376         return c
00377 
00378     def set_path_constraints(self, value):
00379         """ Specify the path constraints to be used (as read from the database) """
00380         if value == None:
00381             self.clear_path_constraints()
00382         else:
00383             if type(value) is Constraints:
00384                 self._g.set_path_constraints_from_msg(conversions.msg_to_string(value))
00385             elif not self._g.set_path_constraints(value):
00386                 raise MoveItCommanderException("Unable to set path constraints " + value)
00387 
00388     def clear_path_constraints(self):
00389         """ Specify that no path constraints are to be used during motion planning """
00390         self._g.clear_path_constraints()
00391 
00392     def set_constraints_database(self, host, port):
00393         """ Specify which database to connect to for loading possible path constraints """
00394         self._g.set_constraints_database(host, port)
00395 
00396     def set_planning_time(self, seconds):
00397         """ Specify the amount of time to be used for motion planning. """
00398         self._g.set_planning_time(seconds)
00399 
00400     def get_planning_time(self):
00401         """ Specify the amount of time to be used for motion planning. """
00402         return self._g.get_planning_time()
00403 
00404     def set_planner_id(self, planner_id):
00405         """ Specify which planner to use when motion planning """
00406         self._g.set_planner_id(planner_id)
00407 
00408     def set_num_planning_attempts(self, num_planning_attempts):
00409         """ Set the number of times the motion plan is to be computed from scratch before the shortest solution is returned. The default value is 1. """
00410         self._g.set_num_planning_attempts(num_planning_attempts)
00411 
00412     def set_workspace(self, ws):
00413         """ Set the workspace for the robot as either [], [minX, minY, maxX, maxY] or [minX, minY, minZ, maxX, maxY, maxZ] """
00414         if len(ws) == 0:
00415             self._g.set_workspace(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
00416         else:
00417             if len(ws) == 4:
00418                 self._g.set_workspace(ws[0], ws[1], 0.0, ws[2], ws[3], 0.0)
00419             else:
00420                 if len(ws) == 6:
00421                     self._g.set_workspace(ws[0], ws[1], ws[2], ws[3], ws[4], ws[5])
00422                 else:
00423                     raise MoveItCommanderException("Expected 0, 4 or 6 values in list specifying workspace")
00424 
00425     def set_max_velocity_scaling_factor(self, value):
00426         """ Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1]. """        
00427         if value > 0 and value <= 1:
00428             self._g.set_max_velocity_scaling_factor(value)
00429         else:
00430             raise MoveItCommanderException("Expected value in the range from 0 to 1 for scaling factor" )
00431 
00432     def set_max_acceleration_scaling_factor(self, value):
00433         """ Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1]. """
00434         if value > 0 and value <= 1:
00435             self._g.set_max_acceleration_scaling_factor(value)
00436         else:
00437             raise MoveItCommanderException("Expected value in the range from 0 to 1 for scaling factor" )
00438 
00439     def go(self, joints = None, wait = True):
00440         """ Set the target of the group and then move the group to the specified target """
00441         if type(joints) is bool:
00442             wait = joints
00443             joints = None
00444 
00445         elif type(joints) is JointState:
00446             self.set_joint_value_target(joints)
00447 
00448         elif type(joints) is Pose:
00449             self.set_pose_target(joints)
00450 
00451         elif not joints == None:
00452             try:
00453                 self.set_joint_value_target(self.get_remembered_joint_values()[joints])
00454             except:
00455                 self.set_joint_value_target(joints)
00456         if wait:
00457             return self._g.move()
00458         else:
00459             return self._g.async_move()
00460 
00461     def plan(self, joints = None):
00462         """ Return a motion plan (a RobotTrajectory) to the set goal state (or specified by the joints argument) """
00463         if type(joints) is JointState:
00464             self.set_joint_value_target(joints)
00465 
00466         elif type(joints) is Pose:
00467             self.set_pose_target(joints)
00468 
00469         elif not joints == None:
00470             try:
00471                 self.set_joint_value_target(self.get_remembered_joint_values()[joints])
00472             except:
00473                 self.set_joint_value_target(joints)
00474         plan = RobotTrajectory()
00475         plan.deserialize(self._g.compute_plan())
00476         return plan
00477 
00478     def compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisions = True):
00479         """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """
00480         (ser_path, fraction) = self._g.compute_cartesian_path([conversions.pose_to_list(p) for p in waypoints], eef_step, jump_threshold, avoid_collisions)
00481         path = RobotTrajectory()
00482         path.deserialize(ser_path)
00483         return (path, fraction)
00484 
00485     def execute(self, plan_msg, wait = True):
00486         """Execute a previously planned path"""
00487         if wait:
00488             return self._g.execute(conversions.msg_to_string(plan_msg))
00489         else:
00490             return self._g.async_execute(conversions.msg_to_string(plan_msg))
00491 
00492     def attach_object(self, object_name, link_name = "", touch_links = []):
00493         """ Given the name of an object existing in the planning scene, attach it to a link. The link used is specified by the second argument. If left unspecified, the end-effector link is used, if one is known. If there is no end-effector link, the first link in the group is used. If no link is identified, failure is reported. True is returned if an attach request was succesfully sent to the move_group node. This does not verify that the attach request also was successfuly applied by move_group."""
00494         return self._g.attach_object(object_name, link_name, touch_links)
00495 
00496     def detach_object(self, name = ""):
00497         """ Given the name of a link, detach the object(s) from that link. If no such link exists, the name is interpreted as an object name. If there is no name specified, an attempt is made to detach all objects attached to any link in the group."""
00498         return self._g.detach_object(name)
00499 
00500     def pick(self, object_name, grasp = []):
00501         """Pick the named object. A grasp message, or a list of Grasp messages can also be specified as argument."""
00502         if type(grasp) is Grasp:
00503             return self._g.pick(object_name, conversions.msg_to_string(grasp))
00504         else:
00505             return self._g.pick(object_name, [conversions.msg_to_string(x) for x in grasp])
00506 
00507     def place(self, object_name, location=None):
00508         """Place the named object at a particular location in the environment or somewhere safe in the world if location is not provided"""
00509         result = False
00510         if location is None:
00511             result = self._g.place(object_name)
00512         elif type(location) is PoseStamped:
00513             old = self.get_pose_reference_frame()
00514             self.set_pose_reference_frame(location.header.frame_id)
00515             result = self._g.place(object_name, conversions.pose_to_list(location.pose))
00516             self.set_pose_reference_frame(old)
00517         elif type(location) is Pose:
00518             result = self._g.place(object_name, conversions.pose_to_list(location))
00519         elif type(location) is PlaceLocation:
00520             result = self._g.place(object_name, conversions.msg_to_string(location))
00521         else:
00522             raise MoveItCommanderException("Parameter location must be a Pose, PoseStamped or PlaceLocation object")
00523         return result
00524 
00525     def set_support_surface_name(self, value):
00526         """ Set the support surface name for a place operation """
00527         self._g.set_support_surface_name(value)
00528 
00529     def retime_trajectory(self, ref_state_in, traj_in, velocity_scaling_factor):
00530         ser_ref_state_in = conversions.msg_to_string(ref_state_in)
00531         ser_traj_in = conversions.msg_to_string(traj_in)
00532         ser_traj_out = self._g.retime_trajectory(ser_ref_state_in, ser_traj_in, velocity_scaling_factor)
00533         traj_out = RobotTrajectory()
00534         traj_out.deserialize(ser_traj_out)
00535         return traj_out


moveit_commander
Author(s): Ioan Sucan
autogenerated on Mon Jul 24 2017 02:22:10
