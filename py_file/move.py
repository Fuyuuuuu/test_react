import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import Int16
#import ggcnn # predict grasp point
#import gqcnn # predict grasp point

#from svd import transform

class ExampleFullArmMovement:
    def __init__(self):
        rospy.init_node('example_full_arm_movement_python')
        self.svd = False
        self.HOME_ACTION_IDENTIFIER = 2
        self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
        self.base_path = "/" + self.robot_name
        self.degrees_of_freedom = rospy.get_param(self.base_path + "/degrees_of_freedom", 6)
        self.is_gripper_present = rospy.get_param(self.base_path + "/is_gripper_present", False)

        self.log_robot_info()

        self.action_topic_sub = rospy.Subscriber(self.base_path + "/action_topic", ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        self.image_sub = rospy.Subscriber('/camera/signal',Int16,self.start_movement_callback)

        # Init the action topic subscriber
        self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

        get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
        rospy.wait_for_service(get_product_configuration_full_name)
        self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

        validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

        self.is_init_success = True

    def log_robot_info(self):
        rospy.loginfo(f"Using robot_name {self.robot_name}, robot has {self.degrees_of_freedom} degrees of freedom and is_gripper_present is {self.is_gripper_present}")

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)
        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(0.01)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(0.01)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.01)
        return True

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(1)
            return True

    def back_to_begin(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        trajectory.waypoints.append(self.FillCartesianWaypoint(0.24854257702827454, 0.29243361949920654, 0.3, -89.08971405029297, 175.7831573486328, 1.8741217851638794,0))
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def go_down(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        trajectory.waypoints.append(self.FillCartesianWaypoint(0.24854257702827454, 0.29243361949920654, -0.0027699354104697704, -89.08971405029297, 175.7831573486328, 1.8741217851638794,0))
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def pick(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()
        trajectory.waypoints.append(self.FillCartesianWaypoint(0.24854257702827454, 0.29243361949920654, 0.3, -89.08971405029297, 175.7831573486328, 1.8741217851638794,0))
        trajectory.waypoints.append(self.FillCartesianWaypoint(0.24854257702827454, 0.12243361949920654, 0.3, -89.08971405029297, 175.7831573486328, 1.8741217851638794,0))
        trajectory.waypoints.append(self.FillCartesianWaypoint(0.24854257702827454, 0.12243361949920654, 0.02439967170357704, -89.08971405029297, 175.7831573486328, 1.8741217851638794,0))
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def start_movement_callback(self, msg):
        try:
            if msg.data == 1:
                rospy.sleep(5)
                self.execute_robot_sequence()
        except Exception as e:
            rospy.logerr(f"在執行過程中發生錯誤: {e}")

    # def start_movement_callback(self, msg):
    #     if msg.data == 1:
    #       if self.svd:
    #             self.example_clear_faults()
    #             self.example_subscribe_to_a_robot_notification()
    #             self.back_to_begin()
    #             self.go_down()
    #             #self.go_position, # 從gqcnn、ggcnn、座標轉換取得的資訊
    #             self.example_send_gripper_command(1)
    #             self.example_send_gripper_command(1)
    #             self.example_set_cartesian_reference_frame()
    #             self.pick()
    #             self.example_send_gripper_command(0)
    #             self.example_send_gripper_command(0)
    #             self.back_to_begin()

    #       else:
    #            self.example_clear_faults()
    #            self.example_subscribe_to_a_robot_notification()
    #            self.back_to_begin()
    #            self.go_down()
    #            self.example_send_gripper_command(1)
    #            self.example_send_gripper_command(1)
    #            self.example_set_cartesian_reference_frame()
    #            self.pick()
    #            self.example_send_gripper_command(0)
    #            self.example_send_gripper_command(0)
    #            self.back_to_begin()

    def execute_robot_sequence(self):
        if not self.example_subscribe_to_a_robot_notification():
            rospy.logerr("訂閱機器人通知失敗")
            return

        if not self.example_clear_faults():
            rospy.logerr("清除錯誤失敗")
            return

        if not self.back_to_begin():
            rospy.logerr("返回初始位置失敗")
            return

        if not self.go_down():
            rospy.logerr("下降動作失敗")
            return

        if not self.example_send_gripper_command(1):
            rospy.logerr("發送夾子命令1失敗")
            return

        if not self.example_send_gripper_command(1):
            rospy.logerr("發送夾子命令1失敗")
            return

        if not self.example_set_cartesian_reference_frame():
            rospy.logerr("設置笛卡爾參考框架失敗")
            return

        if not self.pick():
            rospy.logerr("選擇動作失敗")
            return

        if not self.example_send_gripper_command(0):
            rospy.logerr("發送夾子命令0失敗")
            return

        if not self.example_send_gripper_command(0):
            rospy.logerr("發送夾子命令1失敗")
            return

        if not self.back_to_begin():
            rospy.logerr("返回初始位置失敗")
            return

    def main(self):
        try:
            if self.is_init_success:
                self.example_home_the_robot()
                #寫入資料庫
                rospy.spin()
            else:
                rospy.logerr('初始化錯誤')
        except Exception as e:
            rospy.logerr(f"在main中發生錯誤: {e}")
        finally:
            self.example_home_the_robot()
            #寫入資料庫

if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()

