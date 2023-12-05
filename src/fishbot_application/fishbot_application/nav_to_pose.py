
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time


def main():
    print("1")
    rclpy.init()
    navigator = BasicNavigator()
    print("2")
    # 等待导航启动完成
    # navigator.waitUntilNav2Active()
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(init_pose)
    # navigator.waitUntilNav2Active()
    time.sleep(5)
    # 设置目标点坐标
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.3
    goal_pose.pose.position.y = 0.3
    goal_pose.pose.orientation.w = 1.0
    print("3")
    # 发送目标接收反馈结果
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f'Plan to arrive in {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s')
        # 超时自动取消
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
    # 最终结果判断
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Results: Successed')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Results: Cancelled')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Results: Failed')
    else:
        navigator.get_logger().error('Results: Invaild')
