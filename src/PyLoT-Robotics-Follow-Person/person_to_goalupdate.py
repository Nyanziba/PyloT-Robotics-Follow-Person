#!/usr/bin/env python3
"""
PersonInfoをgoalupdateトピックに変換するROS2ノード
PersonInfoの位置情報からナビゲーション目標を生成します。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
from pylot_msgs.msg import PersonInfo
from tf_transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import os
import asyncio
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class PersonToGoalUpdate(Node):
    """PersonInfoの情報をgoalupdateトピックにPublishするノード"""
    
    def __init__(self):
        super().__init__('person_to_goalupdate')
        
        # パラメータの設定
        self.declare_parameter('person_topic', '/person_info')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_link')  # ロボットのフレームID
        self.declare_parameter('approach_distance', 0.5)  # 人に近づく距離[m]
        self.declare_parameter('approach_angle', 0.0)     # 人に向かう角度のオフセット[rad]
        self.declare_parameter('behavior_tree_xml', '/home/pylot/pylot_robocup/src/ros2_ddsm_robot/params/personfollow.xml')
        self.declare_parameter('timer_period', 0.5)  # タイマーの周期[秒]
        
        # パラメータの取得
        person_topic = self.get_parameter('person_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value
        self.approach_angle = self.get_parameter('approach_angle').get_parameter_value().double_value
        self.behavior_tree_xml = self.get_parameter('behavior_tree_xml').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        
        # TF2の設定
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 初回実行フラグ
        self.first_execution = True
        self.navigation_cancelled = False
        self.person_follow_active = True
        self.latest_goal_pose = None
        self.current_goal_handle = None
        
        # Action Clientの設定
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        
        # lambda式でgoal_poseパブリッシャーを作成
        self.goal_pose_publisher = (lambda: self.create_publisher(PoseStamped, '/goal_update', 10))()
        
        # Publisher とSubscriber の設定
       
        
        self.person_subscriber = self.create_subscription(
            PersonInfo,
            person_topic,
            self.person_info_callback,
            10
        )
        
        self.person_follow_activater = self.create_service( Trigger,
            '/person_follow_active',
            self.person_follow_activater_callback
        )
        self.person_follow_deactivater = self.create_service( Trigger,
            '/person_follow_deactive',
            self.person_follow_deactivater_callback
        )
        
        # タイマーの設定
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'Person to Goal Update node started')
        self.get_logger().info(f'Subscribing to: {person_topic}')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Robot Frame ID: {self.robot_frame_id}')
        self.get_logger().info(f'Approach distance: {self.approach_distance}m')
        self.get_logger().info(f'Timer period: {self.timer_period}s')
        
    def timer_callback(self):
        """定期的にgoalアクションを送信するタイマーコールバック"""
        if not self.person_follow_active:
            self.get_logger().debug('Timer: Person follow is not active')
            return
            
        if self.latest_goal_pose is None:
            self.get_logger().debug('Timer: No goal pose available')
            return
            
        if self.current_goal_handle is not None:
            # 現在のナビゲーションゴールがアクティブな場合は、何もしない
            self.get_logger().debug('Timer: Current goal is active, skipping navigation goal send')
            self.goal_pose_publisher.publish(self.latest_goal_pose)
        else:
            # 現在のナビゲーションゴールがアクティブでない場合は、最新のgoal_poseを送信
            self.get_logger().debug('Timer: No active goal, sending latest goal pose')
            # try:
            #     nav = BasicNavigator()
            #     nav.clearAllCostmaps()
            # except Exception as e:
            #     self.get_logger().warn(f'Failed to clear costmaps: {str(e)}')
            self.send_navigation_goal(self.latest_goal_pose)

        
    def person_follow_activater_callback(self, request, response):
        """person_follow_activeサービスのコールバック"""
        self.person_follow_active = True
        response.success = True
        response.message = "Person follow activated"
        self.get_logger().info(f'Person follow activated self.person_follow_active is  {self.person_follow_active}')
        return response
        
    def person_follow_deactivater_callback(self, request, response):
        """person_follow_deactiveサービスのコールバック"""
        self.person_follow_active = False
        
        # アクティブなナビゲーションゴールをキャンセル
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling current navigation goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            
        
        response.success = True
        response.message = "Person follow deactivated and navigation cancelled"
        return response
        
    def cancel_done_callback(self, future):
        """ナビゲーションキャンセル完了コールバック"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Navigation goal cancelled successfully')
        else:
            self.get_logger().info('Failed to cancel navigation goal')
        self.current_goal_handle = None
        
    def send_navigation_goal(self, goal_pose: PoseStamped):
        """NavigateToPoseアクションを送信"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigation action server not available, waiting...')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = self.behavior_tree_xml
        
        self.get_logger().info('Sending navigation goal...')
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """ナビゲーションゴールのレスポンスコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal_handle = None
            return
            
        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """ナビゲーション結果のコールバック"""
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation completed')
            self.navigation_cancelled = False
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {str(e)}')
            self.navigation_cancelled = True
        finally:
            self.current_goal_handle = None

    def person_info_callback(self, msg: PersonInfo):
        """PersonInfoメッセージを受信してgoalupdateトピックに変換"""
        if not self.person_follow_active:
            return
            
        try:
            # PersonInfoから位置情報を取得（ロボット基準の相対位置）
            person_x = msg.x/1000
            person_y = msg.y/1000
            person_distance = msg.distance
            
            self.get_logger().info(
                f'Received PersonInfo: x={person_x:.2f}, y={person_y:.2f}, distance={person_distance:.2f}'
            )
            
            # ロボットの現在位置をmap座標系で取得
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,           # target frame (map)
                    self.robot_frame_id,     # source frame (base_link)
                    rclpy.time.Time(),       # 最新の変換
                    rclpy.duration.Duration(seconds=1.0)  # タイムアウト
                )
            except TransformException as ex:
                self.get_logger().error(f'Could not transform {self.robot_frame_id} to {self.frame_id}: {ex}')
                return
            
            # ロボットの現在位置と姿勢を取得
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_z = transform.transform.translation.z
            
            # ロボットの現在の向き（yaw）を取得
            from tf_transformations import euler_from_quaternion
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            _, _, robot_yaw = euler_from_quaternion(quaternion)
            
            # PersonInfoの相対位置をmap座標系に変換
            # ロボット基準の相対位置をmap座標系の絶対位置に変換
            person_map_x = robot_x + person_x * math.cos(robot_yaw) - person_y * math.sin(robot_yaw)
            person_map_y = robot_y + person_x * math.sin(robot_yaw) + person_y * math.cos(robot_yaw)
            self.get_logger().info(
                f'Robot position in base_link: x={robot_x:.2f}, y={robot_y:.2f}, z={robot_z:.2f}, yaw={math.degrees(robot_yaw):.1f}deg'
            )
            self.get_logger().info(
                f'Robot position in map: x={robot_x:.2f}, y={robot_y:.2f}, yaw={math.degrees(robot_yaw):.1f}deg'
            )
            self.get_logger().info(
                f'Person position in map: x={person_map_x:.2f}, y={person_map_y:.2f}'
            )
            
            # 人の方向への角度を計算（map座標系での角度）
            angle_to_person = math.atan2(person_map_y - robot_y, person_map_x - robot_x)
            
            # 人に近づく目標位置を計算
            # approach_distance分手前の位置を目標とする
            target_distance = max(0.0, person_distance - self.approach_distance)
            
            # 目標位置の計算（map座標系）
            goal_x = person_map_x
            goal_y = person_map_y
            
            # 人の方向を向く角度を計算（approach_angleのオフセットを追加）
            goal_yaw = angle_to_person + self.approach_angle
            
            # PoseStampedメッセージの作成
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = self.frame_id
            
            # 位置の設定
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            
            # 姿勢の設定（quaternion）
            quaternion = quaternion_from_euler(0, 0, goal_yaw)
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]
            
            # 最新のgoal_poseを保存
            self.latest_goal_pose = goal_pose
            # 初回実行時のみNavigateToPoseアクションを送信（以降はタイマーで送信）
            if self.first_execution:
                self.send_navigation_goal(goal_pose)
                self.first_execution = False
            
            self.get_logger().info(
                f'Updated goal in map frame: x={goal_x:.2f}, y={goal_y:.2f}, yaw={math.degrees(goal_yaw):.1f}deg'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in person_info_callback: {str(e)}')


def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)
    
    try:
        node = PersonToGoalUpdate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()