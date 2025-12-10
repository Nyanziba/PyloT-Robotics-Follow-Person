import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image, CameraInfo
from ultralytics import YOLO
from ultralytics.engine.results import Results
from typing import List
from std_msgs.msg import String
from dynamixel_handler_msgs.msg import DxlCommandsX
from pylot_msgs.msg import PersonInfo
from std_srvs.srv import Trigger
import time
import math


y_max = 1280

min_depth, max_depth = 100, 8000 # mm
integral_y_error = 0
angle_y = 0
integral_x_error = 0
angle_x = 0
last_error_y = 0
last_error_x = 0


class RsSub(Node):
    def __init__(self):
        super().__init__('realsense_track')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/d455/camera/rgbd', self.rgbd_callback, 10)
        
        # カメラ情報の購読者を追加
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, 
            '/d455/camera/color/camera_info', 
            self.camera_info_callback, 
            10
        )
        
        self.pub_personinfo = self.create_publisher(PersonInfo, '/person_info', 10)
        self.timer = None
        self.model = YOLO('/home/pylot/Downloads/yolov8m.pt')
        self.depth_image = None
        self.rgb_image = None
        
        # カメラの光学パラメータ
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.fx = None  # 焦点距離 x
        self.fy = None  # 焦点距離 y
        self.cx = None  # 主点 x
        self.cy = None  # 主点 y
        self.image_width = 640   # デフォルト値
        self.image_height = 480  # デフォルト値
        self.camera_info_received = False
        
        self.integral_y_error = 0
        self.angle_y = 0
        self.integral_x_error = 0
        self.angle_x = 0
        self.last_error_y = 0
        self.last_error_x = 0.0
        self.GoalAngle = 0.0
        self.last_goal_angle = 0.0
        self.last_time = time.time()
        self.is_running = False

        self.publisher_ = self.create_publisher(
            DxlCommandsX, 
            '/dynamixel/commands/x',
            10)
        self.dxl_subscription = self.create_subscription(
            DxlCommandsX,
            '/dynamixel/state/present',
            self.dxl_callback,
            10)
        
        # サービスの作成
        self.start_service = self.create_service(
            Trigger,
            '/start_person_tracking',
            self.start_tracking_callback
        )
        self.stop_service = self.create_service(
            Trigger,
            '/stop_person_tracking',
            self.stop_tracking_callback
        )
        
        self.get_logger().info('人追跡サービスが起動しました。start_person_trackingサービスを呼び出して開始してください。')

    def start_tracking_callback(self, request, response):
        if not self.is_running:
            self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hzで実行
            self.is_running = True
            self.get_logger().info('人追跡を開始しました')
            response.success = True
            response.message = "人追跡を開始しました"
        else:
            response.success = False
            response.message = "既に人追跡が実行中です"
        return response

    def stop_tracking_callback(self, request, response):
        if self.is_running:
            if self.timer is not None:
                self.timer.destroy()
                self.timer = None
            self.is_running = False
            self.get_logger().info('人追跡を停止しました')
            response.success = True
            response.message = "人追跡を停止しました"
        else:
            response.success = False
            response.message = "人追跡は既に停止しています"
        return response
        
    
    def camera_info_callback(self, msg):
        """カメラ情報を受信して光学パラメータを設定"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            
            # 焦点距離と主点を取得
            self.fx = msg.k[0]  # K[0,0]
            self.fy = msg.k[4]  # K[1,1]
            self.cx = msg.k[2]  # K[0,2]
            self.cy = msg.k[5]  # K[1,2]
            
            # 画像サイズ
            self.image_width = msg.width
            self.image_height = msg.height
            
            self.camera_info_received = True
            self.get_logger().info(f'カメラ情報を受信: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')
            self.get_logger().info(f'画像サイズ: {self.image_width}x{self.image_height}')
    
    
    def pixel_to_3d_point(self, u, v, depth_mm):
        """ピクセル座標と深度から3D座標を計算
        Args:
            u, v: ピクセル座標
            depth_mm: 深度値 (mm)
        Returns:
            x, y, z: カメラ座標系での3D座標 (mm)
        """
        if not self.camera_info_received:
            self.get_logger().warn('カメラ情報がまだ受信されていません')
            return None, None, None
            
        if depth_mm <= 0:
            return None, None, None
            
        # ピクセル座標からカメラ座標系への変換
        x = (u - self.cx) * depth_mm / self.fx
        y = (v - self.cy) * depth_mm / self.fy
        z = depth_mm
        
        return x, y, z
    
    
    def get_3d_position_from_bbox(self, x1, y1, x2, y2, depth_image):
        """バウンディングボックスから3D位置を計算
        Args:
            x1, y1, x2, y2: バウンディングボックス座標
            depth_image: 深度画像
        Returns:
            x, y, z: 3D座標 (mm), 有効でない場合はNone
        """
        # 座標を画像サイズ内にクリップ
        x1 = max(0, min(x1, depth_image.shape[1] - 1))
        x2 = max(0, min(x2, depth_image.shape[1] - 1))
        y1 = max(0, min(y1, depth_image.shape[0] - 1))
        y2 = max(0, min(y2, depth_image.shape[0] - 1))
        
        # バウンディングボックスの中心点
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        
        # 深度値の取得（中心点周辺の平均を使用してノイズ軽減）
        roi_size = 5  # 中心点周辺のROIサイズ
        roi_x1 = max(0, cx - roi_size // 2)
        roi_x2 = min(depth_image.shape[1], cx + roi_size // 2 + 1)
        roi_y1 = max(0, cy - roi_size // 2)
        roi_y2 = min(depth_image.shape[0], cy + roi_size // 2 + 1)
        
        depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_depths = depth_roi[(depth_roi > min_depth) & (depth_roi < max_depth)]
        
        if len(valid_depths) == 0:
            return None, None, None
            
        depth_mm = np.median(valid_depths)  # 中央値を使用
        
        # 3D座標を計算
        return self.pixel_to_3d_point(cx, cy, depth_mm)
    
    def calculate_angle_from_center(self, x_3d, z_3d):
        """カメラ中心からの水平角度を計算
        Args:
            x_3d, z_3d: カメラ座標系での3D座標
        Returns:
            angle: 角度（度）
        """
        if z_3d <= 0:
            return 0
        return math.degrees(math.atan2(x_3d, z_3d))
        
    def dxl_callback(self, msg):
        self.last_goal_angle = float(msg.position_deg[1])
        self.get_logger().info(f'last_goal_angle: {self.last_goal_angle}')

    def rgbd_callback(self, msg):
        # RGBDデータをバッファリング
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        
    def timer_callback(self):
        if not self.is_running:
            return
            
        if self.depth_image is None or self.rgb_image is None:
            return

        # 画像サイズの確認とログ出力
        depth_shape = self.depth_image.shape
        rgb_shape = self.rgb_image.shape
        
        if not hasattr(self, '_shape_logged'):
            self.get_logger().info(f'深度画像サイズ: {depth_shape}, RGB画像サイズ: {rgb_shape}')
            self._shape_logged = True

        around_person = False

        #PID制御用のパラメータ
        current_time = time.time()
        time_interval = current_time - self.last_time
        self.last_time = current_time
        # X軸（水平方向）のPIDパラメータ
        Kp_x = 5.0
        Ki_x = 0.3
        Kd_x = 1.6

        Dxl_cmd = DxlCommandsX()

        results: List[Results] = self.model.track(self.rgb_image, persist=True, classes=[0])
        if len(results) == 0:
            return
        
        if results[0].boxes is None:
            return
        
        if len(results[0].boxes) == 0:
            return

        distance = {}

        for result in results:
            if around_person:
                continue
            
            try:
                boxes = result.boxes.cpu().numpy()
                
                if boxes.id is None or len(boxes.xyxy) == 0:
                    continue
                    
                # IDとxyxyの配列長を確認
                if len(boxes.id) == 0 or len(boxes.xyxy) == 0:
                    continue
                
                id = int(boxes.id[0])
                
                # YOLOのxyxy形式: [x1, y1, x2, y2] (左上と右下の座標)
                xyxy_raw = boxes.xyxy[0][:4]
                x1, y1, x2, y2 = map(int, xyxy_raw)
                
                # バウンディングボックスの座標を正しい順序に修正
                x_min, x_max = min(x1, x2), max(x1, x2)
                y_min, y_max = min(y1, y2), max(y1, y2)
                x1, y1, x2, y2 = x_min, y_min, x_max, y_max
                
                # バウンディングボックスの有効性チェック
                if x1 >= x2 or y1 >= y2:
                    self.get_logger().warn(f'無効なバウンディングボックス: [{x1}, {y1}, {x2}, {y2}]')
                    continue
                
                # 座標の妥当性をチェック
                if x1 < 0 or y1 < 0 or x2 < 0 or y2 < 0:
                    self.get_logger().warn(f'負の座標値: [{x1}, {y1}, {x2}, {y2}]')
                    continue
                
                # 座標値が異常に大きくないかチェック
                if x1 > 10000 or y1 > 10000 or x2 > 10000 or y2 > 10000:
                    self.get_logger().warn(f'異常に大きな座標値: [{x1}, {y1}, {x2}, {y2}]')
                    continue
            
            except (IndexError, ValueError, AttributeError) as e:
                self.get_logger().warn(f'YOLO結果の処理中にエラーが発生しました: {str(e)}')
                continue
            
            # 画像範囲外の検出を警告
            if (x1 < 0 or x2 >= self.depth_image.shape[1] or 
                y1 < 0 or y2 >= self.depth_image.shape[0]):
                self.get_logger().warn(f'画像範囲外の検出: bbox=[{x1}, {y1}, {x2}, {y2}], 画像サイズ={self.depth_image.shape}')
            
            # バウンディングボックスの座標を画像サイズ内にクリップ
            x1 = max(0, min(x1, self.depth_image.shape[1] - 1))
            x2 = max(0, min(x2, self.depth_image.shape[1] - 1))
            y1 = max(0, min(y1, self.depth_image.shape[0] - 1))
            y2 = max(0, min(y2, self.depth_image.shape[0] - 1))
            
            # 3D座標を計算して距離を取得
            x_3d, y_3d, z_3d = self.get_3d_position_from_bbox(x1, y1, x2, y2, self.depth_image)
            if z_3d is not None:
                distance[id] = z_3d  # 距離として深度値を使用
                print(f"ID: {id}, bbox: [{x1}, {y1}, {x2}, {y2}], 3D: [{x_3d:.1f}, {y_3d:.1f}, {z_3d:.1f}]mm")
            else:
                # フォールバック: 中心点の深度値を使用（境界チェック付き）
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cx = max(0, min(cx, self.depth_image.shape[1] - 1))
                cy = max(0, min(cy, self.depth_image.shape[0] - 1))
                fallback_depth = self.depth_image[int(cy)][int(cx)]
                if min_depth < fallback_depth < max_depth:
                    distance[id] = fallback_depth
                    print(f"ID: {id}, フォールバック深度: {fallback_depth}mm")

        if len(distance) == 0:
            return
        decide_person = min(distance, key=distance.get)

        # PanTilt Control
        for result in results:
            around_person = True
            try:
                boxes = result.boxes.cpu().numpy()
                names = result.names
                if len(boxes.xyxy) == 0 or boxes.id is None or len(boxes.id) == 0:
                    continue
                id = int(boxes.id[0])
                if id == decide_person:
                    if distance[id] == 0:
                        continue
                        
                    x1, y1, x2, y2 = map(int, boxes.xyxy[0][:4])
                    
                    # 座標の順序を確認し修正
                    x1, x2 = min(x1, x2), max(x1, x2)
                    y1, y2 = min(y1, y2), max(y1, y2)
                    
                    # 座標を画像サイズ内にクリップ
                    x1 = max(0, min(x1, self.depth_image.shape[1] - 1))
                    x2 = max(0, min(x2, self.depth_image.shape[1] - 1))
                    y1 = max(0, min(y1, self.depth_image.shape[0] - 1))
                    y2 = max(0, min(y2, self.depth_image.shape[0] - 1))
                    
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # カメラ情報を使用した画面中心からの誤差計算
                    if self.camera_info_received:
                        error_y = cy - self.cy  # カメラの主点を基準に
                        error_x = cx - self.cx
                    else:
                        # フォールバック: 固定値を使用
                        error_y = cy - 320
                        error_x = cx - 320
                    
                    print(f"画像中心からの誤差: error_x={error_x}, error_y={error_y}")
                    
                    
                    # 水平視野角を計算 (FOV = 2 * arctan(width / (2 * fx)))
                    horizontal_fov = 2 * math.atan(self.image_width / (2 * self.fx))
                    angle_per_pixel = horizontal_fov / self.image_width
                    
                    
                    # PID制御による目標角設定
                    self.integral_x_error += error_x * time_interval
                    derivative_x = (error_x - self.last_error_x) / time_interval if time_interval > 0 else 0
                    
                    angle_x_output = (
                        Kp_x * error_x +
                        Ki_x * self.integral_x_error +
                        Kd_x * derivative_x
                    )
                    
                    print(f"PID制御: P={Kp_x * error_x:.3f}, I={Ki_x * self.integral_x_error:.3f}, D={Kd_x * derivative_x:.3f}, 出力={angle_x_output:.3f}")
                    
                    # 角度を-180度から+180度の範囲に正規化+制限
                    target_angle_offset = angle_x_output * angle_per_pixel
                    self.GoalAngle += target_angle_offset
                    self.GoalAngle = max(-180, min(180, self.GoalAngle))
                    self.last_error_x = error_x


                    Dxl_cmd.position_control.id_list = [2]
                    Dxl_cmd.position_control.position_deg.append(self.GoalAngle)
                    Dxl_cmd.position_control.profile_vel_deg_s.append(20000)
                    Dxl_cmd.position_control.profile_acc_deg_ss.append(20000)
                    self.publisher_.publish(Dxl_cmd)


                    # 3D座標を使用してPersonInfoを公開
                    x_3d, y_3d, z_3d = self.get_3d_position_from_bbox(x1, y1, x2, y2, self.depth_image)
                    
                    person_info = PersonInfo()
                    if x_3d is not None and y_3d is not None and z_3d is not None:
                        # カメラ座標系からロボット座標系への変換（モーター回転角度を考慮）
                        # カメラ座標系: x=右、y=下、z=前
                        # ロボット座標系: x=前、y=左（モーター回転角度を考慮）
                        
                        # モーター角度をロボット座標系の角度に変換 (度単位)
                        motor_angle_deg = self.last_goal_angle
                        
                        # カメラ座標系での3D位置からロボット座標系への基本変換
                        base_x = float(z_3d)  # カメラのz軸がロボットのx軸（前方）
                        base_y = float(-x_3d)  # カメラのx軸の逆がロボットのy軸（左）
                        
                        # モーター回転を考慮した座標変換
                        motor_angle_rad = math.radians(motor_angle_deg)
                        person_info.x = base_x * math.cos(motor_angle_rad) + base_y * math.sin(motor_angle_rad)
                        person_info.y = -base_x * math.sin(motor_angle_rad) + base_y * math.cos(motor_angle_rad)
                        person_info.distance = float(z_3d)
                        person_info.motor_degree = float(motor_angle_deg)
                        
                        self.get_logger().info(f'3D位置: カメラ座標({x_3d:.1f}, {y_3d:.1f}, {z_3d:.1f}), 基本ロボット座標({base_x:.1f}, {base_y:.1f})')
                        self.get_logger().info(f'モーター角度: {motor_angle_deg:.1f}°, 最終ロボット座標({person_info.x:.1f}, {person_info.y:.1f})')
                    else:
                        # フォールバック: モーター回転角度を考慮した計算
                        # カメラ座標系からロボット座標系への変換（モーター回転込み）
                        # カメラ座標系: x=右、y=下、z=前
                        # ロボット座標系: x=前、y=左（モーター回転角度を考慮）
                        person_info.distance = float(distance[id])
                        
                        # モーター角度をロボット座標系の角度に変換 (度単位)
                        motor_angle_deg = self.last_goal_angle 
                        
                        # カメラ視野内の角度偏差 (度単位、左右方向)
                        camera_angle_offset = error_y / 640 * 90
                        
                        # 合成角度 (ロボット前方を0度とする)
                        total_angle_deg = motor_angle_deg + camera_angle_offset
                        
                        # ロボット座標系での位置計算 (x: 前方, y: 左方向)
                        person_info.x = math.cos(math.radians(total_angle_deg)) * distance[id]
                        person_info.y = math.sin(math.radians(total_angle_deg)) * distance[id]
                        person_info.motor_degree = float(motor_angle_deg)
                        # デバッグ情報
                        self.get_logger().info(f'角度計算 - モーター角度: {motor_angle_deg:.1f}°, カメラオフセット: {camera_angle_offset:.1f}°, 合成角度: {total_angle_deg:.1f}°')
                        self.get_logger().info(f'フォールバック座標: ロボット座標({person_info.x:.1f}, {person_info.y:.1f}), 距離: {person_info.distance:.1f}mm')
                    if self.GoalAngle > 180:
                        self.GoalAngle =180
                    elif self.GoalAngle < -180:
                        self.GoalAngle = -180
                    self.pub_personinfo.publish(person_info)
                    self.last_goal_angle = float(self.GoalAngle)
            
            except (IndexError, ValueError, AttributeError) as e:
                self.get_logger().warn(f'人追跡の処理中にエラーが発生しました: {str(e)}')
                continue

    def mask_rgb(self, rgb, depth) -> np.ndarray:
        mask = (depth <= min_depth) | (depth >= max_depth)
        return np.where(np.broadcast_to(mask[:, :, None], rgb.shape), 0, rgb).astype(np.uint8)

def main(args=None):
    rclpy.init(args=args)
    server = RsSub()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()