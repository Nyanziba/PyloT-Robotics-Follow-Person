# PyLoT-Robotics-Follow-Person

人物追従機能を提供するROS2パッケージです。RoboCup@Home の Help Me Carry タスクなどで使用します。

## システム概要

```mermaid
flowchart TB
    subgraph Hardware["ハードウェア"]
        RS[RealSense D455]
        DXL[Dynamixel サーボ]
    end

    subgraph realsense_track["realsense_track.py"]
        RGBD[RGBD受信]
        YOLO[YOLOv8 人物検出]
        PID[PID制御]
        COORD[座標変換]
    end

    subgraph person_to_goal["person_to_goalupdate.py"]
        TF[TF2座標変換]
        GOAL[目標位置計算]
        NAV[Nav2アクション]
    end

    subgraph Nav2["Nav2 ナビゲーション"]
        NTP[NavigateToPose]
    end

    RS -->|/d455/camera/rgbd| RGBD
    RGBD --> YOLO
    YOLO --> PID
    PID -->|DxlCommandsX| DXL
    YOLO --> COORD
    COORD -->|/person_info| TF
    TF --> GOAL
    GOAL --> NAV
    NAV -->|NavigateToPose Action| NTP
```

## ノード詳細

### 1. realsense_track.py

RealSenseカメラで人物を検出・追跡し、パンチルトサーボで追従するノードです。

```mermaid
sequenceDiagram
    participant RS as RealSense
    participant RT as realsense_track
    participant YOLO as YOLOv8
    participant DXL as Dynamixel
    participant PI as /person_info

    RS->>RT: RGBD画像
    RT->>YOLO: 人物検出
    YOLO-->>RT: BBox + ID
    RT->>RT: 最近傍人物選択
    RT->>RT: PID制御計算
    RT->>DXL: サーボ角度指令
    RT->>RT: 3D座標変換
    RT->>PI: PersonInfo Publish
```

#### トピック・サービス

| 種別 | 名前 | 型 | 説明 |
|------|------|-----|------|
| Sub | `/d455/camera/rgbd` | `RGBD` | RGB+深度画像 |
| Sub | `/d455/camera/color/camera_info` | `CameraInfo` | カメラ光学パラメータ |
| Pub | `/person_info` | `PersonInfo` | 人物位置情報 |
| Pub | `/dynamixel/commands/x` | `DxlCommandsX` | サーボ制御指令 |
| Srv | `/start_person_tracking` | `Trigger` | 追跡開始 |
| Srv | `/stop_person_tracking` | `Trigger` | 追跡停止 |

#### 座標変換

```mermaid
flowchart LR
    subgraph Camera["カメラ座標系"]
        C_X["x: 右方向"]
        C_Y["y: 下方向"]
        C_Z["z: 前方向"]
    end

    subgraph Robot["ロボット座標系"]
        R_X["x: 前方向"]
        R_Y["y: 左方向"]
    end

    Camera -->|"モーター回転考慮"| Robot
```

**変換式:**
```
base_x = z_3d (カメラのz → ロボットのx)
base_y = -x_3d (カメラのx反転 → ロボットのy)

robot_x = base_x * cos(θ) + base_y * sin(θ)
robot_y = -base_x * sin(θ) + base_y * cos(θ)
```

---

### 2. person_to_goalupdate.py

`PersonInfo` を受信し、Nav2のナビゲーション目標に変換するノードです。

```mermaid
sequenceDiagram
    participant PI as /person_info
    participant PTG as person_to_goalupdate
    participant TF as TF2
    participant Nav as Nav2

    PI->>PTG: PersonInfo受信
    PTG->>TF: base_link→map変換要求
    TF-->>PTG: Transform
    PTG->>PTG: map座標系に変換
    PTG->>PTG: approach_distance考慮
    PTG->>Nav: NavigateToPose Action
    
    loop タイマー (0.5秒周期)
        PTG->>Nav: goal_update Publish
    end
```

#### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `person_topic` | string | `/person_info` | 入力トピック |
| `frame_id` | string | `map` | 目標座標系 |
| `robot_frame_id` | string | `base_link` | ロボット座標系 |
| `approach_distance` | double | `0.5` | 人物への接近距離 [m] |
| `timer_period` | double | `0.5` | 目標更新周期 [秒] |

#### 座標変換処理

```mermaid
flowchart TD
    A[PersonInfo<br/>ロボット相対座標] --> B[TF2 lookup_transform]
    B --> C[ロボット位置・姿勢取得]
    C --> D[map座標系に変換]
    D --> E[approach_distance<br/>を考慮した目標位置]
    E --> F[NavigateToPose Goal]
```

**変換式:**
```
person_map_x = robot_x + person_x * cos(yaw) - person_y * sin(yaw)
person_map_y = robot_y + person_x * sin(yaw) + person_y * cos(yaw)
```

---

## 使用方法

### 起動

```bash
# 人物追跡ノードを起動
ros2 run pylot_utils realsense_track

# ナビゲーション連携ノードを起動
ros2 run pylot_utils person_to_goalupdate
```

### サービス呼び出し

```bash
# 追跡開始
ros2 service call /start_person_tracking std_srvs/srv/Trigger

# 追跡停止
ros2 service call /stop_person_tracking std_srvs/srv/Trigger

# 人物追従ナビゲーション開始
ros2 service call /person_follow_active std_srvs/srv/Trigger

# 人物追従ナビゲーション停止
ros2 service call /person_follow_deactive std_srvs/srv/Trigger
```

---

## 依存関係

### 必須ライブラリ
- `ultralytics` (YOLOv8)
- `opencv-python`
- `numpy`
- `tf_transformations`

### ROS2パッケージ
- `realsense2_camera_msgs`
- `pylot_msgs`
- `dynamixel_handler_msgs`
- `nav2_msgs`
- `tf2_ros`

---

## ステートマシン

```mermaid
stateDiagram-v2
    [*] --> Idle: ノード起動
    
    Idle --> Tracking: /start_person_tracking
    Tracking --> Idle: /stop_person_tracking
    
    state Tracking {
        [*] --> Detecting
        Detecting --> PersonFound: 人物検出
        PersonFound --> ServoControl: PID計算
        ServoControl --> Publishing: PersonInfo作成
        Publishing --> Detecting: 次フレーム
    }
```

---

## 動画

[デモ動画リンク]


