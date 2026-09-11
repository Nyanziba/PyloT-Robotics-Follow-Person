# PyLoT-Robotics-Follow-Person アーキテクチャ解説

本ドキュメントでは、人物追従システムのロジックと実装上の工夫について詳しく解説します。

---

## 目次

1. [システム全体像](#システム全体像)
2. [realsense_track.py の詳細ロジック](#realsense_trackpy-の詳細ロジック)
3. [person_to_goalupdate.py の詳細ロジック](#person_to_goalupdatepy-の詳細ロジック)
4. [実装上の工夫](#実装上の工夫)

---

## システム全体像

```mermaid
graph TB
    subgraph Perception["知覚層"]
        RS[RealSense D455<br/>RGB-D カメラ]
        YOLO[YOLOv8<br/>人物検出]
    end

    subgraph Processing["処理層"]
        RT[realsense_track<br/>人物追跡・位置推定]
        PTG[person_to_goalupdate<br/>座標変換・目標生成]
    end

    subgraph Control["制御層"]
        SERVO[Dynamixel サーボ<br/>パン制御]
        NAV2[Nav2<br/>自律移動]
    end

    RS --> YOLO
    YOLO --> RT
    RT --> SERVO
    RT --> PTG
    PTG --> NAV2

    style Perception fill:#e1f5fe
    style Processing fill:#fff3e0
    style Control fill:#e8f5e9
```

### データフロー概要

```mermaid
flowchart LR
    A[RGBD画像] --> B[YOLO検出]
    B --> C{複数人物?}
    C -->|Yes| D[最近傍選択]
    C -->|No| E[単一人物]
    D --> F[3D位置推定]
    E --> F
    F --> G[PID制御]
    G --> H[サーボ指令]
    F --> I[座標変換]
    I --> J[Nav2目標]
```

---

## realsense_track.py の詳細ロジック

### 1. 人物検出と追跡

```mermaid
flowchart TD
    subgraph Detection["検出フェーズ"]
        A[RGB画像入力] --> B[YOLOv8 track]
        B --> C[BBox + Track ID]
    end

    subgraph Selection["選択フェーズ"]
        C --> D[各人物の深度取得]
        D --> E[距離計算]
        E --> F{最小距離の人物}
    end

    subgraph Output["出力フェーズ"]
        F --> G[追跡対象決定]
        G --> H[3D座標計算]
        H --> I[PersonInfo Publish]
    end
```

#### ロジック詳細

**YOLOv8のトラッキング機能を活用**

```python
results = self.model.track(self.rgb_image, persist=True, classes=[0])
```

- `persist=True`: フレーム間でIDを維持（同一人物追跡）
- `classes=[0]`: 人物クラスのみ検出

**最近傍人物選択アルゴリズム**

```mermaid
flowchart TD
    A[検出された全人物] --> B[各人物のBBox中心]
    B --> C[深度画像から距離取得]
    C --> D[距離辞書作成<br/>distance = ID: depth]
    D --> E[min関数で最小距離ID取得]
    E --> F[decide_person = 追跡対象]
```

### 2. 3D位置推定

```mermaid
flowchart LR
    subgraph Input["入力"]
        A[ピクセル座標 u, v]
        B[深度値 d]
        C[カメラパラメータ<br/>fx, fy, cx, cy]
    end

    subgraph Calc["計算"]
        D["x = (u - cx) × d / fx"]
        E["y = (v - cy) × d / fy"]
        F["z = d"]
    end

    subgraph Output["出力"]
        G[カメラ座標系<br/>3D位置]
    end

    A --> D
    B --> D
    B --> E
    B --> F
    C --> D
    C --> E
    D --> G
    E --> G
    F --> G
```

#### 深度値取得の工夫

```mermaid
flowchart TD
    A[BBox中心点] --> B[5×5 ROI抽出]
    B --> C[有効深度フィルタ<br/>100mm < d < 8000mm]
    C --> D{有効値あり?}
    D -->|Yes| E[中央値を採用]
    D -->|No| F[フォールバック:<br/>中心点の直接値]
    E --> G[深度値確定]
    F --> G
```

**ノイズ耐性の向上**
- 単一ピクセルではなく5×5領域の中央値を使用
- 異常値（100mm未満、8000mm超）を除外
- フォールバック処理で検出失敗を防止

### 3. PID制御によるサーボ追従

```mermaid
flowchart TD
    subgraph Error["誤差計算"]
        A[人物BBox中心] --> B[画像中心との差分]
        B --> C[error_x = cx - image_center_x]
    end

    subgraph PID["PID演算"]
        C --> D[P項: Kp × error]
        C --> E[I項: Ki × ∫error dt]
        C --> F[D項: Kd × d_error/dt]
        D --> G[angle_output]
        E --> G
        F --> G
    end

    subgraph Output["角度出力"]
        G --> H[FOVベースの角度変換]
        H --> I[GoalAngle更新]
        I --> J[Dynamixelコマンド]
    end
```

#### PIDパラメータ設計

| パラメータ | 値 | 役割 |
|-----------|-----|------|
| Kp | 5.0 | 追従速度の決定 |
| Ki | 0.3 | 定常偏差の除去 |
| Kd | 1.6 | オーバーシュート抑制 |

**角度変換の計算**

```python
horizontal_fov = 2 * math.atan(image_width / (2 * fx))
angle_per_pixel = horizontal_fov / image_width
target_angle = angle_output * angle_per_pixel
```

### 4. 座標変換（カメラ→ロボット）

```mermaid
flowchart LR
    subgraph Camera["カメラ座標系"]
        direction TB
        CX[X: 右方向]
        CY[Y: 下方向]
        CZ[Z: 前方向]
    end

    subgraph Base["基本変換"]
        direction TB
        BX["base_x = z_3d"]
        BY["base_y = -x_3d"]
    end

    subgraph Rotation["モーター回転考慮"]
        direction TB
        RX["x' = base_x×cos(θ) + base_y×sin(θ)"]
        RY["y' = -base_x×sin(θ) + base_y×cos(θ)"]
    end

    subgraph Robot["ロボット座標系"]
        direction TB
        OX[X: 前方向]
        OY[Y: 左方向]
    end

    Camera --> Base
    Base --> Rotation
    Rotation --> Robot
```

---

## person_to_goalupdate.py の詳細ロジック

### 1. TF2による座標変換

```mermaid
sequenceDiagram
    participant PI as PersonInfo
    participant TF as TF2 Buffer
    participant MAP as map座標系

    PI->>TF: lookup_transform(map, base_link)
    TF-->>PI: Transform(x, y, yaw)
    
    Note over PI: ロボット位置取得
    
    PI->>PI: 相対座標 → 絶対座標
    Note over PI: person_map_x = robot_x + person_x×cos(yaw) - person_y×sin(yaw)
    
    PI->>MAP: 目標位置計算
```

#### 座標変換の数学的表現

**回転行列による変換**

$$
\begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = 
\begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix} +
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} x_{rel} \\ y_{rel} \end{bmatrix}
$$

### 2. ナビゲーション目標管理

```mermaid
stateDiagram-v2
    [*] --> Idle: 起動
    
    Idle --> WaitingGoal: PersonInfo受信
    WaitingGoal --> SendingGoal: 初回実行
    SendingGoal --> GoalActive: Goal Accepted
    GoalActive --> WaitingGoal: Goal Complete
    GoalActive --> Idle: Deactivate
    
    state GoalActive {
        [*] --> Navigating
        Navigating --> Updating: タイマー発火
        Updating --> Navigating: goal_update Publish
    }
```

#### タイマーベースの目標更新

```mermaid
flowchart TD
    A[タイマー発火<br/>0.5秒周期] --> B{追従アクティブ?}
    B -->|No| C[スキップ]
    B -->|Yes| D{goal_pose存在?}
    D -->|No| C
    D -->|Yes| E{Nav2アクション中?}
    E -->|Yes| F[goal_update Publish]
    E -->|No| G[新規Nav2 Action送信]
```

---

## 実装上の工夫

### 1. ロバスト性向上

```mermaid
mindmap
  root((ロバスト性))
    深度処理
      中央値フィルタ
      有効範囲チェック
      フォールバック処理
    座標検証
      BBox有効性チェック
      座標範囲クリップ
      異常値検出
    エラーハンドリング
      try-except包括
      ログ出力
      状態復帰
```

#### 深度ノイズ対策

| 問題 | 対策 | 効果 |
|------|------|------|
| 単一ピクセルノイズ | 5×5 ROI + 中央値 | 外れ値の影響を軽減 |
| 無効深度値 | 100-8000mm範囲フィルタ | 測定限界外を除外 |
| 深度取得失敗 | フォールバック処理 | 追跡継続を保証 |

### 2. 処理効率化

```mermaid
flowchart TD
    subgraph Optimization["最適化ポイント"]
        A[RGBDコールバック] --> B[データバッファリングのみ]
        C[タイマーコールバック] --> D[実際の処理実行]
        E[サービス制御] --> F[処理の開始/停止切替]
    end
```

**分離設計の利点**
- RGBDコールバック: 軽量化（バッファリングのみ）
- タイマー: 20Hz固定レートで処理
- 処理落ち時もデータ欠損なし

### 3. 座標系の一貫性

```mermaid
flowchart LR
    subgraph Layer1["知覚層"]
        A[カメラ座標系<br/>x:右 y:下 z:前]
    end

    subgraph Layer2["処理層"]
        B[ロボット座標系<br/>x:前 y:左]
    end

    subgraph Layer3["ナビゲーション層"]
        C[map座標系<br/>TF2管理]
    end

    A -->|基本変換 + 回転| B
    B -->|TF2変換| C
```

#### 座標変換の段階的処理

1. **カメラ→ロボット基本変換**: 軸の入れ替え
2. **モーター回転補正**: パン角度を考慮
3. **ロボット→map変換**: TF2による絶対座標化

### 4. 動的目標追従

```mermaid
sequenceDiagram
    participant P as PersonInfo
    participant G as GoalManager
    participant N as Nav2

    loop 0.5秒周期
        P->>G: 位置更新
        G->>G: latest_goal_pose更新
        
        alt Nav2アクション中
            G->>N: goal_update Publish
            Note over N: 経路修正
        else アクション完了
            G->>N: 新規NavigateToPose
            Note over N: 新経路計画
        end
    end
```

**工夫ポイント**
- `goal_update` トピック: アクション中でも目標更新可能
- カスタムBehavior Tree: 動的目標に対応した経路計画
- タイマーベース更新: 安定した更新レート保証

### 5. サービスによる状態制御

```mermaid
flowchart TD
    subgraph Services["サービスインターフェース"]
        A[/start_person_tracking]
        B[/stop_person_tracking]
        C[/person_follow_active]
        D[/person_follow_deactive]
    end

    subgraph States["内部状態"]
        E[is_running]
        F[person_follow_active]
        G[current_goal_handle]
    end

    A --> E
    B --> E
    C --> F
    D --> F
    D --> G
```

**安全な停止処理**
```python
def person_follow_deactivater_callback(self, request, response):
    self.person_follow_active = False
    
    # アクティブなゴールをキャンセル
    if self.current_goal_handle is not None:
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
```

---

## まとめ

### 設計思想

```mermaid
mindmap
  root((設計思想))
    モジュール分離
      知覚と制御の分離
      トピックベース連携
      独立したテスト可能性
    ロバスト性
      多重フォールバック
      異常値フィルタリング
      エラーハンドリング
    リアルタイム性
      タイマーベース処理
      非同期コールバック
      軽量バッファリング
    拡張性
      パラメータ化
      サービスAPI
      標準メッセージ使用
```

### 今後の改善候補

1. **カルマンフィルタ導入**: 位置推定の平滑化
2. **複数人物追跡**: ID維持・切替ロジック
3. **障害物回避強化**: コストマップとの統合
4. **学習ベース最適化**: PIDゲインの自動調整

---

## 付録: ピンホールカメラモデルによる座標変換

本システムでは、2D画像座標から3D空間座標への変換にピンホールカメラモデルを使用しています。

### ピンホールカメラモデルとは

ピンホールカメラモデルは、3次元空間の点を2次元画像平面に投影する数学的モデルです。

```mermaid
flowchart LR
    subgraph World["3D空間"]
        P["点 (X, Y, Z)"]
    end
    
    subgraph Camera["カメラ"]
        O["光学中心"]
        F["焦点距離 f"]
    end
    
    subgraph Image["画像平面"]
        Q["ピクセル (u, v)"]
    end
    
    P --> O
    O --> Q
```

### 座標変換の数式

#### カメラ内部パラメータ（Intrinsic Matrix）

カメラの内部パラメータは以下の行列 $\mathbf{K}$ で表されます：

$$
\mathbf{K} = \begin{pmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}
$$

**パラメータの説明**

| パラメータ | 記号 | 説明 | 単位 |
|-----------|------|------|------|
| 焦点距離（X方向） | $f_x$ | X軸方向のピクセル単位焦点距離 | pixel |
| 焦点距離（Y方向） | $f_y$ | Y軸方向のピクセル単位焦点距離 | pixel |
| 光学中心（X） | $c_x$ | 画像主点のX座標 | pixel |
| 光学中心（Y） | $c_y$ | 画像主点のY座標 | pixel |
| 深度 | $Z$ | カメラからの距離 | meter |

#### 順方向変換（3D → 2D 投影）

3次元空間の点 $\mathbf{P} = (X, Y, Z)^T$ を画像平面上の点 $\mathbf{p} = (u, v)^T$ に投影します。

**同次座標系での表現：**

$$
\begin{pmatrix}
u' \\
v' \\
w
\end{pmatrix}
= \mathbf{K} \cdot
\begin{pmatrix}
X \\
Y \\
Z
\end{pmatrix}
=
\begin{pmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
X \\
Y \\
Z
\end{pmatrix}
$$

**正規化：**

$$
u = \frac{u'}{w} = f_x \cdot \frac{X}{Z} + c_x
$$

$$
v = \frac{v'}{w} = f_y \cdot \frac{Y}{Z} + c_y
$$

#### 逆変換（2D → 3D 復元）- 本システムで使用

深度 $Z$ が既知の場合、画像座標 $(u, v)$ から3D座標 $(X, Y, Z)$ を復元できます：

$$
\boxed{
X = \frac{(u - c_x) \cdot Z}{f_x}
}
$$

$$
\boxed{
Y = \frac{(v - c_y) \cdot Z}{f_y}
}
$$

**行列形式での逆変換：**

$$
\begin{pmatrix}
X \\
Y \\
Z
\end{pmatrix}
= Z \cdot \mathbf{K}^{-1}
\begin{pmatrix}
u \\
v \\
1
\end{pmatrix}
= Z \cdot
\begin{pmatrix}
\frac{1}{f_x} & 0 & -\frac{c_x}{f_x} \\
0 & \frac{1}{f_y} & -\frac{c_y}{f_y} \\
0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
u \\
v \\
1
\end{pmatrix}
$$

### 変換プロセスの詳細

```mermaid
flowchart TD
    subgraph Input["入力データ"]
        A[YOLO検出<br/>BBox中心座標<br/>"(u, v)"]
        B[深度画像<br/>Depth値<br/>"Z"]
        C[カメラ内部パラメータ<br/>"K"]
    end
    
    subgraph Transform["ピンホール逆変換"]
        D["X = (u - c_x) · Z / f_x"]
        E["Y = (v - c_y) · Z / f_y"]
    end
    
    subgraph Output["出力"]
        F["3D座標<br/>(X, Y, Z)"]
    end
    
    A --> D
    A --> E
    B --> D
    B --> E
    C --> D
    C --> E
    D --> F
    E --> F
```

### 実装コード

`realsense_track.py` での実装：

```python
def calculate_3d_position(self, center_x, center_y, depth):
    """
    2D画像座標と深度から3D空間座標を計算
    
    数式:
        X = (u - c_x) * Z / f_x
        Y = (v - c_y) * Z / f_y
    
    Args:
        center_x (u): BBox中心のX座標（ピクセル）
        center_y (v): BBox中心のY座標（ピクセル）
        depth (Z): 深度値（メートル）
    
    Returns:
        (X, Y, Z): カメラ座標系での3D位置
    """
    # カメラ内部パラメータの取得
    fx = self.camera_intrinsics.fx  # f_x: X方向焦点距離
    fy = self.camera_intrinsics.fy  # f_y: Y方向焦点距離
    cx = self.camera_intrinsics.ppx # c_x: 光学中心X
    cy = self.camera_intrinsics.ppy # c_y: 光学中心Y
    
    # ピンホールカメラモデルの逆変換
    x = (center_x - cx) * depth / fx  # X = (u - c_x) * Z / f_x
    y = (center_y - cy) * depth / fy  # Y = (v - c_y) * Z / f_y
    z = depth                          # Z = depth
    
    return x, y, z
```

### RealSense座標系

カメラ座標系 $\mathcal{C}$ は以下のように定義されます：

- $X_c$ 軸: 右方向（画像の横軸に対応）
- $Y_c$ 軸: 下方向（画像の縦軸に対応）
- $Z_c$ 軸: 前方（深度方向、カメラの光軸）

```mermaid
graph LR
    subgraph Camera["カメラ座標系 C"]
        direction TB
        O["原点 O<br/>(カメラ中心)"]
        X["X_c →<br/>右方向"]
        Y["Y_c ↓<br/>下方向"]
        Z["Z_c ⊙<br/>前方(深度)"]
    end
```

```
カメラ座標系の視覚化:

        Z_c (前方/深度)
         ↑
         │
         │
         ●───→ X_c (右方向)
        /
       /
      ↓
     Y_c (下方向)
```

### なぜこの変換が機能するのか

#### 数学的根拠

ピンホールカメラモデルにおいて、3D点 $\mathbf{P}$ と画像点 $\mathbf{p}$ の関係は：

$$
\lambda \begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{K} \begin{pmatrix} X \\ Y \\ Z \end{pmatrix}
$$

ここで $\lambda = Z$ はスケールファクターです。

RGB-Dカメラでは $Z$（深度）が直接測定できるため、逆変換が一意に決定されます：

$$
\begin{pmatrix} X \\ Y \\ Z \end{pmatrix} = Z \cdot \mathbf{K}^{-1} \begin{pmatrix} u \\ v \\ 1 \end{pmatrix}
$$

```mermaid
flowchart TB
    subgraph Key["成功の鍵"]
        A["RGB-Dカメラ<br/>深度 Z を直接測定"]
        B["キャリブレーション<br/>K が既知"]
        C["数学的一意性<br/>Z + (u,v) → (X,Y,Z)"]
    end
    
    A --> D["各ピクセルの<br/>深度値が利用可能"]
    B --> E["工場出荷時に<br/>較正済み"]
    C --> F["逆変換が<br/>一意に決定"]
    
    D --> G["正確な3D位置推定"]
    E --> G
    F --> G
```

### RealSense SDK との比較

本システムの手動実装と同等の機能が `pyrealsense2` に用意されています：

```python
# pyrealsense2 の関数を使用する場合
import pyrealsense2 as rs

point_3d = rs.rs2_deproject_pixel_to_point(
    intrinsics,              # カメラ内部パラメータ
    [center_x, center_y],    # 2D画像座標
    depth                    # 深度値
)
# point_3d = [X, Y, Z]
```

**手動実装のメリット**
- 処理の透明性（デバッグしやすい）
- カスタマイズ可能（フィルタリング等を追加しやすい）
- 依存関係の軽減

### 座標変換チェーン全体

本システムでの座標変換の全体像：

```mermaid
flowchart LR
    subgraph S1["Step 1: 2D検出"]
        A["YOLO BBox<br/>(u, v)"]
    end
    
    subgraph S2["Step 2: 深度取得"]
        B["Depth Map<br/>Z"]
    end
    
    subgraph S3["Step 3: ピンホール逆変換"]
        C["カメラ座標<br/>(X_c, Y_c, Z_c)"]
    end
    
    subgraph S4["Step 4: ロボット座標変換"]
        D["ロボット座標<br/>(X_r, Y_r)"]
    end
    
    subgraph S5["Step 5: TF2変換"]
        E["マップ座標<br/>(X_m, Y_m)"]
    end
    
    A --> C
    B --> C
    C --> D
    D --> E
```

#### カメラ→ロボット座標変換（Step 4）

カメラ座標系 $\mathcal{C}$ からロボット座標系 $\mathcal{R}$ への変換を行います。

**座標系の定義：**
- カメラ座標系 $\mathcal{C}$: $X_c$（右）, $Y_c$（下）, $Z_c$（前）
- ロボット座標系 $\mathcal{R}$: $X_r$（前）, $Y_r$（左）

**モーター回転を考慮した変換：**

パンモーターの現在角度を $\theta$ [rad] とすると：

$$
\boxed{
\begin{aligned}
X_r &= Z_c \cos\theta - X_c \sin\theta \\
Y_r &= -(Z_c \sin\theta + X_c \cos\theta)
\end{aligned}
}
$$

**回転行列による表現：**

$$
\begin{pmatrix}
X_r \\
Y_r
\end{pmatrix}
=
\begin{pmatrix}
\cos\theta & -\sin\theta \\
-\sin\theta & -\cos\theta
\end{pmatrix}
\begin{pmatrix}
Z_c \\
X_c
\end{pmatrix}
$$

```python
# カメラ座標系: X_c(右), Y_c(下), Z_c(前)
# ロボット座標系: X_r(前), Y_r(左)

import math

# モーター回転を考慮した変換
theta = motor_angle_rad  # パンモーターの角度 [rad]

x_robot = z_camera * math.cos(theta) - x_camera * math.sin(theta)
y_robot = -(z_camera * math.sin(theta) + x_camera * math.cos(theta))
```

#### TF2によるマップ座標変換（Step 5）

ロボット座標系 $\mathcal{R}$ からマップ座標系 $\mathcal{M}$ への変換は、TF2を用いて行います：

$$
\mathbf{P}_{\mathcal{M}} = \mathbf{T}_{\mathcal{R}}^{\mathcal{M}} \cdot \mathbf{P}_{\mathcal{R}}
$$

ここで $\mathbf{T}_{\mathcal{R}}^{\mathcal{M}}$ は `base_link` から `map` への変換行列です。
