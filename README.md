# zlac8015d_hardware

ZLAC8015D モータドライバを ROS 2 の `ros2_control` から操作するための **SocketCAN / CANopen ハードウェアインターフェース**です。対向二輪型移動ロボットの左右駆動輪に対し、車輪角速度指令を受け取り、エンコーダ位置・実測角速度・モータ電流を `ros2_control` に提供します。

ドライバ内蔵の速度制御を使う `velocity` モードと、左右の車輪に独立した一次 LADRC（線形拡張状態オブザーバを含む）を適用して電流を指令する `ladrc_torque` モードを実装しています。

## 動作環境と依存関係

- Ubuntu 22.04 / ROS 2 Humble
- ZLAC8015D モータドライバおよび対応する CAN インターフェース（Linux SocketCAN）
- C++17 / `ament_cmake`
- ROS 2 パッケージ：`rclcpp`、`hardware_interface`、`pluginlib`
- 解析スクリプト：Python 3、NumPy（`analyze_first_order.py` / `analyze_b0.py`）
- 起動時に別パッケージ `zlac_can_bringup` を使用（同一ワークスペースに配置する場合）。

`ros2_control`、コントローラおよび関連ツールが未導入の場合の例：

```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-numpy can-utils
```

> 実機の CAN ビットレート、モータ極性、エンコーダ仕様に依存します。下記の値は添付ソースに実装されている設定です。

## ファイル構成

```text
zlac8015d_hardware/
├── CMakeLists.txt
├── package.xml
├── zlac8015d_hardware.xml
├── include/zlac8015d_hardware/
│   ├── zlac8015d_system.hpp
│   └── ...                         # 旧版・バックアップ
├── src/
│   ├── zlac8015d_system.cpp       # ros2_control 実装
│   ├── first_order_identification.cpp
│   └── running_step_identification.cpp
└── scripts/
    ├── analyze_first_order.py
    ├── analyze_b0.py
    └── analyze_ladrc_step.py
```

- `zlac8015d_system.cpp`：CANopen PDO 設定、状態読取り、速度・電流指令、LADRC、停止処理。
- `zlac8015d_hardware.xml`：`pluginlib` に登録するハードウェアプラグインの定義。
- `first_order_identification.cpp` / `running_step_identification.cpp`：独立した SocketCAN 電流切替実験プログラム。**この添付版では両ファイルの実装内容は同一です。**
- `analyze_first_order.py`：実験 CSV から一次の速度ダイナミクスを推定。
- `analyze_b0.py`：**別途取得した b0 同定用 CSV** を解析するスクリプト（対応する収録プログラムはこのZIPに含まれません）。
- `analyze_ladrc_step.py`：専用のステップ試験CSV用解析スクリプト。通常運転ログ `/tmp/ladrc_track_200hz.csv` の形式とは異なります。

## インストールとビルド

このパッケージと `zlac_can_bringup` を `~/ros2_ws/src/` に配置してから実行します。フォルダ名に `-devel-ADRC` などの接尾辞があっても、ROS 2 のパッケージ名は `zlac8015d_hardware` です。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

ビルド後、プラグインは `zlac8015d_hardware/Zlac8015dSystemHardware` として URDF の `<ros2_control>` 要素から読み込まれます。

## 基本的な起動・確認

CAN インターフェースが既に適切なビットレートで設定されていることを確認します。

```bash
ip -details link show can0
```

`can0` が使用できない場合は、使用機器の CAN ビットレートと配線を確認してから設定してください。ビットレートは環境依存のため、このREADMEでは固定値を指定しません。

ハードウェアを含む一式は bringup パッケージから起動します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

別ターミナルでの確認例：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

CAN フレームを直接確認するには `candump can0` を使用できます。停止は `Ctrl+C` です。

## 制御モード

`zlac_can_bringup/urdf/orange_go_2025.urdf` の `<hardware>` 内で切り替えます。

| `control_mode` | 概要 | ドライバへ送る指令 |
| --- | --- | --- |
| `velocity` | ドライバ内蔵の速度制御を使用 | 目標角速度（`0x300 + node_id`） |
| `ladrc_torque` | 左右の一次 LADRC により車輪角速度を追従 | 電流指令（`0x200 + node_id`） |

どちらのモードも **ROS側のコマンドインターフェースは車輪角速度 `velocity` [rad/s]** です。`ladrc_torque` では、ハードウェアプラグイン内で角速度の目標値を電流指令へ変換します。`diff_drive_controller` の設定を変更せず切り替えられます。

LADRC モードでは概念的に次を使用します。

```text
omega_dot = f + b0 * I_cmd
```

各車輪の ESO が角速度と総外乱を推定し、制御器が電流指令を計算します。実装には起動時の静止摩擦対策（BREAKAWAY → TRACK）、その場旋回時の両輪協調、直進時の同期補正、通常停止時の減速ランプおよび電流制限が含まれます。

## URDFで設定するパラメータ

添付された `orange_go_2025.urdf` の初期設定値です。

| パラメータ | 値 | 説明 |
| --- | ---: | --- |
| `can_interface` | `can0` | SocketCAN インターフェース |
| `node_id` | `1` | CANopen ノード ID（1〜127） |
| `control_mode` | `ladrc_torque` | 制御モード |
| `ladrc_b0_left` | `20.0` | 左車輪の公称入力ゲイン [(rad/s²)/A] |
| `ladrc_b0_right` | `20.0` | 右車輪の公称入力ゲイン [(rad/s²)/A] |
| `ladrc_wc` | `5.0` | 制御帯域幅 [rad/s] |
| `ladrc_wo` | `20.0` | オブザーバ帯域幅 [rad/s] |
| `current_limit_a` | `8.0` | 電流指令の上限 [A] |
| `current_rate_limit_a_per_s` | `20.0` | 電流指令の変化率上限 [A/s] |
| `feedback_timeout_ms` | `30.0` | LADRC モードの速度フィードバック受信タイムアウト [ms] |

`b0`、電流制限、極性は実機特性に合わせて確認してください。**`20.0` は添付URDFの設定値であり、すべての機体に適した同定済みパラメータであることを保証する値ではありません。**

## CANopenの入出力

ノード ID が `1` の場合、主な CAN ID は次のとおりです。

| CAN ID | 方向 | 内容 |
| --- | --- | --- |
| `0x181` | ドライバ → PC | 左右の実測角速度（各32 bit、0.1 rpm単位） |
| `0x281` | ドライバ → PC | 左右の実測電流（各16 bit、0.1 A単位） |
| `0x381` | ドライバ → PC | 左右のエンコーダ位置（各32 bit） |
| `0x301` | PC → ドライバ | 速度指令（`velocity` モード） |
| `0x201` | PC → ドライバ | 電流指令（`ladrc_torque` モード） |
| `0x601` | PC → ドライバ | SDO要求 |

初期化時に NMT / SDO を用いて PDO マッピングとドライバ動作モードを設定します。ソースのイベントタイマ設定値は角速度・電流が `10 ms`、エンコーダ位置が `40 ms` です。実際の受信周期は機器・バス状態に依存します。

エンコーダ換算は **1024 line × 4 = 4096 count/rev** です。左右の正方向が一致するように、左モータの符号を反転しています。エンコーダ位置は初回受信時を原点とする相対位置なので、**launchやハードウェアの再起動で位置が再初期化されます**。

## 運転ログ

`ladrc_torque` モードで通常の制御処理が動作している間、以下のCSVを出力します。

```text
/tmp/ladrc_track_200hz.csv
```

収録列には目標・実測角速度、車輪位置、電流指令・実測電流、ESO 推定値 `z1` / `z2`、BREAKAWAY / TRACK 関連情報が含まれます。ファイルは起動後に新規作成され、200サンプルごとにフラッシュされます。**`velocity` モードではこのLADRCログは出力されません。**

## 電流切替によるモデル同定（オプション）

`first_order_identification` は `ros2_control` とは独立してモータを直接操作するため、**bringupを終了し、車輪を浮かせた状態でのみ**実行してください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run zlac8015d_hardware first_order_identification \
  --run --can can0 --node 1 \
  --initial-current 2.0 --switched-current 3.0 \
  --switch-speed 8.0 --speed-limit 15.0 \
  --max-current 3.0 --repeats 5 \
  --output running_step_2A_3A.csv
```

生成されたCSVに対して一次モデルの解析を実施します。

```bash
python3 ~/ros2_ws/src/zlac8015d_hardware/scripts/analyze_first_order.py \
  running_step_2A_3A.csv --output first_order_fit.csv
```

> 上の解析スクリプトのパスは、ソースフォルダを `zlac8015d_hardware` という名前に配置した場合の例です。実際のフォルダ名に読み替えてください。`--run` 指定時はモータが動作します。実機の電流上限・安全条件に合わせて数値を調整してください。

## 安全上の注意

- `feedback_timeout_ms` による速度データの鮮度確認は **LADRCモードの内部処理**です。通信断の原因を修復するものではありません。
- 受信停止時にはゼロ指令を試みますが、**CAN配線の断線・電源異常時にモータが物理的に停止することは本ソフトだけでは保証できません**。非常停止回路およびドライバの保護設定を別途用意してください。
- 同定プログラムと `ros2_control` を同時起動しないでください。どちらも同じドライバを初期化・操作します。
- 実機への適用前に車輪を浮かせてモータ回転方向、停止挙動、エンコーダ更新を確認してください。
- 自動再起動を追加する場合は、別プログラム `can_launch_watchdog.py` を使用できます（この最初の hardware ZIP には含まれていません）。

## ライセンス

添付された `package.xml` のライセンス欄は `TODO` のままです。GitHubで公開する際は権利関係を確認し、適切な `LICENSE` ファイルとライセンス表記を追加してください。
