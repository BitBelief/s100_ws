# s100_ws

WHEELTEC S100 差速底盤的最小 ROS 2 驅動。協定為自行解碼，非廠商原始碼；MIT。

`/cmd_vel` 進 → `/odom`、`/imu/data_raw`、`/battery_state` 出。

## 建置

```bash
cd ~/s100_ws && colcon build --symlink-install && source install/setup.bash
```

## 跑實機

```bash
ros2 launch wheeltec_s100_driver s100.launch.py port:=/dev/ttyACM0
```

## 沒接車也能驗

`tools/fake_s100.py` 開一個 TCP 埠，照實機協定回 24-byte 幀，並複製了實機量到的行為
（20 Hz、速度一階延遲約 0.2 s、STM32 看門狗 1.0 s 歸零、靜止 az = +1.00 g）。
pyserial 的 `socket://` handler 讓驅動一行都不用改：

```bash
python3 src/wheeltec_s100_driver/tools/fake_s100.py 5757
```

```bash
ros2 run wheeltec_s100_driver s100_driver --ros-args -p port:=socket://127.0.0.1:5757
```

## 測試

```bash
cd src/wheeltec_s100_driver && python3 -m pytest test/ -q
```

## 未完成項目（`config/s100.yaml`）

- `linear_scale` / `angular_scale` 仍是 `1.0`。要跑直線 2 m ＋ 原地轉 10 圈，填入實測／指令的比值。
- `pose_covariance_diagonal` / `twist_covariance_diagonal` 是 placeholder，不是估計出來的不確定度 —— 底盤韌體不回報它。餵進 EKF 或 Nav2 前要先正視這件事。

## 為什麼 build/ install/ log/ 不進版控

colcon 可以從 `src/` 完整重生它們。它們會隨每次建置變動，進版控只會把真正的程式碼改動淹掉。
