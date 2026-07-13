# 點雲運動去畸變（Point Cloud Motion Undistortion）

> 對應程式碼：`src/IMU_Processing.cpp` 的 `ImuProcess::UndistortPcl()`
> 這份文件用白話 + 程式對照的方式，記錄 FAST-LIO2 如何消除 LiDAR 掃描期間因機器人移動造成的點雲變形。

---

## 1. 問題：為什麼點雲會變形？

LiDAR 掃完一圈需要時間（機械/旋轉式約 0.1 秒）。這段時間裡機器人在移動、在轉彎，所以**同一幀點雲裡，先打出的點和後打出的點，其實是機器人在不同位置、不同朝向時量到的**。但 LiDAR 把整圈點打包成「一幀」交出去，預設它們是「同一瞬間量到的」——這個假設在機器人運動時不成立，就造成 **motion distortion（運動畸變）**。運動越快，變形越嚴重。

### 具體例子（平移）

機器人以 1 m/s 筆直前進，LiDAR 掃一圈 0.1 秒 → 這一圈期間機器人前進了 10 cm。

盯住打中同一根柱子的兩個點：

- **點 A**：這圈最開始（t=0.00s）打出，機器人在「起點」，量到柱子距離 **5.00 m**
- **點 B**：這圈最後（t=0.10s）打出，機器人已在「起點前方 10 cm」，量到柱子距離 **4.90 m**

同一根柱子，兩個點卻回報不同距離。電腦若以為機器人沒動，就會覺得「柱子是歪的/糊的」——其實柱子是直的，是機器人的移動被忽略了。

畸變有兩種來源，原理相同（都是「量測當下的位姿」與「掃描結束的位姿」有落差）：
- **平移**：機器人前進，距離變近（上面的例子）
- **旋轉**：機器人轉彎，每個點的角度歪掉

---

## 2. 解法：統一對齊到「掃描結束時刻」

去畸變的核心思想：**不讓每個點各自站在不同位置量，而是把所有點換算成「機器人在掃描結束那一刻量到」的座標。**

以上面的例子，掃描結束時機器人在「起點前方 10 cm」，以此為基準：
- 點 B 本來就在這個位置量的（4.90 m），不用改。
- 點 A 是在 10 cm 前量的（5.00 m）。若機器人當時就已在結束位置，該量到 4.90 m → 把點 A 距離補正為 4.90 m。

修完後兩點都說 4.90 m，柱子恢復又直又平。整幀所有點都這樣搬一次，點雲就像「掃描結束那一瞬間一次拍下」的乾淨快照。

**為什麼選「掃描結束時刻」當基準？** 因為去畸變是反向（時間由晚到早）逐點修正，從最接近基準的點開始往回修，外插誤差累積最小。修正後的點雲時間戳統一在 `lidar_end_time`，後續配準與建圖都以此為準。

---

## 3. 它怎麼知道每個點當時在哪？

三個資訊來源：

1. **機器人在這 0.1 秒怎麼動** → 由 IMU（高頻角速度 + 加速度）經**前向傳播**積分出來，存進 `IMUpose`（掃描期間每個 IMU 時刻的 位置/速度/姿態/加速度/角速度 軌跡）。
2. **每個點是這圈的第幾個時刻打出的** → 前處理階段把「該點相對掃描起點的時間（ms）」塞進點的 `curvature` 欄位。
3. **座標換算** → `P_compensate` 公式把點從「量測時刻的姿態」搬到「掃描結束時刻的姿態」。

---

## 4. 程式邏輯逐段對照

### (a) 前置：接續上一幀、依時間排序

```cpp
auto v_imu = meas.imu;
v_imu.push_front(last_imu_);          // 接上一幀尾巴的 IMU，傳播不留時間縫隙
pcl_out = *(meas.lidar);
sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);  // 依 curvature(相對時間) 排序
```
排序是後面「兩個游標反向掃描」能成立的前提。

### (b) 前向傳播：推狀態 + 存 IMUpose

對每兩筆相鄰 IMU（`head`/`tail`）做一次 EKF `predict()`，把狀態從上一幀結束推進到這一幀結束，並把每個時刻的運動狀態存進 `IMUpose`：

```cpp
kf_state.predict(dt, Q, in);          // 用 IMU 運動模型(use-ikfom.hpp 的 get_f)積分狀態
imu_state = kf_state.get_x();
angvel_last = angvel_avr - imu_state.bg;              // 扣 gyro bias 的角速度
acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba) + imu_state.grav;  // 世界系加速度
IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last,
                             imu_state.vel, imu_state.pos, imu_state.rot...));
```

迴圈後再補傳播一小段到 `pcl_end_time`，得到**全幀基準姿態** `imu_state`（掃描結束時刻的狀態）。

### (c) 反向傳播：逐點去畸變

兩個游標都從尾端（時間最晚）往前走：`it_pcl` 掃點雲，`it_kp` 掃 IMUpose 區間。對每個 IMU 區間，處理所有時間落在其中的點：

```cpp
for (; it_pcl->curvature/1000 > head->offset_time; it_pcl--) {
    dt = it_pcl->curvature/1000 - head->offset_time;   // 該點相對區間起點的時間

    M3D R_i(R_imu * Exp(angvel_avr, dt));              // ① 該點量測時刻的姿態(外插)
    V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);          //    點在「量測時刻 LiDAR 系」的座標
    V3D T_ei(pos_imu + vel_imu*dt + 0.5*acc_imu*dt*dt - imu_state.pos);  // ② 與結束時刻的位移差(世界系)

    V3D P_compensate = imu_state.offset_R_L_I.conjugate() *
                       (imu_state.rot.conjugate() *
                          (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei)
                        - imu_state.offset_T_L_I);      // ③ 完整座標鏈
    it_pcl->x/y/z = P_compensate;                      // 寫回去畸變後座標
}
```

`P_compensate` 公式從最內層 `P_i` 往外讀 = 四步座標轉換：

1. `offset_R_L_I * P_i + offset_T_L_I` — 量測時刻的 **LiDAR 系 → IMU 系**（外參）
2. `R_i * (...) + T_ei` — → **世界系**（用該點時刻姿態 R_i 旋轉、加位移差 T_ei）
3. `imu_state.rot.conjugate() * (...)` — → **掃描結束時刻的 IMU 系**（乘結束姿態的逆 Rₑᵀ）
4. `offset_R_L_I.conjugate() * (... - offset_T_L_I)` — → **掃描結束時刻的 LiDAR 系**（外參的逆）

物理意義：假裝這個點也是在掃描結束那一瞬間、以結束時的感測器姿態量到的。
註解 `// not accurate!` 是因為姿態用外插、位移用等加速度近似，有微小誤差，但對配準足夠。

---

## 5. 與本專案（Isaac Sim）的關係 ⚠️

本專案在 sim 中走 `default_handler`（`config/isaac_sim.yaml` 的 `lidar_type: 5`），該路徑產生的點 **`curvature` 全部為 0**（沒有逐點時間戳）。因此：

- 內層迴圈條件 `it_pcl->curvature/1000 (=0) > head->offset_time` 幾乎不成立
- **去畸變實際上不作用**，每個點原封不動

這在 Isaac Sim 是**正確的**——sim 一次渲染整圈，本來就沒有掃描內時間差、沒有拖影。

**但若之後接真實旋轉式 LiDAR，或 sim 開啟掃描時間模擬**，點雲會有運動畸變，屆時必須讓前處理正確填入每個點的 `curvature`（相對時間），這段去畸變才會真正生效。

---

## 6. 一句話總結

> LiDAR 掃一圈要花時間，機器人這段時間在動，所以一幀點雲其實是「在好幾個不同位置陸續拍的」。去畸變就是**把每個點都換算成「機器人在掃描結束時刻量到」的座標**，讓運動造成的變形消失，交給後續配準的是一張乾淨快照（`feats_undistort`）。
