# FASTLIO2 IESKF `update()`:迭代誤差狀態卡爾曼的後驗修正

> 對象檔案:`src/third-party/FASTLIO2_ROS2/fastlio2/src/map_builder/ieskf.cpp`
> 相關:`lidar_processor.cpp` 的 `updateLossFunc()`(提供雷達量測的 H/b)
> 前置:`predict()` 已在 `imu_processor::undistort()` 裡跑過,產生 IMU 先驗

這份筆記解釋 FAST-LIO2 定位的數學核心 `IESKF::update()`——如何把「IMU 預測的先驗」和「雷達點到平面的量測」融合成最準的姿態估計。

---

## 0. 心智模型

- `predict()`(IMU 前向傳播)= 閉眼靠體感走一步,得到**先驗猜測** `predict_x`,帶著不確定度 `m_P`(會愈積愈大)。
- `update()`(這份筆記)= 張眼看雷達,把先驗**校正**到跟環境對得上,並收縮不確定度。

---

## 1. 它在最小化什麼

`update()` 本質是解一個最佳化問題,同時懲罰兩件事:

```
總誤差 = 【偏離 IMU 先驗多少】     ← 用先驗可信度 P⁻¹ 加權
       + 【雷達點離地圖平面多遠】  ← 用雷達可信度 lidar_cov_inv 加權
```

- 第一項:盡量待在 IMU 猜的姿態附近(IMU 短時間很準)。
- 第二項:點雲要貼合地圖的牆/地板(雷達長時間很準)。

兩者拉扯的平衡點,就是 Kalman filter 的最大後驗估計 (MAP)。這裡用**迭代**方式解,因為雷達量測是非線性的。

---

## 2. 狀態向量與流形 (manifold)

21 維狀態:`r_wi(0-2) t_wi(3-5) r_il(6-8) t_il(9-11) v(12-14) bg(15-17) ba(18-20)`。

旋轉 `r_wi / r_il` 活在彎曲的 SO(3) 空間,**不能直接加減**:

```cpp
// State::operator+=  (ieskf.cpp:14)  在流形上「套用」修正
r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();   // 旋轉用 exp 套用
t_wi += delta.segment<3>(3);                                // 平移可直接加
...

// State::operator-   (ieskf.cpp:25)  在流形上「量出」兩姿態的差
delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();  // 旋轉用 log
delta.segment<3>(3) = t_wi - other.t_wi;
...
```

`Jr / JrInv`(右雅可比及其逆)是切空間的修正因子:因為誤差狀態定義在旋轉的切空間,共變異數 `P` 在流形上傳遞時要乘上這個修正,才數學正確。

---

## 3. `predict()` 回顧(先驗怎麼來的)

在 undistort 裡對每對相鄰 IMU 呼叫,離散運動模型往前推狀態、並膨脹共變異數:

```cpp
void IESKF::predict(const Input &inp, double dt, const M12D &Q) {
    // 名目狀態推進
    delta.segment<3>(0)  = (inp.gyro - bg) * dt;                 // 姿態 <- 角速度
    delta.segment<3>(3)  = v * dt;                                // 位置 <- 速度
    delta.segment<3>(12) = (r_wi*(inp.acc - ba) + g) * dt;        // 速度 <- 加速度(去重力)
    m_x += delta;

    // 誤差狀態轉移矩陣 F、噪聲雅可比 G
    m_F = ...; m_G = ...;
    // 共變異數傳播(不確定度變大)
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}
```

`Q` 是過程噪聲(來自 config 的 `ng/na/nbg/nba`)。重點:**predict 讓 P 變大(愈來愈不確定),update 讓 P 變小(被雷達修準)**。

---

## 4. `update()` 迭代迴圈逐步拆解

```cpp
void IESKF::update() {
    State predict_x = m_x;        // 存下 IMU 先驗(迴圈中不變)
    ...
    for (i = 0; i < m_max_iter; i++) {   // ieskf_max_iter,預設 5

        // ① 在「當前估計 m_x」下算雷達那項(重找鄰居、擬合平面)
        m_loss_func(m_x, shared_data);        // = updateLossFunc
        if (!shared_data.valid) break;        // 無有效平面 → 放棄

        // ② 算「偏離先驗多少」這一項
        delta = m_x - predict_x;              // 流形上的差 (⊟)
        M21D J = Identity;
        J.block<3,3>(0,0) = JrInv(delta.segment<3>(0));   // 旋轉塊的流形修正
        J.block<3,3>(6,6) = JrInv(delta.segment<3>(6));
        H += J.transpose() * m_P.inverse() * J;           // 先驗資訊矩陣
        b += J.transpose() * m_P.inverse() * delta;       // 先驗梯度

        // ③ 把雷達量測疊上去(只影響前 12 維:姿態+外參)
        H.block<12,12>(0,0) += shared_data.H;
        b.block<12,1>(0,0)  += shared_data.b;

        // ④ 解 Gauss-Newton 正規方程,更新狀態
        delta = -H.inverse() * b;             // H·delta = -b
        m_x += delta;                          // 流形上套用
        if (m_stop_func(delta)) break;         // 修正量夠小 → 收斂
    }

    // ⑤ 更新後驗共變異數
    M21D L = Identity;
    L.block<3,3>(0,0) = Jr(delta.segment<3>(0));
    L.block<3,3>(6,6) = Jr(delta.segment<3>(6));
    m_P = L * H.inverse() * L.transpose();
}
```

### 各段意義

- **① 雷達項**:`updateLossFunc` 在當前姿態下,把每個降採樣點投到世界、在 ikd-Tree 找 5 個最近鄰、擬合平面、算點到平面距離,組成 `H(12×12)`、`b(12×1)`。只有前 12 維,因為點到平面只跟姿態/外參有關。權重 `lidar_cov_inv`(config,預設 1000)= 有多信任雷達。
- **② 先驗項**:`P⁻¹` 是先驗可信度。IMU 猜得愈準(P 小)→ P⁻¹ 愈大 → 愈不准偏離先驗。
- **③ 融合**:兩種資訊在資訊矩陣層級**相加**——這就是「緊耦合融合」實際發生處。
- **④ 求解**:`delta = -H⁻¹b` 是 Gauss-Newton 步。
- **停止條件**(`lidar_processor` 建構子,ieskf.cpp 對應 stop_func):
  ```cpp
  return (rot_delta.norm()*57.3 < 0.01)   // 轉角修正 < 0.01 度
      && (t_delta.norm()*100 < 0.015);    // 平移修正 < 0.15 mm
  ```

---

## 5. 為什麼要「迭代」——IESKF 的 "I"

一般 EKF 只線性化一次。但 point-to-plane 是非線性的:姿態一改,每個點的最近鄰、擬合平面、法向量**全都變**。所以第一次算的 H/b 只在舊姿態附近準。

IESKF 的做法:更新 → 用新姿態**重跑** `updateLossFunc`(重找鄰居、重擬合)→ 再更新……反覆逼近,像 Gauss-Newton。快速運動時精度遠勝一次線性化的 EKF。`ieskf_max_iter: 5` 是上限,通常 2~3 次就靠 stop_func 提早收斂。

---

## 6. 資訊形式 (information form) 的巧妙

注意它解的是 `(P⁻¹ + HᵀR⁻¹H) delta = ...`(資訊/Hessian 形式),而不是經典 Kalman gain `K = PHᵀ(HPHᵀ+R)⁻¹`。

原因:雷達量測維度是「數千個點」,狀態只有 21 維。經典形式要反轉「數千×數千」的矩陣;資訊形式只反轉「21×21」的 H。**這是 FAST-LIO 跑得快的關鍵設計之一。**

---

## 7. 輸出

迴圈收斂後:

- `m_x`:修正後的最佳姿態(`r_wi/t_wi` 等)——就是 `lio_node` 拿去發 `lio_odom` / TF 的東西。
- `m_P`:收縮後的後驗共變異數,帶進下一幀的 predict 繼續傳播。

接著 `lidar_processor::process()` 用這個修好的姿態呼叫 `incrCloudMap()`,把新點畫進地圖。

---

## 待深入(TODO)

- `predict()` 裡 `m_F` / `m_G` 各區塊的推導(誤差狀態運動學)。
- `updateLossFunc` 的 Jacobian `B/C/D` 區塊怎麼從點到平面殘差對狀態微分而來。
- `esti_plane` 的平面擬合(最小平方 + 品質檢查)。
