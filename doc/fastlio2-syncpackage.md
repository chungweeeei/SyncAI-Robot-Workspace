# FASTLIO2 `syncPackage()`:`lidar_pushed` 與 `curvature` 排序解析

> 對象檔案:`src/third-party/FASTLIO2_ROS2/fastlio2/src/lio_node.cpp`
> 相關:`utils.cpp`(填 `curvature`)、`map_builder/imu_processor.cpp`(去畸變)

這份筆記解釋 `syncPackage()` 裡兩個容易看不懂的設計:
1. 為什麼需要 `lidar_pushed` 這個旗標
2. 為什麼要對點雲的 `curvature` 欄位做排序

---

## 背景:為什麼要「去畸變 (de-skew / motion compensation)」

雷達掃一圈不是「啪」一張快照,而是像**用手電筒慢慢掃過房間**,大約花 0.1 秒。這段時間內機器人還在移動,所以同一幀裡:

- 掃描**開頭**的點,是機器人在 A 位置量到的
- 掃描**結尾**的點,是機器人在 B 位置量到的

直接把這些點拼起來,整幀會歪掉、糊掉(像拍照手震)。因此要靠 IMU 知道「這 0.1 秒內移動了多少」,把每個點**修正回同一個時刻**——這就是去畸變。

要做這件事,程式需要知道**每個點是在這幀的第幾毫秒被打到的**。這個「時間標籤」被存在 PCL 點的 `curvature` 欄位裡(欄位名是借用的,實際存的是時間):

```cpp
// utils.cpp:70 (Livox CustomMsg 路徑)
p.curvature = msg->points[i].offset_time / 1000000.0f;  // ns -> ms

// utils.cpp:44 (Isaac PointCloud2 路徑) — 沒有逐點時間,設 0 = 不做去畸變
p.curvature = 0.0f;
```

---

## 問題 1:為什麼要判斷 `lidar_pushed`?

**比喻:沖泡麵,但熱水還沒燒開。**

- 雷達幀 = 泡麵(已備好)
- IMU 資料 = 熱水(要慢慢燒到夠)

規則:**IMU 的時間必須覆蓋到「整幀掃完的那一刻」(`cloud_end_time`),才能開始處理這幀**,因為去畸變需要整段掃描期間的 IMU。

```cpp
// lio_node.cpp:189
if (m_state_data.last_imu_time < m_package.cloud_end_time)
    return false;   // IMU 還沒覆蓋到掃描結束,這一 tick 先放棄
```

`timerCB` 每 20ms 觸發一次,IMU 不見得剛好對得上,所以同一幀 lidar 可能連續好幾個 tick 都 return false,直到 IMU buffer 追上。

`lidar_pushed` 就是一張便利貼:**「這幀我已經拆好、料都加好了」**。第一次碰到 front frame 時要做幾件一次性/昂貴的事:

```cpp
// lio_node.cpp:180-188
if (!m_state_data.lidar_pushed)
{
    m_package.cloud = m_state_data.lidar_buffer.front().second;  // 取出點雲
    std::sort(...);                                               // 排序(昂貴!)
    m_package.cloud_start_time = ...;
    m_package.cloud_end_time   = start + back().curvature/1000.0; // 算掃描結束時刻
    m_state_data.lidar_pushed = true;                             // 貼便利貼
}
```

- **沒有旗標**:IMU 還沒追上的每個 tick 都會重新排序整幀點雲、重算時間——純浪費。
- **有旗標**:準備只做一次;之後失敗的 tick 只重新檢查 IMU 條件。

注意「準備」和「真正消費」是分兩階段的——lidar frame 要等 IMU 條件過了才 `pop_front()`(第 198 行),同時把旗標重設 false:

```
tick 1: 沒 pushed → 排序、算時間 → pushed=true → IMU 沒過 → return false
tick 2: 已 pushed → 跳過準備      →              IMU 沒過 → return false
tick 3: 已 pushed → 跳過準備      → IMU 追上 → 撈 IMU、pop lidar、pushed=false → return true
```

**一句話:`lidar_pushed` 讓「準備一幀」只做一次,把它跟「等 IMU 湊齊」解耦,避免重複做排序這種昂貴工作。**

---

## 問題 2:為什麼要對 `curvature`(時間)排序?

**比喻:按時間順序,把一疊照片配上「當時的 GPS 位置」。**

去畸變時,IMU 那邊有一串「時間 → 位置」的紀錄(照時間排好),雷達每個點也要去對「你是第幾毫秒?那時我在哪?」

程式用的是**兩邊都從最新往最舊、同步一步步往回走**的高效做法(雙指標倒序前進):

```cpp
// imu_processor.cpp:105-130 (簡化)
auto it_pcl = package.cloud->points.end() - 1;          // 從最新一點開始
for (auto it_kp = poses_cache.end()-1; ...; it_kp--)    // 從最新的 IMU pose 往回走
{
    auto head = it_kp - 1;
    for (; it_pcl->curvature/1000 > head->offset; it_pcl--)  // 靠時間單調來配對
    {
        dt = it_pcl->curvature/1000 - head->offset;
        ... 用該時刻的 IMU pose 把這點補償回掃描結束時刻 ...
    }
}
```

這個方法的**前提是雷達的點必須照時間排好隊**。但雷達送出來的點順序是照掃描機械模式排的,**時間上是亂的**。若不先排序:

- 程式會把「第 3 毫秒的點」錯配到「第 50 毫秒的位置」→ 修正量算錯 → 整幀去畸變變垃圾 → 里程計精度崩掉。

排序還有個副作用好處:排完之後 `points.back()` 保證是**時間最晚的點**,所以

```cpp
m_package.cloud_end_time = cloud_start_time + cloud->points.back().curvature / 1000.0;
```

才會正確算出「掃描結束時刻」——而這正是問題 1 裡 IMU 要覆蓋到的基準時間。

**一句話:排序是為了讓「點」和「當時的位置」能正確配對,不然去畸變會全錯。**

---

## 串起來看

```
雷達幀進來(點的時間是亂的)
   │
   ├─ 第一次看到 → 排隊(sort)+ 算「掃到第幾毫秒結束」→ 貼便利貼(pushed=true)
   │
   ├─ IMU 還沒燒到那一刻 → 先走,下次再看(便利貼還在,不重排)
   │
   └─ IMU 燒到了 → 撈出這段 IMU、下鍋處理、撕便利貼 → 去畸變(靠排好的順序配位置)
```

- **`lidar_pushed`** 管「**時間上要等 IMU 湊齊**」——不要重複準備。
- **`sort(curvature)`** 管「**空間上要正確配位置**」——不然整幀歪掉。

兩者都是為了把「一整幀在運動中掃出來的點」正確補償回同一時刻。

---

## Isaac Sim 注意事項

Isaac 走的 `pc2ToPCL` 路徑把 `curvature = 0`(`utils.cpp:44`,標明 *"no per-point time -> snapshot, no deskew"*),等於**放棄逐點去畸變**,整幀當同一瞬間快照。sim 低速/高頻掃描影響不大;若之後 Isaac 能吐出 per-point timestamp,這裡是提升精度的著手點。
