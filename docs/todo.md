# TODO

快速 catchup：目前進行中的是 1.1（branch `fix/pointlio-map-gate-gravity`），規劃段在該項目底下，「目前做到哪」寫在規劃段最後。

## 1. Point-LIO 長時間運轉的穩定性

### 1.1 Point-LIO 地圖維護 + 重力約束
branch: fix/pointlio-map-gate-gravity
改動點: src/syncai_pointlio/include/syncai_pointlio/map_builder/ivox_map.h, src/syncai_pointlio/src/map_builder/ivox_map.cpp, src/syncai_pointlio/include/syncai_pointlio/map_builder/lidar_processor.h, src/syncai_pointlio/src/map_builder/lidar_processor.cpp, src/syncai_pointlio/include/syncai_pointlio/map_builder/map_builder.h, src/syncai_pointlio/src/map_builder/map_builder.cpp, src/syncai_pointlio/src/map_builder/imu_initializer.cpp, src/syncai_pointlio/include/syncai_pointlio/map_builder/commons.h, src/syncai_pointlio/include/syncai_pointlio/pointlio_node.hpp, src/syncai_pointlio/src/pointlio_node.cpp, src/syncai_pointlio/params/pointlio_params.yaml, src/syncai_pointlio/CMakeLists.txt, src/syncai_pointlio/package.xml, src/syncai_pointlio/test/test_ivox_map.cpp, src/syncai_pointlio/test/test_fixed_gravity.cpp, src/syncai_pointlio/README.md, CLAUDE.md

#### 影響範圍
每格：現在怎麼做｜檔:行｜動或不動。

| 實例＼層 | ↓ 往下的模組 | 本身 | ↑ 往上的模組 |
|---|---|---|---|
| src/syncai_pointlio/include/syncai_pointlio/map_builder/ivox_map.h | 用 commons.h 的 PointType / PointVec｜commons.h:7｜不動 | 每個 grid 只有點列表，不知道哪些 voxel 已有點｜ivox_map.h:57｜動（U2：grid 加 voxel 佔用集合、addPoints 說明、拒絕計數） | LidarProcessor 持有它、node 讀 numGrids / numPoints / numEvicted｜lidar_processor.h:76、pointlio_node.cpp:649｜動（U2：node 印拒絕計數） |
| src/syncai_pointlio/src/map_builder/ivox_map.cpp | 無｜不動 | addPoints 給什麼收什麼｜ivox_map.cpp:54｜動（U2：已佔用的 voxel 直接拒絕） | incrCloudMap / initCloudMap 呼叫 addPoints｜lidar_processor.cpp:43、:87｜動（U2） |
| src/syncai_pointlio/include/syncai_pointlio/map_builder/lidar_processor.h | 依賴 IVoxMap、PointEKF｜lidar_processor.h:8｜不動 | incrCloudMap 宣告；m_nearest_points 供平面擬合與 voxel 判斷｜lidar_processor.h:51、:92｜動（U2：incrCloudMap 說明改寫） | MapBuilder 呼叫 incrCloudMap、stats()｜map_builder.cpp:120｜動（U3） |
| src/syncai_pointlio/src/map_builder/lidar_processor.cpp | IVoxMap::addPoints / nearestSearch｜ivox_map.cpp:54、:79｜動（U2） | incrCloudMap 用最近 5 鄰居判斷 voxel 是否已有點｜lidar_processor.cpp:43｜動（U2：刪掉鄰居判斷，整批交給 addPoints） | MapBuilder::process 結尾呼叫｜map_builder.cpp:120｜動（U3） |
| src/syncai_pointlio/include/syncai_pointlio/map_builder/map_builder.h | LidarUpdateStats｜lidar_processor.h:36｜不動 | ScanHealth 記每個 scan 的匹配與 IMU 統計｜map_builder.h:22｜動（U3：記這個 scan 有沒有寫入地圖） | PointLIONode::recordHealth 讀 health()｜pointlio_node.cpp:560｜動（U3） |
| src/syncai_pointlio/src/map_builder/map_builder.cpp | incrCloudMap、stats()｜lidar_processor.cpp:43｜動（U2） | 每個 scan 無條件 incrCloudMap｜map_builder.cpp:120｜動（U3：先過閘門）；satu_acc 計數讀設定值｜map_builder.cpp:27｜不動 | timerCB 呼叫 process()｜pointlio_node.cpp:511｜不動 |
| src/syncai_pointlio/src/map_builder/point_ekf.cpp | 無｜不動 | updateIMU 依 satu_acc 丟軸、setConfig 讀設定｜point_ekf.cpp:103、:46｜不動（U4 靠初始共變異數，U5 只改值） | MapBuilder::processIMU｜map_builder.cpp:13｜不動 |
| src/syncai_pointlio/include/syncai_pointlio/map_builder/point_ekf.h | 無｜不動 | State::gravity 9.81、g 為 3 維向量｜point_ekf.h:26｜不動 | IMUInitializer 設定 g 與 P｜imu_initializer.cpp:35｜不動 |
| src/syncai_pointlio/src/map_builder/imu_initializer.cpp | PointEKF 的 x() / P()｜point_ekf.h:68｜不動 | g 的初始共變異數寫死 1e-4｜imu_initializer.cpp:49｜動（U4：改由設定決定，預設 0） | MapBuilder 在 IMU_INIT 呼叫｜map_builder.cpp:45｜不動 |
| src/syncai_pointlio/include/syncai_pointlio/map_builder/commons.h | 無｜不動 | Config：地圖、雜訊、satu、gravity_align｜commons.h:29｜動（U3：閘門門檻；U4：g 初始共變異數） | 外殼宣告參數填入 Config｜pointlio_node.cpp:148｜動 |
| src/syncai_pointlio/include/syncai_pointlio/pointlio_node.hpp | ScanHealth、IVoxMap 計數｜map_builder.h:22｜動（U3） | HealthWindow 彙整每秒統計｜pointlio_node.hpp:121｜動（U2、U3：加計數） | 無｜不動 |
| src/syncai_pointlio/src/pointlio_node.cpp | MapBuilder / IVoxMap / Config｜pointlio_node.cpp:148、:649｜動 | 參數宣告｜pointlio_node.cpp:170；health 行格式｜pointlio_node.cpp:700｜動（U2、U3、U4：新參數、行尾新欄位） | lio_odom / body_cloud 的使用者（localizer、mapping、lio_bridge、costmap、planner）｜syncai_localizer/launch:247｜不動（介面不變） |
| src/syncai_pointlio/params/pointlio_params.yaml | 節點宣告的參數｜pointlio_node.cpp:170｜動 | satu_acc 30｜pointlio_params.yaml:100；地圖參數｜:85｜動（U3、U4 新參數；U5 satu_acc） | launch 載入｜launch/pointlio.launch.py:78｜不動 |
| src/syncai_pointlio/CMakeLists.txt | PCL、Eigen、Sophus｜CMakeLists.txt:61｜不動 | 核心原始碼直接編進 executable｜CMakeLists.txt:68｜動（U1：核心編成 library、加 gtest） | colcon / CI build｜.github/workflows/ci.yml｜不動 |
| src/syncai_pointlio/package.xml | 無｜不動 | test_depend 只有 lint｜package.xml:45｜動（U1：加 ament_cmake_gtest） | rosdep / CI｜scripts/build.sh｜不動 |
| src/syncai_pointlio/test/test_ivox_map.cpp | IVoxMap｜ivox_map.h:28｜新增 | 目前沒有｜（新檔）｜新增（U1 骨架、U2 測試） | colcon test｜CMakeLists.txt:110｜動（U1） |
| src/syncai_pointlio/test/test_fixed_gravity.cpp | PointEKF、IMUInitializer｜point_ekf.h:45｜不動 | 目前沒有｜（新檔）｜新增（U4、U5） | colcon test｜CMakeLists.txt:110｜動（U4） |
| src/syncai_pointlio/README.md | 無｜不動 | 局部地圖、health log、參數表｜README.md:126、:232、:283｜動（每單位同步） | 無｜不動 |
| CLAUDE.md | 無｜不動 | pointlio 那列描述 first-wins 與局部地圖｜CLAUDE.md:93｜動（U2、U3、U4 收尾時改描述） | 無｜不動 |
| src/syncai_costmap/src/terrain_grid.cpp（同類：地圖容器） | 無｜不動 | 滾動網格，射線清除與記憶時間，本身有上限｜terrain_grid.cpp｜不動 | TerrainLayer｜terrain_layer.hpp｜不動 |
| src/syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.cpp（同類：地圖容器） | 無｜不動 | 動態障礙疊加層，30 秒遺忘，本身有上限｜dynamic_overlay.cpp｜不動 | Octo3D｜octomap_planner_3d｜不動 |

#### 診斷
- 地圖：「每個 0.3 m voxel 一點」的規則不在 IVoxMap 裡，由 incrCloudMap 只看最近 5 鄰居判斷，已有點的 voxel 會再收第二點；寫入沒有任何條件，滑動時錯的位姿被寫成走廊副本。前者是結構偏離（容器不擁有自己的不變條件），也是病因，所以改結構。
- 重力：g 是沒有過程雜訊的 3 維向量，靜止時 |g| 與 ba 沿重力的分配看不出來，走路時一路被推低（兩種地圖都是 −0.014/h）；satu_acc 30 m/s² 丟掉 3.06–4.0 g 的有效樣本（被丟的約 90% 有效），IMU z 軸硬削頂在 4.004 g。
- 冷啟動重播：同一段資料新狀態不會滑；2.5 倍速重現跳 scan 也不會滑，所以跳 scan 單獨不是主因。

#### 決定（全局確認時定案）
- 一格一點：地圖內精確判定，刪掉 incrCloudMap 的鄰居判斷。U2 細節（使用者確認）：整張地圖共用一份已佔用 voxel 集合（跨 grid 的 voxel 也嚴格一點，淘汰 grid 時釋放它的 voxel）；第一個 scan 也套用；updateChunk 的搜尋半徑由 5 m 收緊到 √5 m（與鄰居門檻共用一個常數，選點結果不變）；health 行尾加累計 `refused N`。U2 驗證後追加（使用者決定）：map_resolution 0.3 → 0.2，因為嚴格一格一點在 0.3 m 讓匹配變差（走路 eff 94 → 86%），0.2 m 回到 94%、殘差 2.33 cm。
- 寫入閘門：~~eff 低於門檻或速度超過門檻就不寫~~。已移除（使用者決定，11-09）：比例分不出位姿錯和地圖還沒有這個表面，閘門關上後會卡死；只看速度的版本保護的東西在 reset 時本來就會清掉。改做失控偵測 + reset 觸發（1.4）與速度量測（1.2、1.3）。
- 重力：固定 g（初始共變異數 0，無過程雜訊，之後不會被修正）。
- satu_acc：略低於削頂值，約 39.0 m/s²。
- 編譯與測試在用完即丟的容器裡，以現有 install/ 當唯讀底層，只把 syncai_pointlio 編到 build_fix/、install_fix/，不碰 robot01 的 build/、install/。

#### 單位、順序、驗證
| 單位 | 內容 | 驗證 |
|---|---|---|
| U3 | ~~寫入閘門（eff + 速度），health 加未寫入計數~~ 已移除（d31ef90、fc5cba2 revert） | 見 docs/pointlio-divergence-2026-11-09.md §7.1 |
| U4 | 固定 g（初始共變異數參數，預設 0） | 單元：大量 IMU / 光達更新後 g 完全不變；壓力：長時間含衝擊的 IMU 序列 |g| 不漂；重播：|g| 全程 9.81、|ba| 小 |
| U5 | satu_acc 調到削頂值以下 | 單元：3.5 g 樣本被使用、4.004 g 被丟；重播：sat 計數只剩削頂樣本 |

最終驗證：上車長跑至少 3 天，看處理時間、地圖點數、|g|、P3 pitch 偏差是否平穩；時機由使用者決定。

#### 目前做到哪
U1（b888700）、規劃 todo（ef416ad）已提交並推上 origin。U2（IVoxMap 自己保證一格一點、第一個 scan 也套用、搜尋半徑 √5 m、health 加 refused、map_resolution 0.2）完成並提交。重播（11-08 資料 1.0 倍速）：0.2 m 的走路 eff 94%（最差 scan p1 89%）、殘差 2.33 cm、處理時間約同全新地圖的舊版，地圖點數有上限、每圈新增遞減；靜止位置 3 cm 內、無跳 scan。0.25 m 的對照重播在跑（只供參考）。robot 的 install/ 參數檔是 symlink：yaml 的 0.2 會在下次啟動生效，robot 要先重新編譯 syncai_pointlio。U3 的門檻：正常走路最差 scan eff p1 約 89%（0.2 m 重播）與 robot 歷史 88% 一致，0.85 可沿用但要再用重播確認誤擋次數。U3（d31ef90）已提交；11-09 上車爬折返梯時，比例條件在位姿正確下關門，地圖凍結後再也打不開而發散（docs/pointlio-divergence-2026-11-09.md），fc5cba2 先在參數檔關閉閘門。之後整個移除閘門（revert d31ef90、fc5cba2，理由在該報告 §7.1），src/ 回到 U2 的狀態，health 行尾不再有 `unwritten`。下一步：U4、U5；在錄影開著的情況下重爬這座樓梯（該報告 §7.4）；失控偵測與 reset 觸發併入 1.4。

### 1.2 零速度更新（cmd_vel 線速度為 0 時約束平移速度）

### 1.3 腿部運動學速度量測（motor_states + G23 URDF）

### 1.4 LOST 後在充電座上自動 reset_odometry

### 1.5 lio-recorder：localizer 不收斂的 LOST 不觸發存檔，或調高 MAX_SAVES

### 1.6 map_nearby：上游實際是 18（程式碼 fallback），修正 README / CLAUDE.md「upstream's 6」的說法

### 1.7 分析文件更正：IMU z 軸在 4.004 g 削頂，「最大 3.94 g、沒有削頂」不成立

### 1.8 提高陀螺儀信任（imu_meas_omg_cov）的重播實驗

### 1.9 錄一整晚原始資料，供之後每個版本離線比對
