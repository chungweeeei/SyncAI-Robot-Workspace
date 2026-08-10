# MCP Server 應用提案

> 對象:這個 workspace 裡「還沒有人做」的 MCP 工具層
> 相關:`src/syncai_ros_mcp/`(已存在的 runtime server,見 §6 的關係說明)
> 狀態:**提案,尚未實作**。這份文件是設計討論的落點,不是既有行為的說明書。

這份筆記回答兩個問題:在這個 workspace 裡,一個 MCP server 能提供什麼**現在拿不到**
的東西;以及當一個 server 同時握有 **ROS 工具**和 **REST 工具**時,多出來的價值在哪。

---

## 0. 心智模型:MCP server 不是 API proxy

把 REST endpoint 一對一包成 MCP tool,是最容易做也最沒有價值的做法——agent 本來就能
發 HTTP。值得包成工具的只有三種東西:

| 類型 | 為什麼非工具不可 |
|---|---|
| **跨來源關聯** | 答案要同時看日誌、DDS、REST、Temporal 才拼得出來 |
| **內含判斷** | 原始資料倒出來 agent 也讀不懂,需要把部落知識編碼進去 |
| **CLI 摸不到的狀態** | gzip 輪替的 multilog、Temporal workflow history、活的 TF 樹 |

這個 workspace 三種都很多,原因是結構性的:**沒有 lifecycle manager**。開機順序靠
`config/sessions/*.yaml` 的 `sleep` 偏移編碼,每個節點起來就是 active,於是「整套 stack
現在健康嗎」這個問題**沒有任何單一來源可以問**。

---

## 1. 這個 workspace 的四類把手

| 把手 | 位置 | 目前只能怎麼取用 |
|---|---|---|
| 10 個子系統的持久化日誌 | `log/stack/<robot_id>/<name>/`(mapping 走 `mapping/` 子樹),multilog 16 MiB × 10 gzip 輪替 | 手動 `tail current` / `zcat @*.s`,自己對時間戳 |
| 活的 ROS graph + TF + action server | DDS,全部 namespace 在 `<robot_id>/` 下 | `ros2 topic/node/param/service` CLI |
| 每台機的 backend REST / WS | 各機 `:3000`(real profile 是 `network_mode: host`) | curl / 前端 `:3001` |
| 編排狀態 | Temporal `:7233`,task queue 依 `robot_id` 分流 | Temporal UI `:8081` |

有一個細節特別值得注意:`syncai_sys_manager` 的 `MonitorManager` **只把記憶體 / 磁碟用量
打到 stdout,不發 topic**——這是刻意的,因為 `ROS_LOG_DIR` 是 tmpfs,byobu 的
`pipe-pane` 才是持久記錄。也就是說**機器人的資源歷史只存在於日誌裡**,沒有任何程式化的
取用路徑。這件事直接決定了 §3 的第一順位。

---

## 2. ROS 工具 + REST 工具混在同一個 server:這才是核心價值

答案是肯定的,但理由不是「兩種都支援比較方便」,而是**兩邊回答的是不同種類的問題**:

| | ROS 工具 | REST 工具 |
|---|---|---|
| 回答 | 「**現在**這一瞬間的物理狀態」 | 「**被記錄下來**的意圖與歷史」 |
| 資料 | TF、costmap、action 回饋、`cmd_vel` | vertex、task、schedule、地圖 metadata |
| 來源 | DDS(易失、高頻、無歷史) | Postgres(持久、低頻、有 id) |
| 需要 | 同 DDS domain 的 rclpy process | 只要 HTTP 到得了 `:3000` |

真正有價值的工具是**跨過這條線**的那些——單一介面做不到,而且 agent 自己串會串錯:

**範例 A:地圖 vertex 可達性驗證(機器人完全不用動)**

```
verify_vertices(map_name)
  REST : GET /api/v1/maps/{name}  → 取出所有 vertex 與座標
  ROS  : 對每個 vertex 呼叫 planner 的 ComputePathToPose action
  ROS  : 讀 TF map → <robot_id>/base_link 拿目前位置當起點
  判斷 : 哪些 CHARGER / HOME 現在規劃不出路徑、路徑長度是否異常
```

REST 那半知道「應該有哪些點」,ROS 那半知道「現在到不到得了」。合起來是一句
「地圖改完了,幫我驗一下」——分開就是二十次手動操作。

**範例 B:任務失敗的完整因果**

```
explain_task(task_id)
  REST     : GET /api/v1/tasks/{id}        → 步驟清單與宣稱的狀態
  Temporal : workflow history              → 哪個 activity 失敗、retry 幾次
  ROS      : navigate_to_pose 的 result code
  日誌     : 同一時間窗的 controller / lio_bridge 輸出
```

四個來源各自只有一片。目前要回答「為什麼這個任務卡住」得開四個視窗。

**範例 C:狀態的雙重驗證**

`RobotState.msg` 裡有兩組東西刻意並存:commanded 與 measured。`SetPolicyMode` /
`SetMotionKey` 是**單向 UDP,沒有 ack**,而 `low_level_mode` 是從 gait controller 的
telemetry 讀回來的**實測值**。一個工具同時發命令(REST)並確認實測值是否跟上(ROS),
就把「命令送出去了」變成「命令生效了」——這是 REST 單獨永遠給不出的保證。

⚠️ 但 `low_level_mode` 全零是**歧義**的:它既是「還沒收到第一筆」也是合法的
「PPO / Stand」,而且**不帶任何 freshness 資訊**。任何做這種確認的工具都必須自己處理
這個歧義(例如先確認 `motor_status.timestamp` 在前進),不能直接把全零當成一個讀數。

---

## 3. 提案清單(按價值排序)

### 提案 1:日誌考古 MCP ★ 建議先做

**痛點是實測的。** 這個 stack 的故障幾乎都是跨 pane 的因果鏈:LIO 掉了 → lio_bridge
沒 TF → controller 拒 goal → task_runner 回報失敗。**四個 pane 四種說法**,而且沒有任何
讀取工具(舊的 `scripts/tailog.sh` 已隨 `byobu_session*.sh` 一起刪掉)。加上 §1 那個
`MonitorManager` 只寫 stdout 的事實,日誌是機器人資源歷史的**唯一**來源。

```
list_subsystems()                  → 哪些 pane 有日誌、各自最後一筆的時間
tail(subsystem, lines, level)      → 自動跨 current + @*.s 輪替檔,自動解 gzip
grep_logs(pattern, since, until)   → 跨子系統搜尋
timeline(since, until)             → 把所有 pane 依時間合併成一條軸
resource_history(since)            → 從 sys_manager 的 pane 抽出記憶體 / 磁碟曲線
```

`timeline` 是核心,不是 `tail` 的方便版:它做的是**時間對齊**,而那正是人工最容易做錯
的一步。

- 宿主:機器人本機(需要檔案系統)
- 風險:**零**,純唯讀
- 注意:multilog 的 `@*.s` 是 gzip,`current` 不是;時間戳格式由 multilog 的 `t` flag 決定

### 提案 2:Stack doctor — 開機健檢 MCP

把 `CLAUDE.md` 裡的部落知識變成可執行的**判斷**,而不只是把 graph 倒出來:

```
check_stack()   → 依 start_nav.yaml 的期望清單比對實際節點
                  + TF 鏈 map → <robot_id>/odom → <robot_id>/base_link 是否完整
                  + compute_path_to_pose / follow_path / navigate_to_pose 是否 advertise
                  + 主要 topic 的實際發佈頻率
                  + 依 sleep 順序推斷「最可能的元凶是哪一環」
check_params()  → 節點的 live param 對照 params YAML,列出差異
check_identity()→ node namespace / TF frame prefix 是否都等於 [system] robot_id
```

`check_params` 專治那個已知陷阱:launch 的 `Node` 加了 `name=` 會讓 `planner_server`
和它內部的 `global_costmap` 撞成同一個名字,**內部 costmap 靜默失去全部參數**。這種 bug
目前只能靠人記得。`check_identity` 治的是另一個:TF frame 名稱**不會**被 ROS
namespace,所以 launch 必須顯式覆寫,漏了就會安靜地錯。

- 宿主:機器人本機(需要 rclpy + 看得到 DDS)
- 風險:低(唯讀,但要留意 §5 的 spin 執行緒問題)

### 提案 3:車隊 / 容器維運 MCP

`docker-compose.robots.yml` 已經有 real profile 的 `robot01` / `robot02` 加 sim profile
的三台,但**每台的 backend 各自是一個 `:3000`,沒有任何東西站在它們之上**。這一層天然
是 MCP:

```
list_robots() / robot_state(robot_id)   → 聚合各機 :3000(real 走 host 網路 + mDNS *.local)
dispatch(task, prefer=idle)             → 依電量 / 狀態選機(Temporal queue 本來就依 robot_id 分流)
compose_up / down / logs(service)
rebuild(package)
```

順帶解掉一個實際麻煩:容器重建會清掉手裝的 build 依賴(Sophus / GTSAM 是從源碼編的),
這個復原流程可以包成一個工具。

- 宿主:**開發機 / 車隊側**,不是機器人本機
- 風險:中(`compose down` 會停掉一台真機)

### 提案 4:Temporal 任務考古 MCP

`RobotWorkflow` 有 query 和取消,REST 也有 task / schedule,但「這個任務為什麼卡住」的
答案在 workflow history 裡,目前只有 UI `:8081` 看得到。列出執行中 workflow、解釋失敗的
activity、看 retry 次數、對照 `StepType`(`MOVE` / `ARTIFACT` / `STANDUP` / `LIEDOWN`)。

- 宿主:任何 HTTP/gRPC 到得了 `:7233` 的地方
- 風險:唯讀為主;`reset` / `terminate` 屬於「會動的」等級

### 提案 5:導航調參實驗 MCP(只對 sim 開)

13 個 params YAML,RPP controller 加 costmap inflation 是純試誤。做成 agent 迴圈才有意思:
改 live param → 發一次 `NavigateToPose` → 從實際 topic 收集指標(路徑長度、耗時、最小
障礙距離、`cmd_vel` 抖動)→ 回傳評分。

注意 controller **自己 clamp 線加速度**,stack 裡沒有 velocity smoother,所以 `cmd_vel`
的抖動指標直接反映 controller 參數,中間沒有東西幫它擦屁股。

- 宿主:機器人本機或 sim 容器
- 風險:**高——會動的機器人**。建議硬性限定 sim profile

### 提案 6:地圖生產線 MCP

`pgo/save_maps` → `map.pcd` + `patches/` + `poses.txt` → `tools/pcd_to_gridmap.py` →
`gridmap.pgm` / `.yaml` → vertex(`VertexType` GENERAL / ARTIFACT / CHARGER / HOME /
WAITING)。這條線目前一半在 CLI、一半在前端 map editor,而**最後一步是離線的、事後手動
跑的**——所以一個地圖目錄會有一段時間只有 pcd 沒有 gridmap。

最有價值的能力是 §2 範例 A 的**可達性驗證**。

- 宿主:機器人本機(pcd 檔在那裡)+ REST
- 風險:低(產生新檔案,不覆蓋既有地圖)

### 提案 7:rosbag / LIO 回放 MCP

`doc/record-lidar.md` 的錄包流程(`/livox/lidar`、`/livox/imu`,zstd 壓縮,2 GB split)
目前是手貼指令。錄製、列包、看包內容、拿包重跑 LIO 比對漂移。對 FAST-LIO2 調參有用,
但比較窄。

值得一提的關聯:`pgo_node` 是**唯一**能存地圖的東西,keyframe 全在 RAM,mapping 跑完
沒存就沒了,**事後只能靠回放 bag 補救**。這讓錄包從「調參的方便工具」變成「唯一的保險」。

---

## 4. 反面意見:不該做成 MCP 的東西

**「repo 慣例 lint」不該做成 MCP。** 檢查 subscriber 有沒有寫死絕對 topic、
`use_sim_time` 有沒有被 launch 蓋掉、TF frame 參數有沒有 override——這些用現成的檔案
工具就能做,寫成 skill 或 subagent 比 MCP 直接。唯一值得 MCP 的是**「這個檔案跟 nav2
上游差在哪」**:這是一個 port,一半的問題是「這段是我們改的還是原生的」,而回答它需要
一份上游 checkout,那才是真的外部狀態。

**不要做 `cmd_vel` 連續遙控。** `syncai_driver_manager` 那條 UDP 是**單向、無 ack**,
安全停機路徑靠人 Ctrl-C 那個 pane。把 LLM 放進 10 Hz 速度迴圈是壞主意。單發的 motion
key / nav goal 可以(它們有明確的終止條件),連續速度控制不行。

**不要包 `/api/v1/robot/state` 就當成一個提案。** 那是一對一的 REST proxy,agent 自己
curl 就好。要包就包成「帶判斷」的版本:例如同時檢查 `localization_valid`、
`motor_status.timestamp` 是否在前進、`low_level_mode` 是否有意義,然後回傳一句結論。

---

## 5. 架構決策(實作前必須先定的)

**5.1 server 跑在哪一台,決定了它能做什麼**

三種宿主需求互斥,硬塞進一個 process 會很難看:

| 提案 | 需要 |
|---|---|
| 1、2、6 | 機器人本機的檔案系統 / 行程 / DDS |
| 3 | 跨機視野 + docker socket |
| 4 | 只要到得了 Temporal |

建議:**先開一個新 server 只做提案 1 + 2**(同宿主、唯讀、依賴最少),跑順了再決定 3
要不要獨立成 fleet-side server。

**5.2 rclpy 與 MCP 的執行緒關係**

既有的 pattern 是 `rclpy.spin()` 佔主執行緒、FastMCP 跑背景 daemon thread。任何新的
ROS 工具都必須遵守同一件事:**工具函式不能阻塞 spin**。等 action 結果、等 service
回應、`wait_for_message` 這類操作要走自己的 callback group,否則會鎖死那個它正在等的
callback。這是提案 2 和 5 最容易踩的坑。

**5.3 工具要分級,而且分級要出現在名字裡**

```
read-only     : list_*, get_*, check_*, explain_*, tail, timeline
mutating      : set_*, dispatch, create_*        → 需要確認
destructive   : compose_down, kill_session, delete_*  → 預設不開放
```

`/api/v1/robot/set_motion_key` 的 `'4'` (ESTOP) 就是一個現成的教訓:schema 接受它,但
backend **刻意不轉發**。工具層必須複製這種明確拒絕,而不是安靜地放過去。

**5.4 `robot_id` 是每個工具的隱含參數**

單一 DDS domain 可以有好幾台機器。per-robot server 從 `config/system.ini` 解析一次就好;
fleet-side server(提案 3)**每個工具都得顯式收 `robot_id`**,而且不能猜。

---

## 6. 與既有 `syncai_ros_mcp` 的關係

既有那個 server 是**通用的 ROS graph 反射層**:`get_topics` / `get_services` /
`publish_once` / `subscribe_once` / `call_service`,加上 vertex 與 task 的 REST 薄包裝。
它回答的是「這個 graph 裡有什麼」。

這份文件的提案回答的是**「現在出了什麼問題」**和**「這件事對不對」**。差別是判斷,不是
資料——所以不是取代關係。實作時的判準:如果一個工具只是把某個介面倒出來,它屬於既有
server;如果它需要跨來源關聯、或需要把部落知識編碼成結論,它屬於新的。

---

## 7. 建議的落地順序

1. **提案 1**(日誌考古)——唯讀、零風險、當天就有回報,而且補上一個已經被刪掉的工具
2. **提案 2**(stack doctor)——同宿主,把 `CLAUDE.md` 的知識變成可執行的判斷
3. 觀察 agent 實際靠這兩個解掉了什麼,再決定 **3 / 4** 誰先
4. **提案 5**(調參)最後,而且只對 sim

---

## 8. Agent 串接:deepagents(LangChain)

上面所有提案講的都是「工具長什麼樣」;這一節回答「誰來呼叫這些工具」。結論:
**互動式診斷不需要寫任何程式**——Claude Code 本身就是 MCP client:

```bash
claude mcp add --transport http syncai http://robot01.local:8000/mcp
```

deepagents 值得寫的場景是**嵌入式 / 自動化 agent**:排程健檢、operator console
內建的對話式診斷、backend 觸發的自動故障分析。先確定「誰在什麼時候呼叫這個
agent」,再決定要不要自己養一個 harness。

### 8.1 為什麼是 deepagents

三個理由,都直接對到這份文件既有的設計:

1. **MCP 接線是一行 config。** deepagents 透過 `langchain-mcp-adapters` 的
   `MultiServerMCPClient` 吃 MCP tools,支援 streamable HTTP——`syncai_ros_mcp`
   正是 FastMCP over HTTP(port 8000),直接對上。
2. **`interrupt_on` 直接落地 §5.3 的工具分級。** read-only / mutating /
   destructive 三級制可以宣告式地寫成核准政策,不用自己寫 gate。
3. **planning + subagents 適合提案 1、2 的任務形狀。**「任務為什麼卡住」是跨
   日誌 / DDS / Temporal 的多步驟因果鏈,正是它 todo-planning 的用途;它的
   virtual filesystem 也能承接大段日誌輸出,不把主 context 撐爆。

架構上乾淨的一點:agent process 只跟 `:8000` 講 HTTP,**完全不碰 rclpy**,
所以 §5.2 的 spin 執行緒問題與它無關——那始終是 MCP server 側的責任。agent
可以跑在開發機或 fleet 側任何到得了機器人的地方。

### 8.2 最小接線

```python
import asyncio
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.checkpoint.memory import MemorySaver
from deepagents import create_deep_agent

async def main():
    client = MultiServerMCPClient({
        "robot01": {
            "transport": "http",
            "url": "http://robot01.local:8000/mcp",  # mDNS 容器已經解得到
        },
    })
    tools = await client.get_tools()

    agent = create_deep_agent(
        model="anthropic:claude-opus-5",
        tools=tools,
        system_prompt="你是 SyncAI 四足機器人的維運助理...",
        interrupt_on=INTERRUPT_POLICY,   # 見 8.3
        checkpointer=MemorySaver(),      # HITL 必需;上生產換 Postgres checkpointer
    )
    result = await agent.ainvoke(
        {"messages": [{"role": "user", "content": "檢查 robot01 現在的導航狀態"}]},
        config={"configurable": {"thread_id": "1"}},
    )

asyncio.run(main())
```

模型字串用 `anthropic:claude-opus-5`。checkpointer 上生產要換持久化的
Postgres checkpointer——這個 stack 剛好已經有 Postgres(`:5432`)。

### 8.3 工具分級 → `interrupt_on` 政策

§5.3 的分級直接翻譯:

```python
INTERRUPT_POLICY = {
    # read-only:不攔
    "get_topics": False, "tail": False, "timeline": False, "check_stack": False,
    # mutating:要人核准
    "create_task":    {"allowed_decisions": ["approve", "reject"]},
    "set_motion_key": {"allowed_decisions": ["approve", "edit", "reject"]},
    # destructive:核准制,或乾脆不暴露
    "switch_mode":    {"allowed_decisions": ["approve", "reject"]},
}
```

進階:`when` predicate(langchain ≥ 1.3.3)可以做條件式攔截,例如只攔
`set_motion_key` 中 key 為危險值的呼叫,其餘放行。

⚠️ **`interrupt_on` 是 UX,不是安全邊界。** 攔截發生在 agent client 側;任何
直接打 `:8000` 的東西都繞得過去。§5.3 那個教訓(backend 刻意不轉發 ESTOP
`'4'`)的正確位置仍然在 **server 側**——deepagents 的核准機制是疊在上面的
第二層,不是替代。

### 8.4 多機器人的坑

`MultiServerMCPClient` 同時掛 robot01 / robot02 時,兩邊的 `get_topics`、
`tail` **同名衝突**。最省事的做法是**一台機器人一個 agent instance**——跟
Temporal task queue 按 `robot_id` 分流的哲學一致。要單一 agent 管全 fleet,
就回到 §5.4:fleet-side server 每個工具顯式收 `robot_id`,而不是把多個
per-robot server 疊在同一個 client 裡。

### 8.5 與落地順序的配合

跟 §7 的順序天然配對:提案 1、2 全是唯讀工具,先做 agent + 全部
`interrupt_on: False`,零風險跑起來驗證價值;等到要暴露 mutating 工具
(create_task、switch_mode)時,checkpointer + interrupt 的骨架已經在了,
只是加幾行分級設定。注意 HITL 的 approve / reject flow 需要 UI 端點——
operator console(Next.js frontend)是自然的落點,這是接 mutating 工具前
要先做的一件事。
