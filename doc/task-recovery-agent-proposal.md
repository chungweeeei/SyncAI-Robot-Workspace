# 任務自癒迴圈提案:失敗 → 歸因 → 調參 → 重試

> 對象:`src/syncai_device_agent/`(deepagents runtime)+ `syncai_backend` 的 Temporal
>       `RobotWorkflow`
> 相關:`doc/deep-agent-proposal.md`(agent 側接線 / tool vs skill 的判準,本文沿用)
>       `doc/mcp-server-proposals.md` §2 範例 B(`explain_task` 四來源因果鏈)
>       `doc/gridmap-tuning-agent-proposal.md`(同構迴圈的離線安全版,建議先做那個)
> 狀態:**提案,尚未實作**。所需的兩個工具(失敗脈絡 / 參數讀寫)在 `syncai_ros_mcp`
>       都還不存在,見 §2。

這份筆記回答一個問題:「任務失敗後,讓 agent 查日誌、分析、調參數、再跑一次」這個迴圈
做不做得到。

先說結論:**做得到,但成功的版本不是「一個會自己跑迴圈的 agent」。** 迴圈歸 Temporal,
判斷歸 agent,執行權歸白名單 —— 三者分開,這個題目才成立。而且真正的難點不在 agent
框架,在**歸因**:大多數任務失敗根本不是調參數能解決的,一個會反射性調參的 agent 會在
那些情況下把參數改壞、而問題還在。

---

## 0. 心智模型:agent 不管迴圈

最容易寫錯的版本,是讓 agent 自己 while 迴圈:

```
❌ agent: 查日誌 → 調參 → 重跑 → 沒過 → 再查 → 再調 → …
```

LLM 管狀態機不可靠:會無限重試、會忘記自己試過什麼、process 掛掉整個迴圈就斷,而且沒有
任何一步是可稽核的。

正確的分法是把控制權留在 Temporal —— `RobotWorkflow` 本來就在做這件事(依序執行 step、
依 `StepType` 分派、暴露 workflow query、支援取消,task queue 依 `robot_id` 分流):

```
Temporal RobotWorkflow  ←── 管迴圈:重試次數、退避、逾時、狀態持久化、可取消
        │
        │  step 失敗 → 呼叫一個 activity
        ▼
   DiagnoseActivity ──→ deepagent(單次呼叫,不是迴圈)
        │                輸入:結構化失敗脈絡
        │                輸出:一個結構化決定
        ▼
   { action: "retry_as_is" | "adjust_param" | "abort_and_escalate", … }
```

**Agent 只當決策節點:被呼叫一次、給一個判斷、結束。** 這樣才有重試上限、每次嘗試的完整
審計紀錄、中途可取消,而且 agent 掛掉不會弄壞 workflow。

這也符合 `deep-agent-proposal.md` §0 的分層:agent process 只講 HTTP,不碰 rclpy、不進
DDS domain,所以它可以跑在 fleet 側,被 Temporal activity 用 HTTP 叫起來。

---

## 1. 第一個要想清楚的是歸因,不是調參

這是整份提案的核心。任務失敗大致分五類,**只有最後一類調參數有意義**:

| 類型 | 這個 stack 的典型樣態 | 調參數有用嗎 | 正確動作 |
|---|---|---|---|
| 環境暫時性 | 路被人擋住、門關著、障礙物暫留在 costmap | ❌ | 退避後原樣重試,**參數不要動** |
| 定位 | LIO 漂移、`map → odom` 修正跳動、初始位姿沒設(`UNINITIALIZED`) | ❌ | 重定位 / 重設初始位姿 |
| 硬體 | `syncai_driver_manager` 的 UDP telemetry 斷、電量 <20%(`WARNING`) | ❌ | 停止並升級給人 |
| 地圖 / 任務設定 | 目標 vertex 落在 keepout filter 內、根本不可達 | ❌ | 改任務或改地圖,不是改參數 |
| **真的是參數** | inflation 太厚導致窄道規劃不出路徑、lookahead 讓過彎超調、goal checker 容忍度過嚴而 `FollowPath` 逾時 | ✅ | 單一參數微調後重試 |

⚠️ 如果 agent 對每種失敗都反射性地「調參數再試一次」,它會在前四類**把參數改壞,而且
原問題還在**。這比不修更糟,因為現場會多出一個沒人知道的變因。

所以這個 agent 最重要的能力不是「會調參」,而是**會分類、而且敢說「這個我不該碰」**。
skill / system prompt 的篇幅應該大部分花在「什麼時候不要動參數」,預設輸出應偏向
`retry_as_is` 或 `abort_and_escalate`,`adjust_param` 是需要舉證的例外。

`mcp-server-proposals.md` §2 範例 B 提到的那條部落知識(`low_level_mode` 全零是歧義的,
要先確認 `motor_status.timestamp` 在前進)正是這一層的東西 —— 它屬於 SOP,不屬於任何
單一 tool。

---

## 2. 缺口:兩類工具都還不存在

`syncai_ros_mcp` 現有的工具是 topic / service / task / map 四組(`get_topics`、
`get_topic_details`、`publish_once`、`subscribe_once`、`get_services`、`call_service`、
`create_task`、`get_task_state`、`cancel_task`、maps)。這個迴圈需要的兩類**都沒有**:

| 需要 | 現況 | 建議做法 |
|---|---|---|
| 讀失敗脈絡 | ❌ 只有 `get_task_state`,拿得到狀態拿不到死因 | `get_task_failure(task_id)`,見下 |
| 讀 / 改參數 | ❌ 只能用 `call_service` 硬打 `/set_parameters`,agent 不好用也不安全 | `get_params(node)` / `set_param(...)`,見 §3 |

### 2.1 先用 Temporal history,不要先做日誌 parsing

直覺會想「查日誌」,但日誌應該是**第二層**。第一層是 Temporal 的 workflow history —— 那是
**結構化的失敗資料**:哪一個 step、什麼 `StepType`、第幾次重試、什麼時候失敗。讓 LLM 去
grep `log/stack/<robot_id>/<name>/` 底下 16 MiB × 10 gzip 輪替的 multilog,又貴又不準,
而且那些輸出會直接撐爆 context。

建議的工具形狀:

```
get_task_failure(task_id) -> {
  failed_step: {index, type, target_vertex, started_at, failed_at},
  attempt: 1,                       # workflow 已經重試過幾次
  nav_result_code: ...,             # MOVE step 才有
  robot_state_at_failure: {...},    # 電量 / state / pose 是否有效
  log_window: {node, from, to},     # ← 只給座標,不給內容
}
```

`log_window` 只回傳「該去哪裡撈」,agent 判斷需要時再呼叫 `tail_logs` 拿實際文字。這是
漸進揭露,配合 deepagents 內建的 filesystem 工具把大輸出落到檔案而不是 context。

與 `mcp-server-proposals.md` §2 範例 B 的關係:那個 `explain_task` 是**給人看的敘事**,
這個 `get_task_failure` 是**給 workflow 吃的機器可讀版**。兩者共用同一組資料來源,值得
一起實作、共用內部函式,但輸出形狀不同,不要合併成一個工具。

---

## 3. 哪些參數真的能動態改(已查證)

好消息是這個 stack 的動態參數支援比預期完整。以下是原始碼查證結果:

| 節點 / plugin | 動態參數 | 證據 |
|---|---|---|
| Regulated Pure Pursuit | ✅ 22 個 | `plugins/regulated_pure_pursuit_controller/…cpp:210` |
| controller_server 本體 | ✅ | `src/controller_server.cpp:205` |
| goal checker / progress checker | ✅ 全部四個 plugin | `plugins/*_goal_checker.cpp`、`*_progress_checker.cpp` |
| costmap(含 obstacle / inflation / static layer) | ✅ | `costmap_2d_ros.cpp:274` 及各 layer |
| smac_planner_2d | ✅ | `plugins/smac_planner/smac_planner_2d.cpp` |
| **`syncai_backend` 的 ROS 參數** | ❌ **要重啟** | CLAUDE.md「Changing a backend ROS parameter requires restarting the backend」 |

⚠️ **但「動態可調」不等於「應該開放給 agent 調」。** RPP 的動態清單裡包含
`desired_linear_vel`、`max_linear_accel`、`max_angular_accel` —— 正因為它們改得動,才必須
在白名單裡**明確排除**。這些直接決定一台四足機器人的動能,人工專屬。

建議的初版白名單(保守,寧可太窄):

| 參數 | 上下限 | 適用失敗樣態 |
|---|---|---|
| `inflation_layer.inflation_radius` | 依機身尺寸給區間 | 窄道規劃不出路徑 |
| `<goal_checker>.xy_goal_tolerance` | 上限鎖死 | `FollowPath` 到點判定過嚴而逾時 |
| `<goal_checker>.yaw_goal_tolerance` | 上限鎖死 | 同上 |
| `RPP.lookahead_dist` / `min_` / `max_` | 窄區間 | 過彎超調 / 貼牆 |

`desired_linear_vel`、`max_*_accel`、`allow_reversing`、以及任何 costmap 的 topic /
frame 類參數:**永不開放**。

---

## 4. 護欄(六條,從第一版就要有)

1. **白名單**:只有 §3 表列參數可改,其餘一律拒絕。守門邏輯寫在 `set_param` 工具**內部**,
   不是寫在 prompt 裡。
2. **範圍上下限**:每個參數帶 min/max,越界直接拒絕並回報。
3. **一次只改一個參數**:否則出事無法歸因。
4. **重試次數上限**:同一任務最多自動嘗試 2 次,之後一律 `abort_and_escalate`。這條由
   Temporal 執行,不靠 agent 自律。
5. **必回滾**:任務結束(成功或放棄)一律把參數還原。**絕不允許 agent 的臨時調整永久留在
   系統裡** —— 否則三個月後沒人知道現場的參數為什麼跟 repo 裡的不一樣。
6. **全程留痕**:每次 `adjust_param` 寫一筆(任務、失敗原因、參數、前後值、結果),這是
   之後回頭檢討 agent 判斷準不準的唯一依據。

⚠️ 沿用 `deep-agent-proposal.md` §4 的警告:`interrupt_on` 是 UX 不是安全邊界。護欄 1、2
的正確位置在 **MCP server 側的 `set_param` 工具內**,agent 側的核准機制是疊在上面的第二層。

---

## 5. Agent 的輸出必須是結構化的

deepagents 的 `response_format` 可以強制輸出 schema,不要讓它回一段散文給 workflow 解析:

```python
class RecoveryDecision(BaseModel):
    action: Literal["retry_as_is", "adjust_param", "abort_and_escalate"]
    category: Literal["transient", "localization", "hardware", "map_or_task", "tuning"]
    reason: str                      # 給人看的一句話
    changes: list[ParamChange] = []  # 只有 action == adjust_param 時非空
    confidence: Literal["low", "medium", "high"]
```

`category` 欄位不只是給人看的 —— 它讓你事後能統計「agent 把多少 transient 誤判成
tuning」,那是決定要不要進 §6 Phase 2 的關鍵數字。

---

## 6. 三階段落地

| 階段 | 做什麼 | 風險 | 出場條件 |
|---|---|---|---|
| **Phase 0** | 只診斷。失敗時產出分析 + 「我本來會怎麼做」,**不執行**。跑兩週 | 零 | 累積到足夠案例,誤判率可量化 |
| **Phase 1** | 建議 + 人工核准。`set_param` 掛 `interrupt_on`,人在迴圈裡按確認 | 低 | 連續 N 次核准都是「同意」 |
| **Phase 2** | 白名單內自動,護欄六條全開 | 中 | — |

Phase 0 的真正產出不是修好的任務,是**資料**:你會知道它的歸因準不準、哪一類失敗它會誤判。
沒有這份底氣就直接讓它改參數,是在賭。

Phase 1 需要 HITL 的 approve/reject UI —— `deep-agent-proposal.md` §4 已指出 operator
console(Next.js frontend `:3001`)是自然落點。這是接任何 mutating 工具前的共同前置。

---

## 7. 與其他 proposal 的關係

| 文件 | 回答 |
|---|---|
| `mcp-server-proposals.md` | 出了什麼問題 / 這件事對不對(診斷面,MCP server 層) |
| `roboneuron-application-proposal.md` | 機器人怎麼被當成一組 typed 能力(控制面,MCP server 層) |
| `deep-agent-proposal.md` | 誰來呼叫這些工具、tool vs skill 怎麼分(agent 層,**接線**) |
| **本文件** | **agent 拿這些工具做什麼:一個會自癒的任務迴圈(agent 層,應用)** |
| `gridmap-tuning-agent-proposal.md` | 同一個迴圈形狀的離線版,零實體風險 |

實作順序上,本提案**不建議當第一個做的 agent**。它的迴圈形狀(執行 → 量測 → 歸因 →
調參 → 重試 → 收斂或放棄)與 `gridmap-tuning-agent-proposal.md` 完全同構,差別只在那裡的
「執行」是跑一次離線投影,這裡的「執行」是一台四足機器人走出去。**先在離線題目把骨架、
收斂條件、放棄條件、白名單機制練熟**,搬過來時就只剩安全問題要煩惱,不必同時煩惱架構。
