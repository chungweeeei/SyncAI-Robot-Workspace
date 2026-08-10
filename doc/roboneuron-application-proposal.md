# RoboNeuron 機制在本 workspace 的應用提案

> 對象:`src/syncai_ros_mcp/`(既有的 MCP runtime server)
> 相關:`doc/mcp-server-proposals.md`(另一組提案,關係見 §7)
> 論文:RoboNeuron: A Middle-Layer Infrastructure for Agent-Driven Orchestration
> in Embodied AI(arXiv:2512.10394v2,中科院自動化所)
> 狀態:**提案,尚未實作**。

RoboNeuron 是一層接在「LLM agent 的 MCP tool calling」與「ROS2 middleware」之間的
基礎設施。這份筆記回答一個問題:**它的哪些機制值得搬進這個 workspace,哪些不值得**。

先說結論:這個 stack 其實已經有 RoboNeuron 的骨架——`syncai_ros_mcp` 就是它的
control plane,Temporal backend 承擔了 lifecycle 治理。真正值得補的是三個機制:
**schema-based tool derivation、typed 能力工具、以及「穩定邊界內切換」的語意**。
而且有一個現成的完美對應:**gait policy 切換就是我們的 inference switching**。

---

## 0. 論文機制速覽(只列會用到的)

| 機制 | 論文的做法 | 一句話 |
|---|---|---|
| Schema-based tool derivation(Alg. 1) | 從 ROS message 定義自動推導 tool signature,註冊 `(name, Σ, Encoder, Publisher)` 到 registry | agent 看到的是 typed tool,不是萬用 publish |
| Direct path | tool call → 驗證 → 編碼 → 發佈,支援帶 step duration 的短命令序列 | 一次性低延遲原語 |
| Closed-loop path(PIC) | perception–inference–control 三模組,topic 串接,固定 action contract | 長時閉環行為 |
| Lifecycle control | 每個長時模組獨立 OS process(`spawn`),stop tool 先 bounded wait 再強制終止 | 閉環是被治理的服務,不是沒人管的背景任務 |
| Stable inference boundary | VLA 專屬邏輯關在 inference 模組內,換 backend / runtime / 加速 preset 不動周邊 topic 接線 | topology-preserving switching |

論文自己的定位聲明很重要:它**不是** task planner,編排交給外部 LLM agent。這跟我們的
架構剛好同構:LLM agent → MCP → Temporal workflow → BT navigator。所以套用它不需要
動任何任務編排的設計。

---

## 1. 對照表:論文概念 ↔ 本 stack 現狀

| RoboNeuron 機制 | 我們的現狀 | 缺口 |
|---|---|---|
| MCP 統一 tool 介面 | ✅ `syncai_ros_mcp`(FastMCP, port 8000) | 已有 |
| Schema-based tool derivation | ❌ 只有萬用 `publish_once(topic, msg_type, dict)` | **最大缺口,見 §2** |
| Direct path(驗證→編碼→發佈) | ⚠️ 有,但無 typed 驗證、無序列發佈 | §4 |
| Closed-loop path + lifecycle | ⚠️ Temporal task(create/get/cancel)+ byobu `switch_mode` | §5,粒度粗但骨架在 |
| Stable boundary / backend switching | ⚠️ `SetPolicyMode`(PPO/HIMLOCO/CHAMP/ISSAC)存在但沒暴露給 agent | **就差一層,見 §3** |
| Stop tool(bounded wait → 強制) | ⚠️ driver_manager 有 safe-shutdown 路徑,但不是 agent tool | §4 |
| PIC(perception–inference–control) | ❌ 無相機、無 VLA(`src/` 裡沒有任何 torch/onnx/Image 消費者) | **暫不適用,見 §6** |

---

## 2. 提案 A:Schema-based Tool Derivation → 新增 `tools/registry.py` ★ 核心

這是論文的核心貢獻,也正好打中既有實作的三個痛點。現在 agent 要讓機器人動,得這樣:

```
publish_once(topic='/robot01/cmd_vel', msg_type='geometry_msgs/msg/Twist',
             msg={'linear': {'x': 0.3}})
```

1. **agent 必須自己拼絕對 topic 名**——直接違反 CLAUDE.md 的鐵律(never hardcode
   `/<robot_id>/…`)。根本原因是 `mcp_server_node.py` **沒有 namespace**,agent 被迫
   用絕對名。這不是 agent 的錯,是工具面逼它犯規。
2. **沒有 argument schema**——`set_message_fields` 失敗才報錯,agent 只能瞎猜欄位。
3. **每次呼叫都要 agent 重新想 msg_type 字串**——正是論文說的 interface drift 來源。

### 做法(對應論文 Algorithm 1)

- 新增 capability manifest,建議 `config/capabilities.yaml`。每個 entry:
  `{tool_name, topic(相對名), msg_type, description, qos}`。放 config 而不是寫死在
  Python,跟 `config/sessions/*.yaml` 的「window list 是 data」哲學一致。
- `registry.py` 啟動時對每個 entry:`get_message()` →
  `get_fields_and_field_types()` 遞迴展開(`topics.py` 的 `get_message_details`
  **已經寫好這段遞迴**,抽出來重用,不要抄一份)→ 動態生成 pydantic model 當
  argument schema → 建**常駐** publisher 綁 relative topic → `mcp.tool()` 註冊。
- **把 MCP node 放進 `robot_id` namespace**(跟其他 launch 一樣讀
  `config/system.ini`)。registry 的 relative topic 自動解析到 `/<robot_id>/…`,
  agent 從此不知道 namespace 的存在。這正是論文 Case I「同一個 velocity tool 綁到
  不同平台」在我們 fleet 上的意義:**同一份 manifest,robot01 / robot02 各跑一個
  MCP server,tool 面完全相同**。

附帶收益:registry entry 帶 per-topic QoS,順手解掉 `topics.py` 裡那個已註記的
TODO——map topic 需要 TRANSIENT_LOCAL 卻被寫死 VOLATILE。

萬用的 `publish_once` / `subscribe_once` **留著**當 escape hatch:論文 Fig. 1 也保留
了通往低層的 Direct Path。判準:manifest 裡有的能力走 typed tool,沒有的才落回萬用
工具。

第一批 manifest entries:`cmd_vel`(Twist)、relocalize 的 initial pose,加上 §3 的
locomotion 工具(那些是 service,不走這個 topic registry,但共用「typed + 能力卡」
的呈現方式)。

## 3. 提案 B:gait policy 切換 = 我們的 topology-preserving switching → 新增 `tools/locomotion.py`

論文的 "topology-preserving inference switching" 是:換 backend,但觀測流、action
contract、下游接線全部不動。我們的 gait controller 就是這個結構:

- `cmd_vel` 進、步態出——固定契約
- `SetPolicyMode`(0 PPO / 1 HIMLOCO / 2 CHAMP / 3 ISSAC)——**backend switch**,
  切 RL policy 完全不動 nav stack 的任何接線
- `SetMotionKey` / `SetSpeedScale`——runtime preset

它們目前只有萬用 `call_service` 摸得到。建議做成 typed tools:

```
set_locomotion_policy(policy)   # 送 SetPolicyMode 並回讀實測值,見下
set_motion_state(key)           # 同上
emergency_stop()                # 對應論文 Case I 的 stop_base 能力卡
```

### 必須寫進 tool 行為的 caveat:COMMANDED vs MEASURED

這些 service 底層是**單向 UDP、無 ack**(`udpSend()` 丟棄 `sendto()` 的回傳),回
`success` 只代表「送出去了」。`RobotLowLevelMode.msg` 的註解已經把這件事寫得很透。
所以 tool 的正確設計是:呼叫後**訂一次 `mode` topic(或讀
`RobotState.low_level_mode`)回報實測值**,agent 拿到的是 commanded + measured 兩個
值。這比論文做得更誠實——論文的 backend switch 是進程內的,我們的跨了一條不可靠
鏈路。

⚠️ 兩個既有文件已記錄的坑,工具必須處理而不是放過:

- `low_level_mode` 全零是歧義的(「還沒收到第一筆」vs 合法的「PPO / Stand」),且無
  freshness 資訊。回讀前先確認 `motor_status.timestamp` 在前進。
- `policy_state` 沒有 sentinel;`motion_state == 8` 是 controller 自己的 UNKNOWN。
  表外的整數(例如 MPC)要原樣傳回,不 clamp、不當錯誤——`RobotLowLevelMode.msg`
  對此有明確的設計決策。
- ESTOP:backend 的 `set_motion_key` schema 接受 `'4'` 但**刻意不轉發**
  (`doc/mcp-server-proposals.md` §5.3 記錄過)。`emergency_stop()` 要走 ROS service
  那條路,而且是獨立工具、不藏在 `set_motion_state` 的參數空間裡。

## 4. 提案 C:序列發佈 + 停止語意 → direct path 補完

論文 III-B:direct path 支援「帶 step duration 的短命令序列」,結尾有明確終止。對
四足這不是 nice-to-have:單發一個 Twist 之後的行為取決於 gait controller 的
watchdog,agent 一次 `publish_once` 要嘛沒效果、要嘛效果不可控。

```
move_base_timed(vx, wz, duration_s)   # 10 Hz 連續發佈,時間到自動發零速 Twist
```

registry tool 支援 optional 的 `sequence: [{msg, duration}]`,**結尾永遠補一個
zero-Twist**——把論文的 scripted motion 和 stop 語意合在一起。

### 與 `mcp-server-proposals.md` §4 的張力,以及為什麼不衝突

那份文件明確反對「`cmd_vel` 連續遙控」,理由是把 LLM 放進 10 Hz 速度迴圈是壞主意。
這個提案**不是**那個東西,判準正好是那份文件自己給的:「單發的 motion key / nav goal
可以(它們有明確的終止條件)」。`move_base_timed` 的終止條件在**工具內部**——
duration 用完、zero-Twist 收尾,LLM 不在迴圈裡,它只發起一次有界的動作。真正被禁止
的是「agent 每 100 ms 決定一次速度」,那個仍然不做。

儘管如此,仍建議兩道護欄:`duration_s` 上限(例如 5 s)寫死在工具裡;真機 profile
下要求 manifest 顯式開啟這個工具(sim 預設開)。

## 5. 提案 D:lifecycle 暴露,而不是重造 spawn 機制

論文用 `spawn` + stop tool 管長時模組。**不需要照抄**——stack 已有兩層現成的
lifecycle,只是 agent 摸不到:

- **粗粒度**:`NodeManager` 的 byobu session。加 `tools/lifecycle.py`:
  `get_robot_mode()` / `switch_robot_mode(mode)` 包 `GetMode` / `SwitchMode` service
  client。tool description 必須帶上論文式的語意警告:switch 是毀滅性長操作(~40 條
  byobu 命令);MANUAL 切走會丟掉未存的地圖(`pgo_node` keyframes 在 RAM,
  `save_maps` 是唯一的序列化路徑);MAINTENANCE 不可切入。這些 `SwitchMode.srv` 的
  註解裡都有,搬進 description 就好。
- **細粒度(閉環任務)**:`tasks.py` 的 create/get/cancel 走 Temporal,這**已經是**
  論文說的 "explicitly governed system service, not an unmanaged background task"
  ——workflow query 就是論文的 "agent monitors progress",cancellation 就是 stop
  tool。**這塊不用動**,是我們比論文原型更強的地方。

## 6. 反面意見:暫時不要做 PIC

PIC 的前提是有視覺流 + VLA policy。這個 stack 目前**沒有相機、沒有 ROS 側的模型推理**
(G23 的 RL policy 跑在 gait controller 上,不在 ROS 側)。硬套 PIC 沒有掛載點。

但值得**預留契約**:未來若加 VLA(語意目標導航、機械臂),照論文的做法先定死 action
contract topic(論文用 `Float64MultiArray` 載 6-DoF delta + gripper),把推理模組關
在邊界裡。到時 perception = 新增相機 driver node,control = 既有的
controller / task_runner,只補中間那格,周邊不重接。

另一個論文沒有、但我們遲早需要的擴充:**action tools**。論文自承只做了 topic-based
exposure(service / action 是 future work),而這個 stack 的核心入口偏偏是 action
(`NavigateToPose`、`ExecuteTask`)。目前 agent 只能繞道 backend REST。若要讓 agent
直接下導航目標(不經 Temporal 排程),得自己寫 send_goal / feedback / cancel——超出
論文範圍,且 Temporal 路徑夠用的話可以放最後。實作時注意
`mcp-server-proposals.md` §5.2 的執行緒鐵律:等 action 結果不能阻塞 spin。

---

## 7. 與 `mcp-server-proposals.md` 的關係

兩份文件互補,不重疊:那份回答「**出了什麼問題**」(日誌考古、stack doctor、跨來源
關聯),屬於診斷面;這份回答「**agent 怎麼把機器人當成一組 typed 能力來用**」,屬於
控制面。共用的判斷已交叉引用:commanded vs measured 的雙重驗證(那邊 §2 範例 C =
這邊 §3)、工具分級與 ESTOP 的明確拒絕(那邊 §5.3)、rclpy spin 執行緒(那邊 §5.2)。

實作判準:工具只是把某個介面 typed 化 → 屬於這份;工具需要跨來源關聯或把部落知識
編碼成結論 → 屬於那份。

---

## 8. 建議的落地順序

1. **提案 A**(registry + manifest + MCP node 加 namespace)——一次解掉 typed
   tools、namespace 違規、QoS TODO 三件事
2. **提案 B**(locomotion,帶 measured-state 回讀)——工作量小、示範性最強,這就是
   我們的 topology-preserving switching demo
3. **提案 C**(timed sequence + zero-Twist 收尾)——安全性
4. **提案 D**(get_mode / switch_mode)
5. Action tools(視需求,最後)

A + B 合計約 300–400 行 Python,全部落在 `syncai_ros_mcp` 這個 vendored package 裡,
不碰 C++ nav stack 一行——符合論文的定位:middleware 層加東西,既有 control stack
完全不重接。
