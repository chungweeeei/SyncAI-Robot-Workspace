# Deep Agent 應用提案:讓 agent 和機器人產生關聯

> 對象:`src/syncai_device_agent/`(新的 deepagents runtime,目前只有骨架)
> 相關:`doc/mcp-server-proposals.md` §8、`doc/roboneuron-application-proposal.md` §8
>       —— 那兩份從**工具 / server 側**切入,這份從 **agent 側**切入,見 §7
> 框架:`deepagents==0.7.5` / `langchain==1.3.14` / `langgraph==1.2.11`(已裝在
>       `src/syncai_device_agent/.venv`)
> 狀態:**提案,尚未實作**。`main.py` 現在是一個 web_search 研究員範例,`tools.py` 空檔。

這份筆記回答三個實作前必須先想清楚的問題:要不要為機器人 define tools、tool 能不能
「混合 HTTP + ROS」、以及 deepagents 的 skills 該怎麼定義。

先說結論:**要 define tools,但混合 HTTP + ROS 的那個 tool 不住在 agent 這一側。**
deepagent process 只講 HTTP,ROS 的複雜度全部關在 MCP server(`:8000`)後面 —— 這不是
偏好,是 `mcp-server-proposals.md` §5.2 的 spin 執行緒鐵律推出來的結構。而 **skills 不是
「被 call」的東西**,它是把多步驟 SOP 編碼成 `SKILL.md` 的操作手冊,和 tool(原子能力)
是兩個層次。

---

## 0. 心智模型:這裡有兩層,別混在一起

```
┌─ deepagent process(跑在開發機 / fleet 側)────────────┐
│   create_deep_agent(tools=[...], skills=[...])          │
│   ★ 只講 HTTP,永遠不 import rclpy                       │
└──────────────────────┬──────────────────────────────────┘
                       │  HTTP
          ┌────────────┴────────────┐
          ▼                         ▼
   MCP server :8000            backend REST :3000
   (syncai_ros_mcp,跑在機器人本機)   (syncai_backend, FastAPI + Postgres)
   ★ 擁有 rclpy / DDS
   ★ ROS + REST 混合發生在「這一層的單一 tool 內部」
```

`roboneuron-application-proposal.md` §8.1 已經寫死這條界線:**agent process 只跟 `:8000`
講 HTTP,完全不碰 rclpy**,所以 §5.2 的 spin 執行緒問題與 agent 無關 —— 那始終是 MCP
server 側的責任。這一句是本文件三個問題的共同答案,底下只是把它展開。

**為什麼 agent 側不直接碰 ROS?** 技術上你當然可以在 agent 同一個 process 裡寫一個
`@tool` 直接 `import rclpy`。但那會把 agent:

1. **綁進 DDS domain** —— agent 就不能跑在 fleet 側了,得跟機器人同網段;
2. **逼你在 agent process 裡處理 `rclpy.spin()` 執行緒**(`mcp-server-proposals.md`
   §5.2 那個坑:等 action / service 不能阻塞 spin)。

這違背乾淨分層。正確做法:凡是需要碰 ROS 的,就在 MCP server 上做成一個 typed tool,
agent 只透過 HTTP 呼叫它。

---

## 1. 三種 tool 來源(agent 側全是 HTTP)

要讓 agent 和機器人有關聯,就是給它這三類 tool。關鍵是**先想清楚每個 tool 住在哪一層**。

| 來源 | 底層 | 怎麼進到 agent | 判準 |
|---|---|---|---|
| **(a) MCP tools** | 可以是 ROS,也可以是 server 側的 REST/混合 | `langchain-mcp-adapters` 的 `MultiServerMCPClient` 自動拉進來 | 需要碰 ROS、或需要跨來源關聯 → 走這裡 |
| **(b) 直接 REST `@tool`** | 純 HTTP 打 `:3000` | 自己用 `@tool` 包 httpx/requests | 只是查歷史 / 意圖,不碰 ROS |
| **(c) built-in / 第三方** | 視工具而定 | deepagents 內建 + `tools=[...]` | web_search、檔案工具等 |

### (a) MCP tools —— 不是你寫的,是拉進來的

```python
from langchain_mcp_adapters.client import MultiServerMCPClient

client = MultiServerMCPClient({
    "robot01": {"transport": "http", "url": "http://robot01.local:8000/mcp"},
})
mcp_tools = await client.get_tools()   # cmd_vel / check_stack / verify_vertices … 全在這
```

⚠️ **依賴缺口**:`pyproject.toml` 目前只有 `deepagents / dotenv / requests / structlog`,
**沒有 `langchain-mcp-adapters`**。要走 (a) 就得先補這個依賴。

### (b) 直接 REST `@tool`

`mcp-server-proposals.md` §0 說得很清楚:把 REST endpoint 一對一包成 MCP tool 是最沒價值
的做法 —— agent 本來就能發 HTTP。所以純歷史查詢直接在 agent 側用 `@tool` 包就好,不必
繞 MCP server:

```python
from langchain_core.tools import tool
import requests

@tool
def get_map_vertices(map_name: str) -> dict:
    """讀取指定地圖的所有 vertex(GENERAL/ARTIFACT/CHARGER/HOME/WAITING)與座標。"""
    return requests.get(f"http://robot01.local:3000/api/v1/maps/{map_name}").json()
```

然後 merge:

```python
agent = create_deep_agent(
    model="anthropic:claude-opus-4-8",
    tools=[*mcp_tools, get_map_vertices],
)
```

---

## 2. 「混合 HTTP + ROS」的 tool 住在 server 側,不住在 agent 側

這是最容易搞反的一點。從 agent 的角度看,它呼叫的**每一個** tool 都是 HTTP(要嘛打
`:8000` MCP,要嘛打 `:3000` REST)。真正「一個 tool 內部同時 call REST + call ROS」的,
是 **MCP server 上**的工具函式 —— 這正是 `mcp-server-proposals.md` §2 的三個範例:

```
verify_vertices(map_name)   # server 內部:REST 拿 vertex 座標 + ROS 對每點跑 ComputePathToPose
explain_task(task_id)       # server 內部:REST + Temporal history + ROS result code + 日誌
set_locomotion_policy(p)    # server 內部:送 SetPolicyMode(ROS)+ 回讀 measured 值(ROS)
```

REST 那半知道「應該有哪些點 / 宣稱的狀態」,ROS 那半知道「現在到不到得了 / 實測值」。
合起來才是一句「地圖改完了幫我驗一下」或「這任務為什麼卡住」。

精確的分法:

- agent 的 tool ➜ **全是 HTTP**(對它而言 ROS 是透明的)
- MCP server 的 tool ➜ **可純 REST、可純 ROS、可兩者混合**

所以「tool 可以指純 HTTP,也可以混合 HTTP 和 ROS」—— 答案是**可以**,但混合的那一層在
`syncai_ros_mcp`,不在 `syncai_device_agent`。要新增混合工具,是去 `roboneuron` §2 的
registry / §3 的 locomotion 那邊加,不是在 agent 這邊。

---

## 3. skills:不是「被 call」的,是漸進揭露的 SOP

先糾正一個常見誤解。`deepagents==0.7.5` 的 `create_deep_agent` 確實有 `skills` 參數,但
skill **不是**像 tool 那樣被呼叫的東西:

| 原語 | 是什麼 | agent 怎麼用 |
|---|---|---|
| **tool** | 一個可執行函式 | **呼叫**它,拿回傳值 |
| **subagent** | 一個子 agent(透過內建 `task` tool 觸發) | 把子任務**委派**出去 |
| **skill** | 一個目錄 + 一份 `SKILL.md`(YAML frontmatter + markdown) | 在相關時**讀進來當操作手冊**,漸進揭露 |

一句話:**tool 是「能力」,skill 是「怎麼把這些能力串起來完成一件事的 SOP」。**

這正好解掉一個張力:`mcp-server-proposals.md` §2 那些多步驟因果鏈(`explain_task` 要串
四個來源)如果硬塞進一個巨大的 MCP tool,tool 會變成一個上帝函式;拆成「原子能力 =
tool」+「流程 = skill」才乾淨。

### 定義方式(本版真實 API)

skill 是一個**目錄**,裡面一份 `SKILL.md`:

```
skills/
└── diagnose-stuck-task/
    └── SKILL.md
```

```markdown
---
name: diagnose-stuck-task
description: 使用者問「某任務為什麼卡住/失敗」時使用。跨 REST、Temporal、ROS、日誌拼因果。
allowed-tools: [get_task, get_workflow_history, tail_logs, get_nav_result]
---

# 診斷卡住的任務

1. `get_task(task_id)` 取步驟清單與宣稱狀態。
2. `get_workflow_history` 找出哪個 activity 失敗、retry 幾次。
3. 若失敗步驟是 MOVE,`get_nav_result` 讀 navigate_to_pose 的 result code。
4. `tail_logs` 抓同一時間窗的 controller / lio_bridge 輸出。
5. ⚠️ low_level_mode 全零是歧義的(見 RobotState.msg),先確認 motor_status.timestamp
   在前進再下結論 —— 這條部落知識屬於 SOP,不屬於任何單一 tool。
```

frontmatter 支援 `name` / `description` / `allowed-tools`(從 `deepagents/middleware/
skills.py` 確認)。**`description` 是漸進揭露的關鍵**:agent 平常只看到這一行,判斷相關
才把整份 body 讀進 context,所以 skill 再多也不會撐爆主 context —— 這對「日誌考古」這種
會吐大量輸出的任務尤其重要(配合 deepagents 內建的 filesystem 工具)。

掛上去:

```python
agent = create_deep_agent(
    model="anthropic:claude-opus-4-8",
    tools=[*mcp_tools, get_map_vertices],
    skills=["skills/diagnose-stuck-task"],   # 路徑走 deepagents backend filesystem
    interrupt_on=INTERRUPT_POLICY,           # 見 §4
)
```

### skill vs subagent 怎麼選

兩者都能承接多步驟任務,判準是**要不要獨立 context**:

- 流程短、要共用主對話的上下文 → **skill**(只是把 SOP 注入 system prompt 空間)。
- 流程長、會產生大量中間輸出、想隔離 context → **subagent**(透過 `task` tool 委派,
  跑完只回結論)。`mcp-server-proposals.md` §8.1 提到「任務為什麼卡住」適合 todo-planning
  + subagent,就是這個意思。

---

## 4. 工具分級 → `interrupt_on`(照抄既有結論)

deepagents 的 `interrupt_on` 直接落地 `mcp-server-proposals.md` §5.3 的三級制:

```python
INTERRUPT_POLICY = {
    # read-only:不攔
    "get_topics": False, "tail_logs": False, "check_stack": False, "verify_vertices": False,
    # mutating:要人核准
    "create_task":    {"allowed_decisions": ["approve", "reject"]},
    "set_motion_key": {"allowed_decisions": ["approve", "edit", "reject"]},
    # destructive:核准制,或乾脆不暴露
    "switch_mode":    {"allowed_decisions": ["approve", "reject"]},
}
```

⚠️ **`interrupt_on` 是 UX,不是安全邊界**(§8.3 已警告):攔截在 agent client 側,任何
直接打 `:8000` 的東西都繞得過。ESTOP `'4'` 這種明確拒絕的正確位置仍在 **server 側**;
agent 的核准機制是疊在上面的第二層,不是替代。HITL 的 approve/reject 需要 UI 端點,
operator console(Next.js frontend)是自然落點 —— 這是接 mutating 工具前要先做的事。

---

## 5. 多機器人的坑

`MultiServerMCPClient` 同時掛 robot01 / robot02 時,兩邊的 `get_topics`、`tail_logs`
**同名衝突**。最省事的做法是**一台機器人一個 agent instance**,跟 Temporal task queue
按 `robot_id` 分流的哲學一致。要單一 agent 管全 fleet,就回到 `mcp-server-proposals.md`
§5.4:fleet-side server 每個工具顯式收 `robot_id`,而不是把多個 per-robot server 疊在
同一個 client 裡。

---

## 6. `syncai_device_agent` 的落地計畫

現狀:`main.py` 是 deepagents 官方的 web_search 研究員範例,`tools.py` 空檔,`pyproject`
缺 `langchain-mcp-adapters`。建議照下面順序長出來(對齊兩份 proposal 的落地順序):

1. **補依賴**:`langchain-mcp-adapters`(走 MCP tools)。REST tool 用既有的 `requests`
   即可,但 async 場景建議改 `httpx`。
2. **tools.py**:先只放 (b) 純 REST `@tool`(get_map_vertices / get_task / robot_state)
   —— 零風險、不依賴 MCP server 起來。
3. **接 MCP tools**:等 `syncai_ros_mcp` 補上 `roboneuron` 提案 1/2 的唯讀工具
   (check_stack / tail_logs),用 `MultiServerMCPClient` 吃進來。
4. **第一個 skill**:`diagnose-stuck-task`(§3),把 `mcp-server-proposals.md` §2 範例 B
   的四來源因果鏈編碼進去。全程 `interrupt_on: False`,零風險驗證價值。
5. **加 checkpointer + interrupt**:要暴露 mutating 工具(create_task / switch_mode)時,
   HITL 骨架就位。checkpointer 上生產換 Postgres(這 stack 已有 `:5432`),不要留
   `MemorySaver`。

架構上乾淨的一點:整個 `syncai_device_agent` 只跟 `:8000` / `:3000` 講 HTTP,**不進
robot container、不碰 DDS**,所以它可以跑在開發機或 fleet 側任何到得了機器人的地方。

---

## 7. 與另外兩份 proposal 的關係

三份文件是一條線,不重疊:

| 文件 | 回答 | 層 |
|---|---|---|
| `mcp-server-proposals.md` | 出了什麼問題 / 這件事對不對(診斷面) | MCP server |
| `roboneuron-application-proposal.md` | agent 怎麼把機器人當一組 typed 能力(控制面) | MCP server |
| **本文件** | **誰來呼叫這些工具、tool vs skill 怎麼分** | **deepagent** |

那兩份的 §8 都已經起過 deepagents 的頭(接線、`interrupt_on`、多機器人),本文件把它從
「附錄」升成獨立設計,並補上它們沒展開的部分:**三種 tool 來源的判準、混合 HTTP+ROS
發生在哪一層、以及 skills 不是 tool 這件事**。實作判準不變:工具需要碰 ROS 或跨來源
關聯 → 屬 MCP server;工具只查歷史 → agent 側直接 REST;多步驟 SOP → skill;要隔離
context 的長任務 → subagent。
