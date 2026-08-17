# 點雲 → gridmap 參數調校 agent 提案

> 對象:離線地圖產線(`config/system.ini` 的 `[map]` pcd / gridmap paths)
>       + `src/syncai_device_agent/`(deepagents runtime)
> 相關:`doc/task-recovery-agent-proposal.md`(同構迴圈,但那個會讓機器人動)
>       `doc/deep-agent-proposal.md`(agent 側接線 / tool vs skill 的判準)
> 狀態:**提案,尚未實作**。§6 有兩件事需要先確認。

這份筆記回答:3D 點雲壓成 2D gridmap 這件事,適不適合做成 agent。

先說結論:**投影本身不適合,選參數適合。** 而且因為這個題目離線、可量測、零實體風險,
它是 `task-recovery-agent-proposal.md` 那個迴圈的安全練習場 —— **建議把它當作這個
workspace 的第一個真正 agent 迴圈**,先於任何會讓機器人動起來的東西。

---

## 0. 心智模型:一次確定性操作不需要 agent

```
❌ agent → convert_pcd_to_gridmap(input.pcd) → 完成
```

這是一支腳本,不是 agent。LLM 在中間唯一做的事是「決定呼叫哪個函式」,而那根本沒有選擇。
**把單一確定性操作包成 tool 再套一個 agent,只是多了一層貴且慢的殼。** 這跟
`mcp-server-proposals.md` §0 說的「把 REST endpoint 一對一包成 MCP tool 最沒價值」是同一個
道理。

有價值的是**參數**。z 切片高度、resolution、佔據閾值、離群點濾除半徑…… 這些沒有一組
通用值,每個場地都不一樣:

| 參數偏差 | 後果 |
|---|---|
| z 下界太低 | 地面雜訊被當成障礙物,整張圖髒掉 |
| z 上界太高 | 桌子、棧板、矮平台在圖上消失,機器人會撞上去 |
| 濾除半徑太大 | 細柱子、桌腳被吃掉 |
| 佔據閾值太鬆 | 牆體有洞,AMCL 比對與 costmap 都會漏 |

「跑一次 → 看結果 → 判斷哪裡不對 → 調參數 → 再跑」——**這才是 agent loop 的形狀**,而且
判斷那一步確實需要判斷力,不是查表。

---

## 1. 現狀:判斷力目前在人身上

前端有一整組手動修圖工具:`components/maps/map-grid-editor.tsx`、`grid-canvas.tsx`、
`grid-toolbar.tsx`,搭配 `app/maps/[name]/edit/page.tsx`。也就是說目前的流程是:

```
FAST-LIO2 / PGO / HBA → .pcd → (投影) → gridmap → 人用眼睛看 → 手動塗改或改參數重跑
```

這個迴圈每建一次圖就跑一遍,每個場地都要重來,而且判斷依據大多沒有被記錄下來 —— 換一個
人做,就要重新累積一次經驗。這正是值得交給 agent 的形狀:**重複、需要判斷、但判斷有客觀
依據**。

---

## 2. 為什麼這是最好的第一個 agent 迴圈

**(a) 零實體風險。** 純檔案處理,機器人不會動。錯了重跑,成本只有 CPU 時間。可以放心把
permissions 開鬆一點觀察它會做什麼 —— 這在真機上不可能。

**(b) 有客觀評估指標。** 這是迴圈能不能成立的關鍵,見 §3。agent 必須能自我評估「這次比
上次好還是壞」,否則它只是在亂試。

**(c) 天然涵蓋 deepagents 的全部核心概念。** tools / backend / permissions 在這個題目裡
都是必要的而非硬湊,見 §4。

**(d) 它是 `task-recovery-agent-proposal.md` 的安全同構版。** 兩者迴圈形狀完全一樣:

```
執行 → 量測 → 歸因 → 調參 → 重試 → 收斂或放棄
```

差別只在「執行」是跑一次離線投影,還是一台四足機器人走出去。**骨架、收斂條件、放棄條件、
參數白名單全部可以在這裡練熟再搬過去。**

---

## 3. 評估指標決定 loop 成不成立

沒有量化指標,agent 就只能「看圖說故事」,迴圈不會收斂。建議做一個工具:

```
evaluate_gridmap(path) -> {
  unknown_ratio,        # -1 格佔比
  free_connectivity,    # 自由空間連通元件數 / 最大元件佔比:該通的走道有沒有斷
  wall_breaks,          # 牆體斷裂數
  wall_thickness,       # 牆厚分布(過厚 = inflation 或 z 範圍太寬)
  speckle_count,        # 孤立佔據格數量(雜訊指標)
  diff_vs_reference,    # 與上一版地圖 / CAD 的差異(可選)
}
```

⚠️ **主訊號必須是數字,圖片是佐證。** 模型看得懂 pgm,把渲染圖一起餵進去對「哪裡看起來
不對」有幫助,但**不要讓視覺判斷當唯一依據** —— 它不穩定,而且沒辦法用來比較兩次嘗試的
優劣。指標負責收斂,圖片負責解釋。

指標之間會互相拉扯(降低 unknown 通常會提高 speckle),所以 skill 裡要寫清楚優先順序,
例如:連通性 > 牆體完整 > 雜訊 > 未知格佔比。這個優先順序是部落知識,屬於 SOP。

---

## 4. 配置:三個概念各自對應到什麼

| 概念 | 這裡怎麼用 |
|---|---|
| **tools** | `run_projection(params)`、`evaluate_gridmap(path)`、`diff_maps(a, b)` |
| **backend** | `CompositeBackend`:default `StateBackend`(中間資料)+ `FilesystemBackend` 指向工作目錄(真的 .pcd / .pgm)+ `/memories/` 走 `StoreBackend` |
| **permissions** | `deny` 覆蓋 `/input/**`;`deny` 觸碰 `config/`;`interrupt` 寫入 `/approved/**` |
| （不需要） | sandbox。除非投影要跑 open3d / PCL 而你不想裝在本機 |

`/memories/` 是這個題目最有價值的副產品:累積「B1 倉庫這個場地用 z=[0.15, 1.2]」這類
場地級知識。跑過三個場地之後,第四次就有前例可參考而不是從零猜 —— 這才是 `StoreBackend`
真正發揮價值的樣子,比記使用者名字實用得多。

---

## 5. 目錄佈局與護欄

```
工作目錄(FilesystemBackend root)
├── input/site_b1.pcd        ← permissions: 唯讀。重生成本是重新建一次圖
├── candidates/              ← agent 自由寫
│   ├── attempt_01.{pgm,params.json,metrics.json}
│   └── attempt_02.…
└── approved/                ← permissions: interrupt,要人核准才寫入
```

第一版任務敘述:

> 用 `/input/site_b1.pcd` 產生一張可用的 2D gridmap。每次嘗試把參數與評估指標存進
> `/candidates/`,最多 5 次。收斂了就報告哪一組最好、為什麼;沒收斂就說你試了什麼、卡在哪。

護欄(與 `task-recovery-agent-proposal.md` §4 同一套習慣,刻意保持一致):

1. **次數上限 5 次** —— 否則它會一直調下去。
2. **參數白名單 + 範圍** —— 例如 z 切片限 0～2 m,別讓它交出物理上荒謬的值。
3. **原始資料唯讀** —— `.pcd` 覆蓋不掉,那是重建一次圖才拿得回來的東西。
4. **產出隔離** —— agent 只寫 `/candidates/`,進 `/approved/` 一律要人核准。
5. **每次嘗試留痕** —— params + metrics 成對存檔,否則無法比較也無法回頭檢討。

---

## 6. 待確認

實作前要先釐清兩件事(本文件未查證):

1. **目前 `.pcd → gridmap` 是誰做的?** 可能在 `FASTLIO2_ROS2` submodule
   (`chungweeeei/SyncAI-Fast-LIO2`,含 LIO + PGO + HBA + localizer)、`scripts/`、或
   `syncai_sys_manager` 的 map manager 裡。
2. **它能不能用 CLI 帶參數跑?** 這是整個提案的前提 —— agent 無法呼叫的東西就無法迭代。
   如果目前是寫死參數或埋在 GUI 流程裡,**第一步不是寫 agent,是先把它變成一支可帶參數
   的 CLI**。那一步本身就有價值,即使最後不做 agent。

---

## 7. 與其他 proposal 的關係

| 文件 | 回答 |
|---|---|
| `mcp-server-proposals.md` | 出了什麼問題(診斷面,MCP server 層) |
| `roboneuron-application-proposal.md` | 機器人怎麼被當成 typed 能力(控制面,MCP server 層) |
| `deep-agent-proposal.md` | 誰來呼叫工具、tool vs skill 怎麼分(agent 層,接線) |
| `task-recovery-agent-proposal.md` | 線上任務自癒迴圈(agent 層,**會動的**) |
| **本文件** | **離線地圖產線的調參迴圈(agent 層,不會動的)** |

一個結構性差異值得指出:本提案**不需要 MCP server**。它的工具是本機 CLI 包裝,不碰 ROS
graph、不碰 DDS、不需要機器人開機。所以它可以完全獨立於前三份文件的落地進度推進 ——
這也是它適合當第一個練習的原因之一。
