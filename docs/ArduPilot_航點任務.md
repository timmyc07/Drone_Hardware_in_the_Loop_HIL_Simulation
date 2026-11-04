# ArduCopter 本機航點任務（0.11 版）

本功能已完成指定版本的真實 SITL AUTO 任務及獨立重播驗證。範圍僅限本機 ArduCopter 4.7.2 四旋翼及專案的 `node-mavlink` 2.3.0；不是實體飛控、硬體在迴路或實機任務規劃服務。

## 操作流程

1. 依 [ArduPilot SITL 操作](ArduPilot_SITL操作.md)啟動本機服務及 ArduCopter，等待版本與參數確認、定位和家點就緒。健康檢查保持啟用；暖機或定位不足時命令可能被拒絕，應依日誌等待並重試。
2. 在本機介面編輯 1–50 個航點，每點為 `{north, east, altitude, hold}`：北、東及高度以公尺表示，`hold` 為 0–60 秒整數；高度為 1 m 至目前圍欄高度，0 m 留給自動產生的降落項。航點使用模擬地點附近的局部座標，轉換與高度基準須由飛控家點確認。
3. 上傳後等待飛控逐項回覆，再下載核對；只有讀回內容相符，狀態才可標記已驗證。上傳 ACK 只代表傳輸結果，不代表飛行或任務完成。
4. 開始任務後，依飛控回報觀察目前序號與已到達序號。任務包含自動降落；應確認落地及上鎖，不能僅依命令已送出判定成功。

家點使用任務序號 0，起飛為 1，使用者航點為 2 至 N+1，降落為 N+2。起飛高度取第一個航點高度，降落位置取最後一個航點的水平位置。失敗、超時、中斷或定位失效不能當作成功；未解決的傳輸狀態須向使用者顯示。

北、東以模擬原點（25.033°N、121.5654°E、海拔 20 m）為基準，依該版本 `Location::offset_latlng` 的球面近似轉換；高度以飛控家點為相對基準。上傳前會確認真實家點回報與模擬原點相符，容許水平 2 m、海拔 1 m 的啟動差異。此範圍是航點規劃限制，沒有設定 ArduPilot 原生飛行圍欄。

## 介接狀態

伴隨服務狀態的 `mission` 包含 `phase`、`homeReady`、`verified`、`points`、`origin`、`limits`、`currentSeq`、`reachedSeqs` 與 `error`。`phase` 為 `empty`、`uploading`、`verifying`、`ready`、`running`、`completed`、`interrupted` 或 `error`。WebSocket 以 `mission` 訊息傳入航點，並以 `command` 的 `mission` 動作開始；資料及命令均須在本機服務驗證。

`readyToUpload` 另要求地面、未解鎖、定位與家點就緒，以及不早於最近解鎖命令回覆的心跳。上傳交易限時 15 秒；草稿與已核對路線不同時不能啟動。暫停會凍結物理時間；停止、切換模式及人工降落等中止狀態不會當作任務完成。

上傳採 MAVLink 任務協定：`MISSION_COUNT` 後，依飛控發出的 `MISSION_REQUEST`（40）或 `MISSION_REQUEST_INT`（51）回傳對應序號的 `MISSION_ITEM_INT`（73），最後檢查 `MISSION_ACK`；再下載並逐欄核對座標、高度、參數與相容的座標框架，容許高度的公分級量化。Copter 4.7.2 儲存後的 LAND `param4` 為 +1，依該版本標準化結果核對。全球座標整數欄位使用緯經度乘以 10⁷，所選 frame 的高度基準須一致。重複、錯序、超時及拒絕回應都需處理，不能虛構完成狀態。參考 [MAVLink 任務協定](https://mavlink.io/en/services/mission.html)及 [ArduPilot 任務上傳與下載](https://ardupilot.org/dev/docs/mavlink-mission-upload-download.html)。

`MAV_CMD_MISSION_START` 為 300；193 是 `MAV_CMD_DO_PAUSE_CONTINUE`，224 是 `MAV_CMD_DO_SET_MISSION_CURRENT`，不可互換。ArduCopter AUTO 地面起飛仍受健康檢查及飛行模式條件控制；本功能不強制解鎖、不關閉檢查，也不修改 `AUTO_OPTIONS`。官方行為參見 [Copter AUTO 模式](https://ardupilot.org/copter/docs/auto-mode.html)。

Context7 沒有 `node-mavlink` 2.3.0 與 Copter 4.7.2 的精確版本索引；本專案以安裝的 2.3.0 訊息定義及 [Copter-4.7.2 官方原始碼](https://github.com/ArduPilot/ardupilot/tree/Copter-4.7.2)補核。這些結果不代表實機校準或任務安全保證。

## 完成證據與測試範圍

Copter 4.7.2 的最終 LAND 不一定送出到達通知，上鎖後目前序號也會回到 1；因此完成判定要求所有使用者航點已收到到達通知、曾選中最後 LAND、曾解鎖並離地、飛控回報在地且已上鎖、物理模型在地並穩定，且沒有人工接管。紀錄保留 `completionEvidence`，不自行捏造飛控完成通知。

一次真實 SITL 測試完成四個 3 m 高、各停留 1 秒的航點與自動降落，總模擬時間 94.078 秒；獨立重播 94,078 個控制輸入（含 1,007 啟動步數）與 1,883 個樣本，位置、四元數及馬達最大誤差均為 0。測試資料見[精簡驗證結果](validation/ardupilot-auto-mission.json)。目前僅 ArduCopter 外部飛控支援此任務，PX4 外部任務與實體 HIL 尚未支援。

SITL JSON 的 `missionEvents` 保存上傳、讀回、目前序號、到達及完成證據。離線重播重建已接受控制輸入的物理軌跡，不會重新執行飛控，也不會認證匯入的任務註記；任務成功依原生飛行驗證及遙測證據判定。
