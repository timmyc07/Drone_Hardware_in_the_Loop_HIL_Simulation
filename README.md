# Drone Flight Lab

瀏覽器中的四軸飛行模擬器，可在 3D 場景調整目標、測試擾動，並記錄與回放飛行資料。線上入口：[Drone Flight Lab](https://drone-flight-lab-reikous.chen-yu-ting0718-05.chatgpt.site)。

這是軟體在迴路（SIL）模擬，並非硬體在迴路：目前沒有 MCU 硬體連接或 MAVLink，也沒有實機參數校準或狀態估測器。原研究計畫中的 Unity 方案是歷史提案；瀏覽器模擬器是目前實作的另一種方案。

## 本機啟動

需要 Node.js 22.12 以上及 npm。使用 Three.js 0.187.0 與 Vite 8.3.5，不需要後端或 `.env`。

```sh
npm ci
npm run dev
```

依終端機顯示的本機網址開啟。其他指令：

```sh
npm test          # 執行測試
npm run build     # 建立正式版本
npm run preview   # 預覽已建立的版本
```

## 功能與模型

- 3D 場景提供軌道、俯視與跟隨視角，以及旋翼、地面投影和飛行軌跡。
- 六自由度剛體以 NED 世界座標、FRD 機體座標及四元數表示；物理模型以 RK4 和 200 Hz 在 Worker 中運行，與畫面更新分離。
- 可設定高度、滾轉、俯仰與偏航角速度目標，並選擇懸停、滾轉階躍或陣風情境。陣風情境每 8 秒觸發一次、持續 1 秒。
- 管理區可調整模擬參數，設定儲存在瀏覽器的 localStorage。
- 以模擬時間 20 Hz 記錄，最多 30 分鐘；可匯出 CSV 或 JSON，也可匯入 JSON 回放。

模擬器使用理想姿態及高度觀測並可加入雜訊，角速度觀測來自陀螺儀。馬達以轉速延遲及轉速平方推算推力；PWM 為合成輸出。慣量由質量及臂長推算，空氣阻力使用線性模型，地面為平面；目前沒有水平位置保持控制。雜訊設為 1 時各感測值每次取樣的標準差列於 [sim.js](web/sim.js)。這些簡化不代表已通過實機驗證。

## 參考資料

- [Quadcopter_SimCon](https://github.com/bobzwik/Quadcopter_SimCon)：四軸動力學方程與座標慣例的參考；本專案未複製其程式碼。
- [Flightmare](https://github.com/uzh-rpg/flightmare)：飛行模擬架構參考。
- [Three.js r187 原始碼](https://github.com/mrdoob/three.js/tree/r187)：3D 函式庫版本參考。Context7 當時最新索引為 r186，因此版本另以官方 r187 標籤核對。
- [Vite 文件](https://vite.dev/)：建置與開發工具；本專案使用 8.3.5。

## 專案結構

- `web/sim.js`：物理、控制與感測模型。
- `web/worker.js`：模擬排程與記錄；回放由 `web/main.js` 讀取已記錄狀態。
- `web/scene.js`、`web/main.js`、`web/style.css`：3D 場景及操作介面。
- `web/recording.js`、`web/tests/`：記錄格式、CSV 與測試。
- `docs/`：原研究及硬體規劃。

## 研究與硬體規劃

- [研究計畫（交叉審核修正版）](docs/研究計畫_交叉審核修正版.md)
- [硬體準備與採購計畫](docs/硬體準備與採購計畫.md)

研究文件最後核對日期：2026-10-26。文件中的實驗設計與時程屬於規劃，並非已完成的里程碑。
