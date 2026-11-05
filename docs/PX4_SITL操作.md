# 本機 PX4 SITL 操作

飛控通訊與被動健康回報的判讀方式見[飛控健康診斷](飛控健康診斷.md)；命令送出後仍需確認飛控實際狀態。
此功能在本機執行真正的 PX4 軟體飛控，並以模擬感測器與四旋翼物理模型閉環；不是實體飛控或硬體在迴路。支援範圍僅限 PX4 v1.17.0、來源 SHA `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`、`10016_none_iris` 四旋翼；不支援外部固定翼、VTOL、ArduPilot 或任意 PX4 版本。啟動後須確認畫面顯示版本已驗證，並留意 PX4 自身健康檢查；指令已送出不代表解鎖或起飛成功。

## 啟動

準備好符合上述版本與 SHA 的本機 PX4 可執行檔，再於專案目錄執行：

```sh
npm ci
npm run build
npm run companion
```

在瀏覽器開啟 [本機 Flight Lab](http://127.0.0.1:8787)，進入「飛控介接」，選擇 PX4 並輸入 PX4 可執行檔的**絕對路徑**，選擇「啟動或接回 PX4」。如果已有暫停中的 session，刷新頁面後可保留路徑空白並接回，模擬不會自動繼續。等待版本驗證與資料流動，再使用起飛、降落、暫停或停止等按鈕，並以狀態、高度與日誌確認結果。先匯出 SITL JSON 再結束；回到內建模擬前請依介面提示保留需要的資料。公開網站不直接連接此本機服務，請從上述本機網址操作。

服務僅綁定 `127.0.0.1`；PX4 模擬器使用本機 TCP 4560；受控任務介接使用本機 UDP 14560／14561，部分飛行命令使用固定控制台指令，不是通用 MAVLink 地面站；不提供任意 UDP 地面站、任意控制台命令或遠端端點。斷線會暫停，等待、暫停或通訊故障時模擬時間不推進；不保證硬即時。

## 在 macOS Apple Silicon 建置已驗證版本

以下頂層套件版本曾在 macOS ARM64、Python 3.9.6 與 Apple Clang 21 測試；其他平台尚未驗證。先安裝 Xcode Command Line Tools、Git 與 Python，在專案根目錄執行；工具來源與 Python 環境集中於 `local/tools/`。這不是完整的 Python 傳遞依賴鎖定檔。已收納的本機編譯品可直接使用，以下指令供全新建置使用。

```sh
mkdir -p "$PWD/local/tools"
PX4_DIR="$PWD/local/tools/px4-v1.17.0"
PX4_ENV="$PWD/local/tools/px4-python"
git clone --branch v1.17.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
python3 -m venv "$PX4_ENV"
"$PX4_ENV/bin/python" -m pip install cmake==3.31.10 ninja==1.13.2 empy==3.3.4 jinja2==3.1.6 kconfiglib==14.1.0 pyyaml==6.0.3 packaging==26.3 toml==0.10.2 jsonschema==4.25.1 numpy==2.0.2 pyros-genmsg==0.5.8 pymavlink==2.4.50
git -C "$PX4_DIR" submodule update --init --depth 1 src/modules/mavlink/mavlink src/lib/events/libevents src/lib/heatshrink/heatshrink src/drivers/gps/devices
git -C "$PX4_DIR/src/modules/mavlink/mavlink" submodule update --init --depth 1 pymavlink
PX4_PATCH="$PWD/companion/patches/px4-loopback.patch"
git -C "$PX4_DIR" apply --check "$PX4_PATCH"
git -C "$PX4_DIR" apply "$PX4_PATCH"
PATH="$PX4_ENV/bin:$PATH" make -C "$PX4_DIR" px4_sitl_default
```

PX4 基底來源為 v1.17.0、提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；loopback 修補 SHA-256 為 `a66390ec624cb83a7e6ae2a4971634418e84629618e668b957466d12e1ec6674`，本機測試編譯品 SHA-256 為 `04634c9d42ecc77842257dd85f96013206eb009063768931d5102c3c97303b4a`。飛控自報基底 SHA 和編譯品雜湊不構成對修補內容的密碼學證明；請保留建置及 socket 驗證。詳見[修補說明](../companion/patches/README.md)。建置會另外取得 Micro-XRCE-DDS Client；測試時使用的提交為 `711aef423edd1820347b866d1e4164832df35d04`。完成後可在介面輸入 `PX4_DIR` 對應的 `build/px4_sitl_default/bin/px4` 絕對路徑；使用前由 companion 檢查 PX4 版本與來源 SHA。

## 資料與驗證

PX4 收到模擬感測器訊息，回傳的執行器輸出直接驅動 RK4 物理模型，沒有退回內建 PID 控制。模型以 4 ms 模擬步長運行，畫面所示為物理模型真值，不是 PX4 的估測值或設定目標。GPS 為簡化合成定位；磁場使用 PX4 隨附的 WMM-2020 查表，氣壓採 ISA 近似，雜訊與模型均未經實機校準。

SITL 紀錄採獨立的 `flight-lab-px4` 版本 1，包含啟動步數、執行器指令、事件、採樣資料與來源資訊，不能作為現行內建飛行紀錄 schema 16 匯入。離線重播只重演已記錄的物理輸入，不執行 PX4：

```sh
node companion/replay.js /absolute/path/to/sitl-recording.json
```

若要自行執行完整 PX4 驗證並保存產物：

```sh
node companion/verify-px4.mjs /absolute/path/to/px4 /absolute/path/to/artifacts
```

已驗證的單次測試包含 60.04 秒模擬懸停、陣風及降落自動上鎖，共 91.38 秒模擬時間；高度 RMSE 約 0.0058 m、最大水平位移約 0.3585 m，接收 22,830 筆控制輸出，CRC 與時間戳不一致均為 0。獨立離線重播核對 1,828 個樣本與 15 個啟動步數，位置、四元數及馬達最大誤差均為 0。這是特定版本與條件的測試結果，並非實機精度、物理 HIL 或其他機型保證。瀏覽器實測已完成 PX4 起飛至 2.50 m、暫停後刷新並以空白路徑接回原暫停模擬、匯出紀錄、繼續飛行及降落自動上鎖；接回不會自動繼續，測試中 CRC 錯誤為 0。亦已驗證返回內建模式。

實作使用 `node-mavlink` 2.3.0（LGPL-3）與 `ws` 8.22.1（MIT）；套件版本及 API 已對照安裝原始碼，Context7 沒有這兩個精確版本的文件索引。參考 [PX4 v1.17 模擬原始碼](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0/src/modules/simulation)、[Iris 模型設定](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris)與 [MAVLink 訊息定義](https://mavlink.io/en/messages/common.html)。

0.10 版另支援 ArduCopter，兩種飛控擇一運行；其原生飛行閉環與離線重播已驗證，詳見 [ArduPilot SITL 操作](ArduPilot_SITL操作.md)。

0.12 版 PX4 航點任務已通過兩圈原生 SITL 與獨立重播驗證，參見 [PX4 航點任務](PX4_航點任務.md)。

## 0.32 電池推進模型

現行獨立格式 1 紀錄必須包含 `modelVersion: "0.13.0"`；缺少此標記或使用舊模型的紀錄會被拒絕。物理端的電池供電現會改變實際旋翼推力，離線重播也核對電池與推進資料；ArduPilot 原有 JSON 電壓／電流通道沿用，PX4 未新增電池輸入通道。詳見[電池與推進供電](電池與推進供電.md)。以上先前版本的驗證數字屬歷史結果，不能代表新模型的起降驗證。
