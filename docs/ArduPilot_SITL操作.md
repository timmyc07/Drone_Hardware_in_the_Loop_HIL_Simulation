# 本機 ArduCopter SITL 操作


飛控通訊與被動健康回報的判讀方式見[飛控健康診斷](飛控健康診斷.md)；命令送出後仍需確認飛控實際狀態。
本功能使用本機 ArduCopter 軟體飛控與四旋翼物理模型閉環，非實體飛控或硬體在迴路。支援官方 [Copter-4.7.2](https://github.com/ArduPilot/ardupilot/releases/tag/Copter-4.7.2)，基底提交 `fb773567896014da007bca58358704d4a0d8516f`，僅限四旋翼地面起始；不支援外部固定翼或 VTOL。

畫面顯示物理模型真值，並非 ArduPilot EKF 估測值。飛控回報版本及八字元 Git 前綴，與預期完整來源 SHA 分別呈現；短前綴不能證明完整來源、修補或執行檔內容。二進位 SHA-256 可識別檔案，不能單獨證明來源。

## 建置與啟動

本機測試使用 macOS Apple Silicon、Python 3.9.6、Apple Clang 21、macOS 15.4 SDK 與 classic linker。以下頂層套件版本經測試，並非完整傳遞依賴鎖定；其他平台尚未驗證。請先安裝 Xcode Command Line Tools、Git 與 Python，並在專案根目錄執行：

```sh
ARDU_DIR="$HOME/Library/Caches/drone-flight-lab/ardupilot-Copter-4.7.2"
ARDU_ENV="$HOME/Library/Caches/drone-flight-lab/ardupilot-tools"
ARDU_PATCH="$PWD/companion/patches/ardupilot-loopback.patch"
git clone --branch Copter-4.7.2 --depth 1 https://github.com/ArduPilot/ardupilot.git "$ARDU_DIR"
python3 -m venv "$ARDU_ENV"
"$ARDU_ENV/bin/python" -m pip install empy==3.3.4 pymavlink==2.4.50 pyserial==3.5 pexpect==4.9.0 Jinja2==3.1.6 PyYAML==6.0.3 numpy==2.0.2 lxml==6.1.3 packaging==26.3
git -C "$ARDU_DIR" submodule update --init --depth 1 modules/waf modules/mavlink modules/littlefs modules/lwip modules/DroneCAN/DSDL modules/DroneCAN/pydronecan modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard
git -C "$ARDU_DIR/modules/mavlink" submodule update --init --depth 1 pymavlink
git -C "$ARDU_DIR" apply --check "$ARDU_PATCH"
git -C "$ARDU_DIR" apply "$ARDU_PATCH"
(cd "$ARDU_DIR" && PATH="$ARDU_ENV/bin:$PATH" SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk LDFLAGS=-fuse-ld=/Library/Developer/CommandLineTools/usr/bin/ld-classic ./waf configure --board sitl)
(cd "$ARDU_DIR" && PATH="$ARDU_ENV/bin:$PATH" SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk LDFLAGS=-fuse-ld=/Library/Developer/CommandLineTools/usr/bin/ld-classic ./waf copter -j4)
```

建置會自動取得 gtest。此機曾因不支援的 serial9 啟動參數而發生 SIGSEGV，目前啟動方式已避開該參數。[本機 loopback 修補](../companion/patches/README.md)採 GPL-3.0-or-later，修補 SHA-256 為 `892be52e36b8e6bdbc7dd6f4208a99f250b1aae116abd2e8080b9b3b068c0e42`；測試編譯品 SHA-256 為 `a56f45f38e1fb78af5ae5cf5770d01a6a1d1a630308854a2867fe6ac9b67945c`。不同環境的編譯品不一定有相同雜湊。

在 Flight Lab 專案根目錄執行 `npm ci`、`npm run build`、`npm run companion`，然後開啟 [本機 Flight Lab](http://127.0.0.1:8787)。在「飛控介接」選擇 ArduCopter，輸入建置出的 `build/sitl/bin/arducopter` 絕對路徑並啟動。等待版本與參數確認、GPS／EKF 暖機及健康檢查，再操作起飛和降落；暖機期間解鎖可能被拒絕，請依日誌等待後重試。命令已送出不代表已起飛或成功解鎖。

## 時序、記錄與驗證

[JSON 外部物理介面](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/libraries/SITL/examples/JSON/readme.md)以 UDP 傳送 servo 輸出，物理端回傳絕對模擬時間、機體 IMU、NED 位置與速度、姿態。模擬物理步長 1 ms（1000 Hz），飛控主迴圈 400 Hz；等待或暫停時模擬時間停止。健康檢查保持 `ARMING_SKIPCHK=0`；GPS 使用 serial3／4 內部 pipe 與 protocol 5，`RC_PROTOCOLS=524288` 使用合成 RC。這些是模擬設定，不適用實機。

ArduPilot 使用獨立 `flight-lab-ardupilot` 版本 1 紀錄，與內建 schema 11 及 PX4 紀錄不同。先匯出 SITL JSON；離線重播不重新執行飛控：

```sh
node companion/replay.js /absolute/path/to/ardupilot-recording.json
```

單次原生測試在 126.056 秒模擬時間內完成起飛、60.026 秒懸停及降落自動上鎖，懸停高度 RMSE 約 0.00179 m、最大水平位移約 0.30074 m；126,056 筆執行器輸出中無無效封包或序號不一致，MAVLink CRC 錯誤為 0。獨立重播核對 126,056 筆輸入（含 1,007 啟動步數）與 2,523 個樣本，位置、姿態及馬達最大誤差均為 0。詳見[精簡驗證資料](validation/ardupilot-copter-4.7.2.json)；這些結果限於測試條件，並非實機性能保證。

Context7 沒有 Copter 4.7.2 的精確版本索引，協定以該版本的官方 tag 原始碼核對。更多來源及修補細節見 [ArduPilot 修補說明](../companion/patches/README.md)。

本機 ArduCopter 航點任務及真實 SITL 測試見 [ArduPilot 航點任務](ArduPilot_航點任務.md)；PX4 外部航點任務已通過兩圈原生 SITL 與獨立重播驗證。
