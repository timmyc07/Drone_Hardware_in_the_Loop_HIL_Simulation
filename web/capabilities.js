// 公開資料盤點；狀態描述本專案，不代表外部產品狀態。
export const SOURCES = [
  {
    name: "PX4 模擬",
    url: "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    kind: "開源文件",
  },
  {
    name: "PX4 HITL",
    url: "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/hitl.md",
    kind: "開源文件",
  },
  {
    name: "ArduPilot SITL",
    url: "https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html",
    kind: "官方文件",
  },
  {
    name: "ArduPilot 周邊",
    url: "https://ardupilot.org/dev/docs/adding_simulated_devices.html",
    kind: "官方文件",
  },
  {
    name: "AirSim",
    url: "https://github.com/microsoft/AirSim",
    kind: "開源專案",
  },
  {
    name: "AirSim 感測器",
    url: "https://github.com/microsoft/AirSim/blob/main/docs/sensors.md",
    kind: "開源文件",
  },
  {
    name: "Flightmare",
    url: "https://github.com/uzh-rpg/flightmare",
    kind: "開源專案",
  },
  {
    name: "Flightmare 介接",
    url: "https://github.com/uzh-rpg/flightmare/wiki/Introduction",
    kind: "開源文件",
  },
  {
    name: "Webots",
    url: "https://www.cyberbotics.com/doc/reference/nodes-and-keywords?version=R2023a",
    kind: "官方文件",
  },
  {
    name: "Isaac Sim 5.1",
    url: "https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/overview.html",
    kind: "官方文件",
  },
  {
    name: "Isaac Sim RTX 感測器",
    url: "https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html",
    kind: "官方文件",
  },
  {
    name: "Liftoff Micro Drones",
    url: "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
    kind: "官方公告",
  },
  {
    name: "VelociDrone",
    url: "https://www.velocidrone.com/features",
    kind: "官方產品頁",
  },
  {
    name: "VelociDrone 排行榜",
    url: "https://www.velocidrone.com/leaderboards",
    kind: "官方產品頁",
  },
  {
    name: "Zephyr 功能",
    url: "https://zephyr-sim.com/features?question=1",
    kind: "官方產品頁",
  },
  {
    name: "Zephyr 指導",
    url: "https://wiki.zephyr-sim.com/getting-started/scenarios/tutorial",
    kind: "官方文件",
  },
  {
    name: "Zephyr 教學管理",
    url: "https://wiki.zephyr-sim.com/help/faqs",
    kind: "官方文件",
  },
  {
    name: "Zephyr NIST",
    url: "https://zephyr-sim.com/nist-tutorial",
    kind: "官方文件",
  },
  {
    name: "SRIZFLY SESP-U1",
    url: "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    kind: "官方規格書",
  },
  {
    name: "SIMNET 手動固定翼課程",
    url: "https://www.simnet.aero/product-page/online-course-manual-piloting-of-fixed-wing-uavs",
    kind: "官方課程",
  },
  {
    name: "SIMNET SITL",
    url: "https://www.simnet.aero/post/autonomous-sitl-flight-simulation-tutorial",
    kind: "官方教學",
  },
  {
    name: "Airbus SE-RPAS",
    url: "https://www.airbus.com/en/products-services/defence/uas/uas-solutions/uas-simulation-trainer-se-rpas",
    kind: "官方產品頁",
  },
  {
    name: "Airbus SE-RPAS 規格",
    url: "https://www.airbus.com/sites/g/files/jlcbta136/files/2023-02/SE-RPAS%20Training%20System.pdf",
    kind: "官方規格書",
  },
  {
    name: "GA-ASI / CAE SkyGuardian Mission Trainer",
    url: "https://www.ga-asi.com/training-and-support-services/skyguardian-mission-trainer",
    kind: "官方產品頁",
  },
  {
    name: "美國陸軍 UMS",
    url: "https://www.army.mil/article/187989/army_simulator_provides_readiness_to_drone_flight_crews",
    kind: "官方政府資料",
  },
  {
    name: "EASA 實際飛行技能",
    url: "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
    kind: "官方規範",
  },
  {
    name: "PX4 Gazebo",
    url: "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sim_gazebo_gz/index.md",
    kind: "開源文件",
  },
  {
    name: "AirSim 影像 API",
    url: "https://github.com/microsoft/AirSim/blob/main/docs/image_apis.md",
    kind: "開源文件",
  },
  {
    name: "AirSim API",
    url: "https://github.com/microsoft/AirSim/blob/main/docs/apis.md",
    kind: "開源文件",
  },
  {
    name: "SIMNET 自主任務課程",
    url: "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    kind: "官方課程",
  },
  {
    name: "Liftoff 客製及模式",
    url: "https://www.liftoff-game.com/news/early-access-release-date-announcement",
    kind: "官方公告",
  },
  {
    name: "VelociDrone 手冊",
    url: "https://www.velocidrone.com/desktop_manual",
    kind: "官方手冊",
  },
  {
    name: "Webots 感測器",
    url: "https://www.cyberbotics.com/doc/guide/sensors?version=cyberbotics%3AR2019a",
    kind: "官方文件",
  },
  {
    name: "Uncrashed：FPV Drone Simulator",
    url: "https://store.steampowered.com/app/1682970/",
    kind: "發行者產品頁",
  },
  {
    name: "TRYP FPV",
    url: "https://store.steampowered.com/app/1881200/",
    kind: "發行者產品頁",
  },
  {
    name: "PX4 故障注入",
    url: "https://docs.px4.io/main/en/debug/failure_injection",
    kind: "官方文件",
  },
  {
    name: "ArduPilot 模擬故障",
    url: "https://ardupilot.org/dev/docs/SITL_simulation_parameters.html",
    kind: "官方文件",
  },
  {
    name: "PX4 氣動模型",
    url: "https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/liftdrag_plugin/README.md",
    kind: "開源文件",
  },
  {
    name: "PX4 固定翼跑道起飛",
    url: "https://docs.px4.io/main/en/flight_modes_fw/takeoff",
    kind: "官方文件",
  },
  {
    name: "AirSim 場景物件 API",
    url: "https://github.com/microsoft/AirSim/blob/main/PythonClient/airsim/client.py",
    kind: "開源文件",
  },
  {
    name: "Webots 場景編輯",
    url: "https://github.com/cyberbotics/webots/blob/master/docs/guide/the-user-interface.md",
    kind: "開源文件",
  },
  {
    name: "FPV Web",
    url: "https://github.com/rachmataditiya/fpv-web",
    kind: "開源專案",
  },
  {
    name: "JSBSim",
    url: "https://github.com/JSBSim-Team/jsbsim",
    kind: "開源專案",
  },
];

export const CAPABILITIES = [
  {
    id: "quad-6dof",
    category: "飛行模型",
    title: "四旋翼六自由度",
    description: "四旋翼平移及旋轉動力學。",
    status: "implemented",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    ],
  },
  {
    id: "fixed-wing",
    category: "飛行模型",
    title: "固定翼動力學",
    description:
      "固定翼採簡化升阻力、力矩與失速近似，從空中開始；未經實機校準。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
      "https://www.simnet.aero/product-page/online-course-manual-piloting-of-fixed-wing-uavs",
      "https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/liftdrag_plugin/README.md",
      "https://github.com/JSBSim-Team/jsbsim",
    ],
  },
  {
    id: "vtol",
    category: "飛行模型",
    title: "垂直起降轉換",
    description:
      "lift+cruise 機型提供懸停與巡航轉換；最低轉換高度 8 m，未實機校準。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
      "https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/liftdrag_plugin/README.md",
      "https://github.com/JSBSim-Team/jsbsim",
    ],
  },
  {
    id: "multi-rotor",
    category: "飛行模型",
    title: "多種旋翼配置",
    description:
      "四、六、八旋翼 X 型配置；新機型已測試飛行及資料往返，未提供故障容錯或外部六／八旋翼飛控介接。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    ],
  },
  {
    id: "vehicle-setup",
    category: "飛行模型",
    title: "機體與組件設定",
    description: "提供三類機型預設及機翼、推進與舵面參數。",
    status: "partial",
    sources: [
      "https://zephyr-sim.com/features?question=1",
      "https://www.liftoff-game.com/news/early-access-release-date-announcement",
      "https://store.steampowered.com/app/1881200/",
    ],
  },
  {
    id: "battery",
    category: "飛行模型",
    title: "電池與續航",
    description: "以簡化電量與電壓下降估算續航；未以電壓降額馬達推力。",
    status: "partial",
    sources: [
      "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
    ],
  },
  {
    id: "pid",
    category: "控制與航行",
    title: "PID 飛行控制",
    description: "目前使用理想狀態觀測的 PID 控制。",
    status: "implemented",
    sources: ["https://github.com/uzh-rpg/flightmare/wiki/Introduction"],
  },
  {
    id: "manual",
    category: "控制與航行",
    title: "手動操控",
    description: "支援鍵盤及瀏覽器手把輸入；實體手把尚未驗證。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
      "https://www.velocidrone.com/desktop_manual",
      "https://store.steampowered.com/app/1682970/",
    ],
  },
  {
    id: "hover",
    category: "控制與航行",
    title: "定點懸停",
    description: "提供位置保持與定點懸停練習；控制使用模擬狀態，非 GPS／EKF。",
    status: "partial",
    sources: [
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
    ],
  },
  {
    id: "takeoff",
    category: "控制與航行",
    title: "起飛",
    description:
      "四旋翼及 VTOL 提供起飛指令；固定翼提供簡化跑道起飛與中止；未實機校準。",
    status: "partial",
    sources: [
      "https://docs.px4.io/main/en/flight_modes_fw/takeoff",
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
      "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    ],
  },
  {
    id: "landing",
    category: "控制與航行",
    title: "降落",
    description:
      "四旋翼與 VTOL 支援降落；固定翼提供家點進場、拉平及滑跑近似；無風與 3 m/s 風測試完成，非精準跑道落點。",
    status: "partial",
    sources: [
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
      "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    ],
  },
  {
    id: "waypoints",
    category: "控制與航行",
    title: "航點任務",
    description:
      "支援內建航點及指定 ArduCopter／PX4 四旋翼的任務上傳、讀回核對與自動起降。",
    status: "partial",
    sources: [
      "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    ],
  },
  {
    id: "rtl",
    category: "控制與航行",
    title: "返航",
    description: "四旋翼與 VTOL 使用簡化返航；固定翼返回家點盤旋。",
    status: "partial",
    sources: [
      "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    ],
  },
  {
    id: "geofence",
    category: "控制與航行",
    title: "飛行範圍",
    description: "提供半徑及高度圍欄與簡化返航處置。",
    status: "partial",
    sources: [
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
    ],
  },
  {
    id: "terrain-follow",
    category: "控制與航行",
    title: "地形跟隨",
    description:
      "內建多旋翼及懸停 VTOL 位置模式依局部地形真值跟隨；已驗證八旋翼飛行及重播，非感測器估測或避障。",
    status: "partial",
    sources: [
      "https://www.simnet.aero/product-page/introduction-to-arducopter-auto-missions-with-mission-planner",
    ],
  },
  {
    id: "los",
    category: "控制與航行",
    title: "目視操控",
    description: "可使用環繞、俯視與跟隨視角；無固定地面飛行員觀察視角。",
    status: "partial",
    sources: [
      "https://www.simnet.aero/product-page/online-course-manual-piloting-of-fixed-wing-uavs",
    ],
  },
  {
    id: "camera-fpv",
    category: "控制與航行",
    title: "第一人稱視角",
    description: "提供機身第一人稱畫面；並非校準相機資料。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim"],
  },
  {
    id: "third-person",
    category: "控制與航行",
    title: "第三人稱視角",
    description: "環繞、俯視及跟隨觀察。",
    status: "implemented",
    sources: [
      "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
    ],
  },
  {
    id: "wind",
    category: "環境",
    title: "風與陣風",
    description: "恆定風與陣風擾動。",
    status: "implemented",
    sources: [
      "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    ],
  },
  {
    id: "time-of-day",
    category: "環境",
    title: "日夜變化",
    description: "提供時間與光照視覺變化，不含經校準的照度模型。",
    status: "partial",
    sources: [
      "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    ],
  },
  {
    id: "fog",
    category: "環境",
    title: "霧與能見度",
    description: "提供場景視覺霧效，不影響感測器物理。",
    status: "partial",
    sources: [
      "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    ],
  },
  {
    id: "rain",
    category: "環境",
    title: "雨景",
    description: "提供場景視覺降雨，不影響感測器或飛行物理。",
    status: "partial",
    sources: [
      "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    ],
  },
  {
    id: "custom-map",
    category: "環境",
    title: "自訂地圖",
    description: "可編輯並保存本機障礙盒與環境，非完整 GIS 或任意網格。",
    status: "partial",
    sources: [
      "https://github.com/cyberbotics/webots/blob/master/docs/guide/the-user-interface.md",
      "https://srizfly.com/wp-content/uploads/2024/10/SRIZFLY-SESP-U1-Drone-Simulator-TDS-24Q3.pdf",
    ],
  },
  {
    id: "physical-obstacles",
    category: "環境",
    title: "障礙物",
    description:
      "固定場景障礙以 0.25 m 飛機包絡做掃掠 AABB 碰撞，非網格精確碰撞。",
    status: "partial",
    sources: ["https://www.velocidrone.com/features"],
  },
  {
    id: "global-terrain",
    category: "環境",
    title: "地理地形",
    description: "提供局部高度網格；不含地理資料、投影或廣域 GIS。",
    status: "partial",
    sources: [
      "https://www.ga-asi.com/training-and-support-services/skyguardian-mission-trainer",
    ],
  },
  {
    id: "track-builder",
    category: "環境",
    title: "賽道編輯",
    description: "可編輯、排序並保存本機自訂航門與場景；非社群或多人服務。",
    status: "partial",
    sources: [
      "https://github.com/rachmataditiya/fpv-web",
      "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
      "https://www.velocidrone.com/features",
    ],
  },
  {
    id: "shared-tracks",
    category: "環境",
    title: "社群賽道",
    description: "分享與下載其他人建立的賽道。",
    status: "external",
    sources: [
      "https://www.velocidrone.com/features",
      "https://www.velocidrone.com/desktop_manual",
    ],
  },
  {
    id: "imu-noise",
    category: "感測與通訊",
    title: "IMU 雜訊",
    description: "模擬慣性感測雜訊。",
    status: "implemented",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/sensors.md"],
  },
  {
    id: "sensor-latency",
    category: "感測與通訊",
    title: "感測延遲",
    description: "六種虛擬感測器可設定共同延遲；飛控仍使用既有理想觀測。",
    status: "implemented",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/sensors.md"],
  },
  {
    id: "sensor-dropout",
    category: "感測與通訊",
    title: "感測資料遺失",
    description: "六種虛擬感測器可設定共同丟包及單項故障；故障不影響飛控。",
    status: "implemented",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/sensors.md"],
  },
  {
    id: "gps",
    category: "感測與通訊",
    title: "定位感測器",
    description:
      "以局部地理近似輸出定位及速度並加入雜訊；非實體 GNSS 或飛控估測。",
    status: "partial",
    sources: [
      "https://www.cyberbotics.com/doc/guide/sensors?version=cyberbotics%3AR2019a",
    ],
  },
  {
    id: "barometer",
    category: "感測與通訊",
    title: "氣壓高度計",
    description: "以簡化大氣模型輸出氣壓及高度，未經實體校準。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/sensors.md"],
  },
  {
    id: "magnetometer",
    category: "感測與通訊",
    title: "磁力計",
    description:
      "內建模式輸出簡化固定磁場；PX4 SITL 使用 WMM-2020 查表，均未實機校準。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/sensors.md"],
  },
  {
    id: "camera-data",
    category: "感測與通訊",
    title: "相機感測資料",
    description:
      "提供同步 RGB 與場景類別、相機內參及獨立資料集；非實體相機或經校準模型。",
    status: "partial",
    sources: [
      "https://github.com/microsoft/AirSim/blob/main/docs/image_apis.md",
      "https://github.com/microsoft/AirSim",
      "https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/overview.html",
    ],
  },
  {
    id: "depth",
    category: "感測與通訊",
    title: "深度影像",
    description: "提供相機光軸方向平面深度，無效值為 0；非徑向距離或實體校準。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/apis.md"],
  },
  {
    id: "optical-flow",
    category: "感測與通訊",
    title: "光流",
    description: "輸出基於運動與地面距離的近似光流；沒有實體相機模型。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim"],
  },
  {
    id: "lidar",
    category: "感測與通訊",
    title: "光達",
    description:
      "多層射線及 NED 點雲 CSV／PCD 匯出；涵蓋局部高度地形與障礙盒，非完整場景或實機校準。",
    status: "partial",
    sources: [
      "https://github.com/microsoft/AirSim/blob/main/docs/sensors.md",
      "https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/overview.html",
    ],
  },
  {
    id: "radar",
    category: "感測與通訊",
    title: "雷達",
    description: "使用專用感測器模型輸出雷達資料。",
    status: "planned",
    sources: [
      "https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html",
    ],
  },
  {
    id: "distance",
    category: "感測與通訊",
    title: "距離感測",
    description: "提供朝下測距的幾何近似；不含通用多方向距離感測器。",
    status: "partial",
    sources: [
      "https://github.com/microsoft/AirSim/blob/main/docs/sensors.md",
      "https://www.cyberbotics.com/doc/reference/nodes-and-keywords?version=R2023a",
    ],
  },
  {
    id: "gimbal",
    category: "感測與通訊",
    title: "雲台",
    description:
      "理想三軸雲台提供機體／世界方向目標、限速與實際姿態；瀏覽器影格與資料集往返已驗證，非實機校準或外部飛控雲台。",
    status: "partial",
    sources: ["https://ardupilot.org/dev/docs/adding_simulated_devices.html"],
  },
  {
    id: "camera-operator",
    category: "感測與通訊",
    title: "感測器操作員",
    description: "以獨立角色練習感測器操作。",
    status: "planned",
    sources: [
      "https://www.airbus.com/en/products-services/defence/uas/uas-solutions/uas-simulation-trainer-se-rpas",
    ],
  },
  {
    id: "motor-fault",
    category: "故障與安全",
    title: "馬達故障",
    description: "可設定馬達效率下降或失效；模型尚未實機校準。",
    status: "partial",
    sources: [
      "https://ardupilot.org/dev/docs/adding_simulated_devices.html",
      "https://docs.px4.io/main/en/debug/failure_injection",
    ],
  },
  {
    id: "link-loss",
    category: "故障與安全",
    title: "通訊中斷",
    description: "可模擬斷訊並觸發簡化安全處置。",
    status: "partial",
    sources: [
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
      "https://ardupilot.org/dev/docs/SITL_simulation_parameters.html",
    ],
  },
  {
    id: "emergency",
    category: "故障與安全",
    title: "緊急狀況訓練",
    description: "支援電池及故障觸發的簡化安全處置，非完整緊急程序訓練。",
    status: "partial",
    sources: [
      "https://www.airbus.com/en/products-services/defence/uas/uas-solutions/uas-simulation-trainer-se-rpas",
    ],
  },
  {
    id: "tutorial",
    category: "訓練與評估",
    title: "基礎教學",
    description:
      "提供七步內建模擬飛行引導；已驗證模擬飛行與重播，非實機認證或完整教學系統。",
    status: "partial",
    sources: ["https://wiki.zephyr-sim.com/getting-started/scenarios/tutorial"],
  },
  {
    id: "precision",
    category: "訓練與評估",
    title: "精準操控",
    description: "提供有限的定點懸停與降落練習。",
    status: "partial",
    sources: [
      "https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems?page=5",
    ],
  },
  {
    id: "inspection",
    category: "訓練與評估",
    title: "設施巡檢",
    description: "練習橋梁或高塔等巡檢場景。",
    status: "planned",
    sources: ["https://zephyr-sim.com/features?question=1"],
  },
  {
    id: "nist",
    category: "訓練與評估",
    title: "標準化練習",
    description: "依 NIST 訓練課程執行觀測練習。",
    status: "planned",
    sources: ["https://zephyr-sim.com/nist-tutorial"],
  },
  {
    id: "racing-gates",
    category: "訓練與評估",
    title: "競速門",
    description:
      "近距離四門與巡航四門依序判定穿越；固定翼在這兩類航門中僅支援巡航航線。",
    status: "partial",
    sources: ["https://www.velocidrone.com/features"],
  },
  {
    id: "race-time",
    category: "訓練與評估",
    title: "計時成績",
    description: "記錄練習時間及分數；不含完整競賽與排行榜。",
    status: "partial",
    sources: ["https://www.velocidrone.com/leaderboards"],
  },
  {
    id: "leaderboard",
    category: "訓練與評估",
    title: "排行榜",
    description: "追蹤已驗證賽道成績。",
    status: "external",
    sources: ["https://www.velocidrone.com/leaderboards"],
  },
  {
    id: "multiplayer",
    category: "訓練與評估",
    title: "多人飛行",
    description: "多人同場練習或競速。",
    status: "external",
    sources: [
      "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
      "https://www.velocidrone.com/features",
    ],
  },
  {
    id: "instructor",
    category: "訓練與評估",
    title: "教官操作站",
    description: "教官控制訓練及監督學員。",
    status: "planned",
    sources: [
      "https://www.airbus.com/sites/g/files/jlcbta136/files/2023-02/SE-RPAS%20Training%20System.pdf",
    ],
  },
  {
    id: "class-records",
    category: "訓練與評估",
    title: "班級訓練紀錄",
    description: "檢視個人與班級飛行訓練資料。",
    status: "external",
    sources: ["https://wiki.zephyr-sim.com/help/faqs"],
  },
  {
    id: "course-modules",
    category: "訓練與評估",
    title: "課程模組",
    description: "提供七種預設課程與自訂航門練習；非學習管理系統。",
    status: "partial",
    sources: ["https://wiki.zephyr-sim.com/getting-started/scenarios/tutorial"],
  },
  {
    id: "procedure-review",
    category: "訓練與評估",
    title: "程序檢核",
    description: "依內建模擬狀態檢核基礎飛行步驟；非實機程序認證。",
    status: "partial",
    sources: [
      "https://www.airbus.com/sites/g/files/jlcbta136/files/2023-02/SE-RPAS%20Training%20System.pdf",
    ],
  },
  {
    id: "record-csv",
    category: "資料與整合",
    title: "CSV 飛行紀錄",
    description: "可匯出 164 欄飛行及感測資料 CSV。",
    status: "partial",
    sources: ["https://wiki.zephyr-sim.com/help/faqs"],
  },
  {
    id: "replay-json",
    category: "資料與整合",
    title: "JSON 重播",
    description: "版本 10 JSON 保存完整場景、地形與事件；拒絕版本 1 至 9。",
    status: "partial",
    sources: [
      "https://www.liftoff-game.com/news/full-release-liftoff-micro-drones",
    ],
  },
  {
    id: "admin",
    category: "資料與整合",
    title: "管理員設定",
    description: "以管理介面調整模擬設定。",
    status: "implemented",
    sources: ["https://www.velocidrone.com/desktop_manual"],
  },
  {
    id: "sitl-px4",
    category: "資料與整合",
    title: "PX4 SITL",
    description:
      "本機連接指定 PX4 v1.17.0 四旋翼閉環；固定翼、VTOL 與其他版本尚未支援。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
      "https://www.simnet.aero/post/autonomous-sitl-flight-simulation-tutorial",
    ],
  },
  {
    id: "sitl-ardupilot",
    category: "資料與整合",
    title: "ArduPilot SITL",
    description:
      "本機連接指定 ArduCopter 4.7.2 四旋翼閉環，已驗證懸停、AUTO 航點與起降；其他機型與實體 HIL 尚未支援。",
    status: "partial",
    sources: [
      "https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html",
      "https://www.simnet.aero/post/autonomous-sitl-flight-simulation-tutorial",
    ],
  },
  {
    id: "hardware-hitl",
    category: "資料與整合",
    title: "實體硬體迴路",
    description: "將飛控硬體接入模擬迴路。",
    status: "external",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/hitl.md",
    ],
  },
  {
    id: "mavlink",
    category: "資料與整合",
    title: "MAVLink 模擬介面",
    description:
      "支援指定 PX4 感測器及執行器通訊，以及 ArduCopter 任務讀寫；不提供通用地面站連接或實體 HIL。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
      "https://ardupilot.org/dev/docs/mavlink-mission-upload-download.html",
    ],
  },
  {
    id: "ros",
    category: "資料與整合",
    title: "ROS 介接",
    description: "與機器人研究工具交換資料。",
    status: "external",
    sources: ["https://github.com/uzh-rpg/flightmare/wiki/Introduction"],
  },
  {
    id: "api",
    category: "資料與整合",
    title: "程式控制 API",
    description:
      "同頁 API 1.8 提供場景、雲台與位置高度基準設定；外部 SITL 使用獨立本機服務。",
    status: "partial",
    sources: ["https://github.com/microsoft/AirSim/blob/main/docs/apis.md"],
  },
  {
    id: "multi-vehicle",
    category: "資料與整合",
    title: "多載具模擬",
    description: "同時模擬多個載具。",
    status: "planned",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    ],
  },
  {
    id: "distributed",
    category: "資料與整合",
    title: "網路化訓練",
    description: "支援互通的網路化訓練環境。",
    status: "external",
    sources: [
      "https://www.ga-asi.com/training-and-support-services/skyguardian-mission-trainer",
    ],
  },
  {
    id: "platform-independent",
    category: "資料與整合",
    title: "跨平台訓練配置",
    description: "以共用訓練架構支援不同載具。",
    status: "external",
    sources: [
      "https://www.airbus.com/en/products-services/defence/uas/uas-solutions/uas-simulation-trainer-se-rpas",
    ],
  },
  {
    id: "flight-stack-ground",
    category: "資料與整合",
    title: "地面站連接",
    description: "以外部地面站觀察及操控模擬載具。",
    status: "external",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    ],
  },
  {
    id: "headless",
    category: "資料與整合",
    title: "無畫面執行",
    description: "可關閉瀏覽器畫面渲染；不等同伺服器無頭執行。",
    status: "partial",
    sources: [
      "https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md",
    ],
  },
];
