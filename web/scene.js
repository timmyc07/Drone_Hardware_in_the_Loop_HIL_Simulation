import * as THREE from "three";
import { createTerrain } from "./terrain.js";
import { inspectionPanel } from "./inspection.js";
import { createNistLaneGroup, NIST_LANE } from "./nist-geometry.js";
import { lidarPoints, MAX_LIDAR_POINTS } from "./sensors.js";
import { getRotorLayout, MULTIROTOR_AIRFRAMES } from "./rotor-geometry.js";
import { AIRFRAME_PRESETS, RUNWAY } from "./sim.js";
import { createCameraRenderer } from "./camera-renderer.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export function obstacleVisualPlacement(item) {
  return {
    size: [item.depth, item.height, item.width],
    center: [item.east, item.baseAltitude + item.height / 2, -item.north],
  };
}

export function createScene(container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#101c2d");
  scene.fog = new THREE.Fog("#101c2d", 600, 1200);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 2000);
  const renderer = new THREE.WebGLRenderer({
    antialias: true,
    logarithmicDepthBuffer: true,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.setAttribute("aria-label", "Drone flight simulation");
  container.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxDistance = 700;
  controls.minDistance = 1.5;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;

  scene.add(new THREE.HemisphereLight(0xc2e9ff, 0x192236, 2));
  const key = new THREE.DirectionalLight(0xffd7b2, 2.8);
  key.position.set(4, 9, 5);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x64dce2, 1.3);
  fill.position.set(-6, 4, -5);
  scene.add(fill);
  const grid = new THREE.GridHelper(1200, 240, 0x35566b, 0x233c50);
  grid.material.transparent = true;
  grid.material.opacity = 0.48;
  scene.add(grid);
  const ground = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshStandardMaterial({
      color: 0x1c2d36,
      roughness: 1,
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 1,
    }),
  );

  ground.userData.cameraClass = 1;
  scene.add(ground);

  const nist = new THREE.Group();
  scene.add(nist);
  let nistEnabled = false;
  function setNistLane(enabled) {
    if (enabled === nistEnabled) return;
    clear(nist);
    nistEnabled = enabled;
    runway.visible = !enabled && terrain.runwayCompatible;
    pad.visible = !enabled && terrain.data.kind !== 'dem';
    if (!enabled) return;
    nist.add(createNistLaneGroup({ labelTexture(target) {
      const canvas = document.createElement('canvas');
      canvas.width = canvas.height = 256;
      const context = canvas.getContext('2d');
      context.fillStyle = '#fff'; context.fillRect(0, 0, 256, 256);
      context.fillStyle = '#111'; context.font = 'bold 76px sans-serif';
      context.textAlign = 'center'; context.textBaseline = 'middle';
      context.fillText(target.id, 128, 128);
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      return texture;
    } }));
    const paint = (geometry, color, north = 0, east = 0, map = null) => {
      const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color, map, side: THREE.DoubleSide }));
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.set(east, 0.001, -north);
      mesh.userData.cameraClass = 1;
      nist.add(mesh);
    };
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 512;
    const context = canvas.getContext('2d');
    context.fillStyle = '#f2f2e7'; context.fillRect(0, 0, 512, 512);
    context.strokeStyle = '#00ad55'; context.lineWidth = 0.02 / NIST_LANE.launchPadSize * 512;
    context.beginPath(); context.arc(256, 256, (NIST_LANE.launchCircleRadius - 0.01) / NIST_LANE.launchPadSize * 512, 0, Math.PI * 2); context.stroke();
    context.fillStyle = '#111'; context.font = 'bold 80px sans-serif'; context.textAlign = 'center'; context.textBaseline = 'middle'; context.fillText('L', 256, 256);
    const padTexture = new THREE.CanvasTexture(canvas); padTexture.colorSpace = THREE.SRGBColorSpace;
    paint(new THREE.PlaneGeometry(NIST_LANE.launchPadSize, NIST_LANE.launchPadSize), 0xffffff, 0, 0, padTexture);
    const start = NIST_LANE.launchPadSize / 2;
    paint(new THREE.PlaneGeometry(0.025, NIST_LANE.length - start), 0xe8db8c, (NIST_LANE.length + start) / 2);
  }

  let terrain = createTerrain();
  let runway, pad;
  function setTerrain(value) {
    const next = createTerrain(value);
    const mesh = next.mesh();
    const positions = new Float32Array(mesh.positionsNed.length);
    for (let i = 0; i < positions.length; i += 3) {
      positions[i] = mesh.positionsNed[i + 1];
      positions[i + 1] = -mesh.positionsNed[i + 2];
      positions[i + 2] = -mesh.positionsNed[i];
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(mesh.indices);
    geometry.computeVertexNormals();
    ground.geometry.dispose();
    ground.geometry = geometry;
    terrain = next;
    grid.visible = next.data.kind === "flat";
    if (runway) runway.visible = next.runwayCompatible && !nistEnabled;
    if (pad) pad.visible = next.data.kind !== "dem" && !nistEnabled;
  }
  setTerrain();

  const material = (color, metalness = 0, roughness = 0.5) =>
    new THREE.MeshStandardMaterial({ color, metalness, roughness });
  const dark = material(0x1d2934, 0.65, 0.36);
  const shell = material(0x354654, 0.55, 0.36);
  const black = material(0x101820, 0.35, 0.45);
  const teal = material(0x56d7d1, 0.45, 0.3);
  const orange = material(0xffa568, 0.4, 0.35);
  const rotorMaterial = material(0x82969d, 0.6, 0.36);
  const drone = new THREE.Group();
  drone.userData.cameraClass = 7;
  drone.userData.cameraVehicleId = "drone-1";
  scene.add(drone);
  const box = (parent, size, position, mat) => {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);
    mesh.position.set(...position);
    parent.add(mesh);
    return mesh;
  };
  const cylinder = (parent, radius, height, position, mat, segments = 24) => {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(radius, radius, height, segments),
      mat,
    );
    mesh.position.set(...position);
    parent.add(mesh);
    return mesh;
  };
  runway = new THREE.Group();
  runway.userData.cameraClass = 2;
  const runwayLength = RUNWAY.endNorth - RUNWAY.startNorth;
  runway.position.z = -(RUNWAY.startNorth + RUNWAY.endNorth) / 2;
  runway.rotation.y = -RUNWAY.headingRad;
  scene.add(runway);
  const asphalt = material(0x30383e, 0, 0.95);
  const runwayPaint = material(0xe1e7df, 0, 0.8);
  box(
    runway,
    [RUNWAY.halfWidth * 2, 0.006, runwayLength],
    [0, -0.002, 0],
    asphalt,
  );
  for (const side of [-1, 1]) {
    box(
      runway,
      [0.22, 0.01, runwayLength],
      [side * (RUNWAY.halfWidth - 0.5), 0.005, 0],
      runwayPaint,
    );
    for (let stripe = 0; stripe < 4; stripe++) {
      const x = side * (RUNWAY.halfWidth * (0.2 + stripe * 0.17));
      for (const end of [-1, 1])
        box(
          runway,
          [0.6, 0.01, 4],
          [x, 0.005, end * (runwayLength / 2 - 4)],
          runwayPaint,
        );
    }
  }
  for (
    let offset = -runwayLength / 2 + 12;
    offset < runwayLength / 2 - 12;
    offset += 12
  )
    box(runway, [0.2, 0.01, 6], [0, 0.005, offset], runwayPaint);
  // Arrow points north, toward the far end of the runway.
  box(runway, [0.3, 0.012, 5], [0, 0.006, runwayLength / 2 - 14], runwayPaint);
  for (const side of [-1, 1]) {
    const head = box(
      runway,
      [0.3, 0.012, 3],
      [side * 0.9, 0.006, runwayLength / 2 - 16],
      runwayPaint,
    );
    head.rotation.y = (side * Math.PI) / 4;
  }
  function multirotor(kind) {
    const quad = new THREE.Group();
    // Keep the visual skids on the ground while the outer group remains at the physical CG.
    quad.position.y = 0.181;
    quad.userData.airframe = kind;
    drone.add(quad);
    box(quad, [0.29, 0.085, 0.34], [0, 0, 0], dark);
    box(quad, [0.23, 0.055, 0.27], [0, 0.065, 0], shell);
    box(quad, [0.12, 0.012, 0.16], [0, 0.1, -0.015], black);
    box(quad, [0.1, 0.015, 0.025], [0, 0.11, -0.06], teal);
    const rotors = [];
    const visualArm =
      kind === "quadrotor" ? 0.325 : AIRFRAME_PRESETS[kind].arm * Math.SQRT2;
    for (const motor of getRotorLayout(kind, visualArm)) {
      const x = motor.positionFrd[1],
        z = -motor.positionFrd[0];
      const beam = box(
        quad,
        [0.075, 0.045, Math.hypot(x, z)],
        [x / 2, 0, z / 2],
        shell,
      );
      beam.rotation.y = Math.atan2(x, z);
      cylinder(quad, 0.047, 0.045, [x, 0.033, z], dark);
      cylinder(quad, 0.036, 0.015, [x, 0.063, z], x > 0 ? teal : orange);
      const rotor = new THREE.Group();
      rotor.position.set(x, 0.079, z);
      rotor.userData.motorIndex = motor.index;
      rotor.userData.motorSpin = motor.spin;
      box(rotor, [0.028, 0.008, 0.31], [0, 0, 0], rotorMaterial);
      cylinder(rotor, 0.017, 0.014, [0, 0.006, 0], black);
      quad.add(rotor);
      rotors.push({ rotor, motor });
    }
    for (const x of [-0.12, 0.12]) {
      box(quad, [0.022, 0.15, 0.022], [x, -0.095, 0.09], dark).rotation.z =
        x > 0 ? -0.15 : 0.15;
      box(quad, [0.022, 0.15, 0.022], [x, -0.095, -0.09], dark).rotation.z =
        x > 0 ? -0.15 : 0.15;
      box(quad, [0.035, 0.022, 0.29], [x * 1.1, -0.17, 0], black);
    }
    box(quad, [0.105, 0.075, 0.06], [0, -0.025, -0.2], black);
    const lens = cylinder(quad, 0.027, 0.015, [0, -0.025, -0.237], teal);
    lens.rotation.x = Math.PI / 2;

    quad.visible = kind === "quadrotor";
    return { aircraft: quad, rotors };
  }
  const multirotors = Object.fromEntries(
    MULTIROTOR_AIRFRAMES.map((kind) => [kind, multirotor(kind)]),
  );

  function winged(liftRotors) {
    const aircraft = new THREE.Group();
    // Gear bottoms sit at the physical ground plane without changing the flight state.
    aircraft.position.y = 0.271;
    aircraft.userData.airframe = liftRotors ? "vtol" : "fixedwing";
    for (const [x, z] of [
      [-0.13, 0.3],
      [0.13, 0.3],
      [0, -0.43],
    ]) {
      box(aircraft, [0.025, 0.18, 0.025], [x, -0.18, z], dark);
      box(aircraft, [0.045, 0.05, 0.11], [x, -0.245, z], black);
    }
    box(aircraft, [0.25, 0.19, 1.25], [0, 0, 0], shell);
    box(aircraft, [0.18, 0.1, 0.28], [0, 0.13, -0.25], black);
    box(aircraft, [2.2, 0.065, 0.35], [0, 0.02, -0.02], shell);
    box(aircraft, [2.05, 0.015, 0.04], [0, 0.06, -0.19], teal);
    box(aircraft, [0.8, 0.04, 0.22], [0, 0.08, 0.53], shell);
    box(aircraft, [0.045, 0.25, 0.25], [0, 0.21, 0.5], shell);
    const nose = cylinder(aircraft, 0.06, 0.11, [0, 0, -0.66], dark);
    nose.rotation.x = Math.PI / 2;
    const propeller = new THREE.Group();
    propeller.position.z = -0.73;
    propeller.userData.pusher = true;
    box(propeller, [0.025, 0.52, 0.015], [0, 0, 0], rotorMaterial);
    aircraft.add(propeller);
    const ailerons = [-0.7, 0.7].map((x) =>
      box(aircraft, [0.65, 0.025, 0.11], [x, 0.02, 0.2], dark),
    );
    const elevator = box(aircraft, [0.65, 0.025, 0.08], [0, 0.08, 0.68], dark);
    const rudder = box(aircraft, [0.03, 0.17, 0.08], [0, 0.23, 0.65], dark);
    const lift = [];
    if (liftRotors) {
      for (const x of [-0.5, 0.5])
        box(aircraft, [0.07, 0.06, 1.1], [x, 0.03, 0], dark);
      for (const motor of getRotorLayout("vtol", Math.SQRT2 * 0.5)) {
        const x = motor.positionFrd[1],
          z = -motor.positionFrd[0];
        const mount = cylinder(aircraft, 0.06, 0.06, [x, 0.09, z], dark);
        mount.userData.liftMount = motor.index;
        const rotor = new THREE.Group();
        rotor.position.set(x, 0.13, z);
        rotor.userData.motorIndex = motor.index;
        rotor.userData.motorSpin = motor.spin;
        box(rotor, [0.035, 0.012, 0.48], [0, 0, 0], rotorMaterial);
        aircraft.add(rotor);
        lift.push({ rotor, motor });
        const cap = cylinder(aircraft, 0.025, 0.02, [x, 0.14, z], orange);
        cap.userData.liftMount = motor.index;
      }
    }
    aircraft.visible = false;
    drone.add(aircraft);
    return { aircraft, propeller, ailerons, elevator, rudder, lift };
  }
  const fixedwing = winged(false);
  const vtol = winged(true);
  const airframes = {
    quadrotor: multirotors.quadrotor.aircraft,
    hexarotor: multirotors.hexarotor.aircraft,
    octorotor: multirotors.octorotor.aircraft,
    fixedwing: fixedwing.aircraft,
    vtol: vtol.aircraft,
  };
  let airframe = "quadrotor";

  pad = new THREE.Group();
  scene.add(pad);
  const ringMaterial = new THREE.MeshBasicMaterial({
    color: 0x5edbd5,
    transparent: true,
    opacity: 0.6,
    side: THREE.DoubleSide,
  });
  for (const radius of [0.8, 1.0, 2.0]) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(radius - 0.012, radius, 96),
      ringMaterial,
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.008;
    pad.add(ring);
  }
  function marker(text, x, z, color) {
    const canvas = document.createElement("canvas");
    canvas.width = 128;
    canvas.height = 128;
    const context = canvas.getContext("2d");
    context.fillStyle = color;
    context.font = "bold 80px sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(text, 64, 64);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(0.5, 0.5),
      new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(x, 0.012, z);
    pad.add(mesh);
  }
  marker("N", 0, -2.4, "#8be4df");
  marker("E", 2.4, 0, "#ffb27b");
  const projection = new THREE.Mesh(
    new THREE.RingGeometry(0.22, 0.235, 48),
    ringMaterial,
  );
  projection.rotation.x = -Math.PI / 2;
  projection.position.y = 0.02;
  scene.add(projection);
  const trailGeometry = new THREE.BufferGeometry();
  const trailPositions = new Float32Array(1200 * 3);
  trailGeometry.setAttribute(
    "position",
    new THREE.BufferAttribute(trailPositions, 3).setUsage(
      THREE.DynamicDrawUsage,
    ),
  );
  trailGeometry.setDrawRange(0, 0);
  const trail = new THREE.Line(
    trailGeometry,
    new THREE.LineBasicMaterial({
      color: 0x5edbd5,
      transparent: true,
      opacity: 0.65,
    }),
  );
  trail.frustumCulled = false;
  scene.add(trail);
  const environment = new THREE.Group();
  const obstacles = new THREE.Group();
  const mission = new THREE.Group();
  const gates = new THREE.Group();
  scene.add(environment, obstacles, mission, gates);
  obstacles.userData.cameraClass = 5;
  gates.userData.cameraClass = 6;
  const clear = (group) => {
    while (group.children.length) {
      const child = group.children[0];
      group.remove(child);
      child.traverse((item) => {
        item.geometry?.dispose();
        const materials = Array.isArray(item.material)
          ? item.material
          : [item.material];
        materials.forEach((mat) => {
          mat?.map?.dispose();
          mat?.dispose();
        });
      });
    }
  };
  let rain = null;
  function setEnvironment({
    preset = "lab",
    timeOfDay = 12,
    fog = 0,
    rain: rainAmount = 0,
  }) {
    clear(environment);
    const daylight = Math.max(0.12, Math.sin(((timeOfDay - 6) * Math.PI) / 12));
    const sky = new THREE.Color(
      preset === "field" ? 0x799cab : 0x253c50,
    ).multiplyScalar(0.28 + daylight * 0.72);
    scene.background = sky;
    scene.fog = new THREE.Fog(sky, 600 - fog * 580, 1200 - fog * 1100);
    key.intensity = 0.5 + daylight * 2.3;
    fill.intensity = 0.7 + (1 - daylight) * 0.9;
    if (preset === "field") {
      ground.material.color.setHex(0x364e43);
      for (let i = 0; i < 20; i++) {
        const tree = new THREE.Mesh(
          new THREE.ConeGeometry(0.7 + (i % 3) * 0.2, 3, 7),
          material(0x28483d, 0, 1),
        );
        const angle = (i * Math.PI * 2) / 20;
        const x = Math.cos(angle) * 18,
          z = Math.sin(angle) * 18;
        if (!terrain.contains(-z, x)) {
          tree.geometry.dispose();
          tree.material.dispose();
          continue;
        }
        tree.position.set(x, 1.5 + terrain.sample(-z, x).heightM, z);
        tree.userData.cameraClass = 3;
        environment.add(tree);
      }
    } else if (preset === "warehouse") {
      ground.material.color.setHex(0x33414b);
      for (let i = -2; i <= 2; i++) {
        const z = i * 8;
        if (!terrain.contains(-z, -10) || !terrain.contains(-z, 10)) continue;
        for (const x of [-10, 10])
          box(
            environment,
            [0.3, 7, 0.3],
            [x, 3.5 + terrain.sample(-z, x).heightM, z],
            material(0x526273, 0.5),
          );
        box(
          environment,
          [20, 0.25, 0.25],
          [0, 7 + terrain.sample(-z, 0).heightM, z],
          material(0x526273, 0.5),
        );
      }
    }
    if (preset === "lab") ground.material.color.setHex(0x1c2d36);
    if (preset === "warehouse")
      environment.traverse((object) => {
        if (object.isMesh) object.userData.cameraClass = 4;
      });
    rain = null;
    if (rainAmount > 0) {
      const positions = new Float32Array(
        Math.ceil(400 * Math.min(1, rainAmount)) * 3,
      );
      for (let i = 0; i < positions.length; i += 3) {
        positions[i] = Math.random() * 30 - 15;
        positions[i + 1] = Math.random() * 14;
        positions[i + 2] = Math.random() * 30 - 15;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3),
      );
      rain = new THREE.Points(
        geometry,
        new THREE.PointsMaterial({
          color: 0xa9d9e9,
          size: 0.055,
          transparent: true,
          opacity: 0.65,
        }),
      );
      rain.userData.cameraRgbOnly = true;
      environment.add(rain);
    }
  }
  function setObstacles(items, targets = []) {
    clear(obstacles);
    for (const item of items.slice(0, 100)) {
      const placement = obstacleVisualPlacement(item);
      box(
        obstacles,
        placement.size,
        placement.center,
        material(0x9b765b, 0.2, 0.75),
      );
    }
    const vec = ([n, e, d]) => new THREE.Vector3(e, -d, -n);
    for (const target of targets) {
      const panel = inspectionPanel(target, items);
      const c = vec(panel.centerNed).addScaledVector(
        vec(panel.normalNed),
        0.002,
      );
      const right = vec(panel.rightNed),
        down = vec(panel.downNed);
      for (const [u, v, color] of [
        [-0.25, -0.25, 0xf6d781],
        [0.25, -0.25, 0x325867],
        [-0.25, 0.25, 0x325867],
        [0.25, 0.25, 0xf6d781],
      ]) {
        const geometry = new THREE.PlaneGeometry(
          target.width / 2,
          target.height / 2,
        );
        const mesh = new THREE.Mesh(
          geometry,
          new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide }),
        );
        const normal = right.clone().cross(down.clone().negate());
        mesh.quaternion.setFromRotationMatrix(
          new THREE.Matrix4().makeBasis(right, down.clone().negate(), normal),
        );
        mesh.position
          .copy(c)
          .addScaledVector(right, u * target.width)
          .addScaledVector(down, v * target.height);
        obstacles.add(mesh);
      }
    }
  }
  function setMission(waypoints) {
    clear(mission);
    const points = waypoints
      .slice(0, 200)
      .map(
        (point) => new THREE.Vector3(point.east, point.altitude, -point.north),
      );
    if (points.length > 1)
      mission.add(
        new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(points),
          new THREE.LineBasicMaterial({ color: 0xffbf78 }),
        ),
      );
    for (const point of points) {
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.12, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0xffbf78 }),
      );
      marker.position.copy(point);
      marker.userData.waypoint = true;
      mission.add(marker);
    }
  }
  function setGates(items) {
    clear(gates);
    for (const item of items.slice(0, 100)) {
      const gate = new THREE.Mesh(
        new THREE.TorusGeometry(item.radius, 0.045, 8, 64),
        new THREE.MeshBasicMaterial({ color: 0x5edbd5 }),
      );
      gate.position.set(item.east, item.altitude, -item.north);
      gate.rotation.y = -item.yaw;
      gates.add(gate);
    }
  }
  setEnvironment({});
  const scanGeometry = new THREE.BufferGeometry();
  const scanAttribute = new THREE.BufferAttribute(
    new Float32Array(MAX_LIDAR_POINTS * 3),
    3,
  ).setUsage(THREE.DynamicDrawUsage);
  scanGeometry.setAttribute("position", scanAttribute);
  const scanColors = new THREE.BufferAttribute(
    new Float32Array(MAX_LIDAR_POINTS * 3),
    3,
  ).setUsage(THREE.DynamicDrawUsage);
  scanGeometry.setAttribute("color", scanColors);
  scanGeometry.setDrawRange(0, 0);
  const scan = new THREE.Points(
    scanGeometry,
    new THREE.PointsMaterial({ vertexColors: true, size: 0.09 }),
  );
  scan.frustumCulled = false;
  scene.add(scan);
  function setSensorScan(reading) {
    const points = reading?.valid ? lidarPoints(reading) : [];
    let pointIndex = 0;
    const color = new THREE.Color();
    if (reading?.valid)
      reading.rangesM.forEach((range, i) => {
        if (!Number.isFinite(range)) return;
        const [north, east, down] = points[pointIndex];
        scanAttribute.array.set([east, -down, -north], pointIndex * 3);
        const layer = Math.floor(i / reading.azimuthCount);
        color.setHSL(
          reading.channelCount === 1
            ? 0.08
            : 0.65 - (0.55 * layer) / (reading.channelCount - 1),
          0.9,
          0.6,
        );
        scanColors.setXYZ(pointIndex, color.r, color.g, color.b);
        pointIndex++;
      });
    scanColors.needsUpdate = true;
    scanGeometry.setDrawRange(0, points.length);
    scanAttribute.needsUpdate = true;
  }
  const peers = new Map();
  const savedTrails = new Map();
  let activeVehicleId = "drone-1";
  function scaleAircraft(group, vehicle) {
    const kind = vehicle.snapshot.airframe;
    const model = group.children.find(
      (child) => child.userData.airframe === kind,
    );
    if (!model) return;
    if (MULTIROTOR_AIRFRAMES.includes(kind)) {
      const baseArm =
        kind === "quadrotor" ? 0.325 : AIRFRAME_PRESETS[kind].arm * Math.SQRT2;
      const scale = vehicle.config.arm / baseArm;
      model.scale.setScalar(scale);
      model.position.y = 0.181 * scale;
    } else {
      model.scale.set(
        vehicle.config.wingSpan / 2.2,
        1,
        vehicle.config.wingChord / 0.25,
      );
      if (kind === "vtol") {
        const layout = getRotorLayout("vtol", vehicle.config.arm);
        model.traverse((object) => {
          const index = object.userData.liftMount ?? object.userData.motorIndex;
          if (!Number.isInteger(index)) return;
          const position = layout[index].positionFrd;
          object.position.x = position[1] / model.scale.x;
          object.position.z = -position[0] / model.scale.z;
        });
      }
    }
  }
  function updateWorld(world, selectedId, wallDt) {
    const selected = world.vehicles.find(
      (vehicle) => vehicle.id === selectedId,
    );
    if (!selected) throw new Error("Unknown active vehicle");
    if (activeVehicleId !== selectedId) {
      savedTrails.set(activeVehicleId, {
        positions: trailPositions.slice(),
        count,
        lastTime,
      });
    }
    const ids = new Set(world.vehicles.map((vehicle) => vehicle.id));
    for (const [id, peer] of peers)
      if (!ids.has(id) || id === selectedId) {
        if (ids.has(id))
          savedTrails.set(id, {
            positions: peer.trail.geometry.attributes.position.array.slice(),
            count: peer.count,
            lastTime: peer.lastTime,
          });
        else savedTrails.delete(id);
        scene.remove(peer.root, peer.trail);
        peer.trail.geometry.dispose();
        peer.trail.material.dispose();
        peers.delete(id);
      }
    for (const vehicle of world.vehicles) {
      if (vehicle.id === selectedId) continue;
      let peer = peers.get(vehicle.id);
      if (!peer) {
        const root = drone.clone(true);
        delete root.userData.cameraExclude;
        root.userData.cameraVehicleId = vehicle.id;
        root.userData.cameraClass = 7;
        root.visible = true;
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute(
          "position",
          new THREE.BufferAttribute(new Float32Array(1200 * 3), 3),
        );
        geometry.setDrawRange(0, 0);
        const line = new THREE.Line(
          geometry,
          new THREE.LineBasicMaterial({ color: 0xffbf78 }),
        );
        line.frustumCulled = false;
        scene.add(root, line);
        const saved = savedTrails.get(vehicle.id);
        if (saved) {
          geometry.attributes.position.array.set(saved.positions);
          geometry.setDrawRange(0, saved.count);
          geometry.attributes.position.needsUpdate = true;
        }
        peer = {
          root,
          trail: line,
          count: saved?.count || 0,
          lastTime: saved?.lastTime ?? -Infinity,
        };
        peers.set(vehicle.id, peer);
      }
      const snapshot = vehicle.snapshot,
        [north, east, down] = snapshot.position,
        [w, x, y, z] = snapshot.quaternion;
      peer.root.position.set(east, -down, -north);
      peer.root.quaternion.set(y, -z, -x, w).normalize();
      peer.root.children.forEach((child) => {
        if (child.userData.airframe)
          child.visible = child.userData.airframe === snapshot.airframe;
      });
      scaleAircraft(peer.root, vehicle);
      peer.root.traverse((object) => {
        const motor = object.userData.motorIndex;
        if (Number.isInteger(motor))
          object.rotation.y +=
            (object.userData.motorSpin === "ccw" ? 1 : -1) *
            snapshot.motors[motor] *
            Math.max(0, wallDt) *
            90;
        if (object.userData.pusher)
          object.rotation.z +=
            snapshot.propulsion.motor * Math.max(0, wallDt) * 90;
      });
      if (snapshot.time < peer.lastTime) peer.count = 0;
      if (snapshot.time !== peer.lastTime) {
        const positions = peer.trail.geometry.attributes.position.array;
        if (peer.count === 1200) {
          positions.copyWithin(0, 3);
          peer.count--;
        }
        positions.set([east, -down, -north], peer.count++ * 3);
        peer.trail.geometry.setDrawRange(0, peer.count);
        peer.trail.geometry.attributes.position.needsUpdate = true;
        peer.lastTime = snapshot.time;
      }
    }
    if (activeVehicleId !== selectedId) {
      activeVehicleId = selectedId;
      drone.userData.cameraVehicleId = selectedId;
      const saved = savedTrails.get(selectedId);
      count = saved?.count || 0;
      lastTime = saved?.lastTime ?? -Infinity;
      if (saved) trailPositions.set(saved.positions);
      trailGeometry.setDrawRange(0, count);
      trailGeometry.attributes.position.needsUpdate = true;
      const [north, east, down] = selected.snapshot.position;
      target.set(east, -down, -north);
      previousTarget.copy(target);
      resetView();
    }
    scaleAircraft(drone, selected);
    api.update(selected.snapshot, wallDt);
  }
  let count = 0;
  let lastTime = -Infinity;
  let mode = "orbit";
  let visible = true;
  const target = new THREE.Vector3(0, 2, 0);
  const previousTarget = target.clone();
  function resetView() {
    if (mode === "fpv") return;
    controls.target.copy(target);
    camera.position
      .copy(target)
      .add(
        mode === "top"
          ? new THREE.Vector3(
              0.001,
              MULTIROTOR_AIRFRAMES.includes(airframe) ? 8 : 15,
              0,
            )
          : new THREE.Vector3(3, 2, 4).multiplyScalar(
              MULTIROTOR_AIRFRAMES.includes(airframe) ? 1 : 2,
            ),
      );
    controls.update();
  }
  resetView();
  const resize = () => {
    const width = container.clientWidth;
    const height = container.clientHeight;
    if (!width || !height) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };
  const observer = new ResizeObserver(resize);
  observer.observe(container);
  resize();
  const cameraRenderer = createCameraRenderer(renderer, scene);
  const api = {
    captureCamera: cameraRenderer.captureCamera,
    updateWorld,
    update(snapshot, wallDt, missionIndex) {
      const nextAirframe = Object.hasOwn(airframes, snapshot.airframe)
        ? snapshot.airframe
        : "quadrotor";
      if (nextAirframe !== airframe) {
        airframes[airframe].visible = false;
        airframe = nextAirframe;
        airframes[airframe].visible = true;
        if (mode === "follow") resetView();
      }
      const [north, east, down] = snapshot.position;
      const [w, x, y, z] = snapshot.quaternion;
      target.set(east, -down, -north);
      drone.position.copy(target);
      drone.quaternion.set(y, -z, -x, w).normalize();
      projection.visible = terrain.contains(north, east);
      if (projection.visible)
        projection.position.y = terrain.sample(north, east).heightM + 0.02;
      projection.position.x = east;
      projection.position.z = -north;
      if (snapshot.time < lastTime) count = 0;
      if (snapshot.time !== lastTime) {
        if (count === 1200) {
          trailPositions.copyWithin(0, 3);
          count--;
        }
        trailPositions.set([east, -down, -north], count * 3);
        count++;
        trailGeometry.setDrawRange(0, count);
        trailGeometry.attributes.position.needsUpdate = true;
        lastTime = snapshot.time;
      }
      if (MULTIROTOR_AIRFRAMES.includes(airframe)) {
        multirotors[airframe].rotors.forEach(({ rotor, motor }) => {
          rotor.rotation.y +=
            (motor.spin === "ccw" ? 1 : -1) *
            snapshot.motors[motor.index] *
            Math.max(0, wallDt) *
            90;
        });
      } else {
        const model = airframe === "fixedwing" ? fixedwing : vtol;
        model.propeller.rotation.z +=
          (snapshot.propulsion?.motor || 0) * Math.max(0, wallDt) * 90;
        const surfaces = snapshot.controlSurfaces;
        if (surfaces) {
          model.ailerons[0].rotation.x = surfaces.aileron;
          model.ailerons[1].rotation.x = -surfaces.aileron;
          model.elevator.rotation.x = surfaces.elevator;
          model.rudder.rotation.y = surfaces.rudder;
        }
        model.lift.forEach(({ rotor, motor }) => {
          rotor.rotation.y +=
            (motor.spin === "ccw" ? 1 : -1) *
            snapshot.motors[motor.index] *
            Math.max(0, wallDt) *
            90;
        });
      }
      if (mode === "follow") {
        const delta = target.clone().sub(previousTarget);
        camera.position.add(delta);
        controls.target.add(delta);
      }
      if (mode === "fpv") {
        camera.position.copy(target);
        camera.quaternion.copy(drone.quaternion);
      }
      const markers = mission.children.filter((item) => item.userData.waypoint);
      markers.forEach((item, i) =>
        item.material.color.setHex(
          i ===
            (missionIndex === undefined
              ? snapshot.navigation?.index
              : missionIndex)
            ? 0xffffff
            : 0xffbf78,
        ),
      );
      gates.children.forEach((item, i) =>
        item.material.color.setHex(
          snapshot.training?.nextGate && i === snapshot.training.passed
            ? 0xffb46c
            : 0x5edbd5,
        ),
      );
      if (rain && wallDt > 0) {
        const positions = rain.geometry.attributes.position;
        for (let i = 1; i < positions.array.length; i += 3)
          positions.array[i] = (positions.array[i] - wallDt * 8 + 14) % 14;
        positions.needsUpdate = true;
        rain.position.set(target.x, 0, target.z);
      }
      previousTarget.copy(target);
      if (visible) {
        if (mode !== "fpv") controls.update();
        renderer.render(scene, camera);
      }
    },
    setVisible(value) {
      visible = value;
      renderer.domElement.style.display = value ? "" : "none";
      if (value) resize();
    },
    setCamera(value) {
      mode = value;
      controls.enabled = value !== "fpv";
      drone.visible = value !== "fpv";
      camera.fov = value === "fpv" ? 90 : 42;
      camera.updateProjectionMatrix();
      resetView();
    },
    resetView,
    setSensorScan,
    setEnvironment,
    setTerrain,
    setObstacles,
    setNistLane,
    setMission,
    setGates,
    dispose() {
      cameraRenderer.dispose();
      observer.disconnect();
      controls.dispose();
      scene.traverse((object) => {
        object.geometry?.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((item) => {
          item?.map?.dispose();
          item?.dispose();
        });
      });
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
  return api;
}
