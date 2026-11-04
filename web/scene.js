import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export function createScene(container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#101c2d");
  scene.fog = new THREE.Fog("#101c2d", 20, 65);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 150);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.setAttribute("aria-label", "Quadrotor flight simulation");
  container.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxDistance = 35;
  controls.minDistance = 1.5;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;

  scene.add(new THREE.HemisphereLight(0xc2e9ff, 0x192236, 2));
  const key = new THREE.DirectionalLight(0xffd7b2, 2.8);
  key.position.set(4, 9, 5);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x64dce2, 1.3);
  fill.position.set(-6, 4, -5);
  scene.add(fill);
  const grid = new THREE.GridHelper(60, 120, 0x35566b, 0x233c50);
  grid.material.transparent = true;
  grid.material.opacity = 0.48;
  scene.add(grid);

  const material = (color, metalness = 0, roughness = 0.5) =>
    new THREE.MeshStandardMaterial({ color, metalness, roughness });
  const dark = material(0x1d2934, 0.65, 0.36);
  const shell = material(0x354654, 0.55, 0.36);
  const black = material(0x101820, 0.35, 0.45);
  const teal = material(0x56d7d1, 0.45, 0.3);
  const orange = material(0xffa568, 0.4, 0.35);
  const rotorMaterial = material(0x82969d, 0.6, 0.36);
  const drone = new THREE.Group();
  drone.scale.setScalar(2.2);
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
  box(drone, [0.29, 0.085, 0.34], [0, 0, 0], dark);
  box(drone, [0.23, 0.055, 0.27], [0, 0.065, 0], shell);
  box(drone, [0.12, 0.012, 0.16], [0, 0.1, -0.015], black);
  box(drone, [0.1, 0.015, 0.025], [0, 0.11, -0.06], teal);
  const rotors = [];
  const arm = 0.23;
  for (const [x, z] of [
    [arm, -arm],
    [arm, arm],
    [-arm, arm],
    [-arm, -arm],
  ]) {
    const beam = box(
      drone,
      [0.075, 0.045, Math.hypot(x, z)],
      [x / 2, 0, z / 2],
      shell,
    );
    beam.rotation.y = Math.atan2(x, z);
    cylinder(drone, 0.047, 0.045, [x, 0.033, z], dark);
    cylinder(drone, 0.036, 0.015, [x, 0.063, z], x > 0 ? teal : orange);
    const rotor = new THREE.Group();
    rotor.position.set(x, 0.079, z);
    box(rotor, [0.028, 0.008, 0.31], [0, 0, 0], rotorMaterial);
    cylinder(rotor, 0.017, 0.014, [0, 0.006, 0], black);
    drone.add(rotor);
    rotors.push(rotor);
  }
  for (const x of [-0.12, 0.12]) {
    box(drone, [0.022, 0.15, 0.022], [x, -0.095, 0.09], dark).rotation.z =
      x > 0 ? -0.15 : 0.15;
    box(drone, [0.022, 0.15, 0.022], [x, -0.095, -0.09], dark).rotation.z =
      x > 0 ? -0.15 : 0.15;
    box(drone, [0.035, 0.022, 0.29], [x * 1.1, -0.17, 0], black);
  }
  box(drone, [0.105, 0.075, 0.06], [0, -0.025, -0.2], black);
  const lens = cylinder(drone, 0.027, 0.015, [0, -0.025, -0.237], teal);
  lens.rotation.x = Math.PI / 2;

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
    scene.add(ring);
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
    scene.add(mesh);
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
  let count = 0;
  let lastTime = -Infinity;
  let mode = "orbit";
  let visible = true;
  const target = new THREE.Vector3(0, 2, 0);
  const previousTarget = target.clone();
  function resetView() {
    controls.target.copy(target);
    camera.position
      .copy(target)
      .add(
        mode === "top"
          ? new THREE.Vector3(0.001, 8, 0)
          : new THREE.Vector3(3, 2, 4),
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
  return {
    update(snapshot, wallDt) {
      const [north, east, down] = snapshot.position;
      const [w, x, y, z] = snapshot.quaternion;
      target.set(east, -down, -north);
      drone.position.copy(target);
      drone.quaternion.set(y, -z, -x, w).normalize();
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
      rotors.forEach((rotor, i) => {
        rotor.rotation.y +=
          (i % 2 ? -1 : 1) *
          (snapshot.motors[i] || 0) *
          Math.max(0, wallDt) *
          90;
      });
      if (mode === "follow") {
        const delta = target.clone().sub(previousTarget);
        camera.position.add(delta);
        controls.target.add(delta);
      }
      previousTarget.copy(target);
      if (visible) {
        controls.update();
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
      resetView();
    },
    resetView,
    dispose() {
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
}
