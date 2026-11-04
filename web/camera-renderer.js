import * as THREE from "three";
import { cameraMetadata } from "./camera.js";

// Three.js 0.187.0: WebGLRenderer.readRenderTargetPixelsAsync, RawShaderMaterial,
// and RenderTarget verified against the installed release sources (Context7 ends at r186).
const vertexShader = `
precision highp float;
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
in vec3 position;
out highp float cameraDepth;
void main() {
  vec4 viewPosition = modelViewMatrix * vec4(position, 1.0);
  cameraDepth = -viewPosition.z;
  gl_Position = projectionMatrix * viewPosition;
}`;
const depthFragment = `
precision highp float;
in highp float cameraDepth;
uniform float nearM;
uniform float farM;
out vec4 outputColor;
void main() {
  if (cameraDepth < nearM || cameraDepth > farM) discard;
  float mm = floor(cameraDepth * 1000.0 + 0.5);
  outputColor = vec4(floor(mm / 65536.0), floor(mod(mm, 65536.0) / 256.0), mod(mm, 256.0), 255.0) / 255.0;
}`;
const classFragment = `
precision highp float;
in highp float cameraDepth;
uniform float nearM;
uniform float farM;
uniform float classId;
out vec4 outputColor;
void main() {
  if (cameraDepth < nearM || cameraDepth > farM) discard;
  outputColor = vec4(classId / 255.0, 0.0, 0.0, 1.0);
}
`;
const worldVector = ([north, east, down]) => [east, -down, -north];
function cameraClass(object) {
  let id = 0;
  for (let current = object; current; current = current.parent) {
    if (current.userData.cameraExclude) return -1;
    if (!id && Number.isInteger(current.userData.cameraClass))
      id = current.userData.cameraClass;
  }
  return id;
}
export function createCameraRenderer(renderer, scene) {
  const gl = renderer.getContext();
  const camera = new THREE.PerspectiveCamera();
  const materials = new Map();
  let targets = [],
    busy = false,
    disposed = false,
    contextLost = gl.isContextLost();
  const lost = () => {
    contextLost = true;
  };
  const restored = () => {
    contextLost = false;
  };
  renderer.domElement.addEventListener("webglcontextlost", lost);
  renderer.domElement.addEventListener("webglcontextrestored", restored);
  function checkAvailable() {
    if (disposed) throw new Error("Camera renderer has been disposed");
    if (contextLost || gl.isContextLost())
      throw new Error("Camera capture unavailable: WebGL context lost");
  }
  function materialFor(kind, id, side) {
    const key = `${kind}:${id}:${side}`;
    if (!materials.has(key))
      materials.set(
        key,
        new THREE.RawShaderMaterial({
          name: `Camera ${kind} ${id}`,
          glslVersion: THREE.GLSL3,
          vertexShader,
          fragmentShader: kind === "depth" ? depthFragment : classFragment,
          uniforms: {
            nearM: { value: 0 },
            farM: { value: 0 },
            classId: { value: id },
          },
          side,
          blending: THREE.NoBlending,
          depthTest: true,
          depthWrite: true,
          transparent: false,
          toneMapped: false,
          fog: false,
        }),
      );
    return materials.get(key);
  }
  function configure(metadata) {
    if (
      !targets.length ||
      targets[0].width !== metadata.width ||
      targets[0].height !== metadata.height
    ) {
      targets.forEach((target) => target.dispose());
      targets = [
        THREE.SRGBColorSpace,
        THREE.NoColorSpace,
        THREE.NoColorSpace,
      ].map(
        (colorSpace) =>
          new THREE.WebGLRenderTarget(metadata.width, metadata.height, {
            format: THREE.RGBAFormat,
            type: THREE.UnsignedByteType,
            colorSpace,
            minFilter: THREE.NearestFilter,
            magFilter: THREE.NearestFilter,
            generateMipmaps: false,
            depthBuffer: true,
            stencilBuffer: false,
            samples: 0,
          }),
      );
    }
    camera.fov = metadata.intrinsics.verticalFovDeg;
    camera.aspect = metadata.width / metadata.height;
    camera.near = metadata.nearM;
    camera.far = metadata.farM;
    camera.updateProjectionMatrix();
    const rotation = metadata.pose.opticalToNed;
    const right = worldVector([rotation[0], rotation[3], rotation[6]]);
    const up = worldVector([-rotation[1], -rotation[4], -rotation[7]]);
    const back = worldVector([-rotation[2], -rotation[5], -rotation[8]]);
    const position = worldVector(metadata.pose.positionNed);
    camera.matrixWorld.set(
      right[0],
      up[0],
      back[0],
      position[0],
      right[1],
      up[1],
      back[1],
      position[1],
      right[2],
      up[2],
      back[2],
      position[2],
      0,
      0,
      0,
      1,
    );
    camera.matrixWorld.decompose(
      camera.position,
      camera.quaternion,
      camera.scale,
    );
    camera.updateMatrixWorld(true);
  }
  function disposeResources() {
    targets.forEach((target) => target.dispose());
    targets = [];
    materials.forEach((material) => material.dispose());
    materials.clear();
  }
  async function captureCamera(snapshot, config = {}) {
    checkAvailable();
    if (busy) throw new Error("A camera capture is already in progress");
    const metadata = cameraMetadata(snapshot, config);
    busy = true;
    let reads = [];
    try {
      configure(metadata);
      const buffers = [0, 1, 2].map(
        () => new Uint8Array(metadata.width * metadata.height * 4),
      );
      const objects = [];
      scene.traverse((object) => {
        if (
          object.isMesh ||
          object.isLine ||
          object.isPoints ||
          object.isSprite
        )
          objects.push({
            object,
            visible: object.visible,
            material: object.material,
            classId: cameraClass(object),
            rgbOnly: object.userData.cameraRgbOnly === true,
          });
      });
      const saved = {
        target: renderer.getRenderTarget(),
        cubeFace: renderer.getActiveCubeFace(),
        mip: renderer.getActiveMipmapLevel(),
        viewport: renderer.getViewport(new THREE.Vector4()),
        scissor: renderer.getScissor(new THREE.Vector4()),
        scissorTest: renderer.getScissorTest(),
        clearColor: renderer.getClearColor(new THREE.Color()),
        clearAlpha: renderer.getClearAlpha(),
        autoClear: renderer.autoClear,
        autoClearColor: renderer.autoClearColor,
        autoClearDepth: renderer.autoClearDepth,
        autoClearStencil: renderer.autoClearStencil,
        outputColorSpace: renderer.outputColorSpace,
        toneMapping: renderer.toneMapping,
        toneMappingExposure: renderer.toneMappingExposure,
        clippingPlanes: renderer.clippingPlanes,
        localClippingEnabled: renderer.localClippingEnabled,
        xr: renderer.xr.enabled,
        overrideMaterial: scene.overrideMaterial,
        background: scene.background,
        fog: scene.fog,
        shaderError: renderer.debug.onShaderError,
        checkShaderErrors: renderer.debug.checkShaderErrors,
      };
      try {
        renderer.xr.enabled = false;
        renderer.clippingPlanes = [];
        renderer.localClippingEnabled = false;
        renderer.autoClear = true;
        renderer.autoClearColor = true;
        renderer.autoClearDepth = true;
        renderer.autoClearStencil = true;
        renderer.outputColorSpace = THREE.SRGBColorSpace;
        scene.overrideMaterial = null;
        renderer.debug.checkShaderErrors = true;
        renderer.debug.onShaderError = () => {
          throw new Error("Camera shader compilation failed");
        };
        for (let pass = 0; pass < 3; pass++) {
          checkAvailable();
          const rgb = pass === 0,
            kind = pass === 1 ? "depth" : "classes";
          scene.background = rgb ? saved.background : null;
          scene.fog = rgb ? saved.fog : null;
          renderer.toneMapping = rgb ? saved.toneMapping : THREE.NoToneMapping;
          renderer.setClearColor(
            rgb ? saved.clearColor : 0x000000,
            rgb ? 1 : 0,
          );
          for (const entry of objects) {
            const { object, classId } = entry;
            const included =
              (classId >= 1 && classId <= 6 && object.isMesh) ||
              (rgb && entry.rgbOnly && classId !== -1);
            object.visible = entry.visible && included;
            if (rgb || !included) object.material = entry.material;
            else {
              const replace = (original) => {
                const material = materialFor(kind, classId, original.side);
                material.uniforms.nearM.value = metadata.nearM;
                material.uniforms.farM.value = metadata.farM;
                return material;
              };
              object.material = Array.isArray(entry.material)
                ? entry.material.map(replace)
                : replace(entry.material);
            }
          }
          renderer.setRenderTarget(targets[pass]);
          renderer.clear(true, true, true);
          renderer.render(scene, camera);
          // The r187 method submits its PBO read/fence synchronously before its first await.
          reads.push(
            renderer.readRenderTargetPixelsAsync(
              targets[pass],
              0,
              0,
              metadata.width,
              metadata.height,
              buffers[pass],
            ),
          );
        }
      } finally {
        for (const entry of objects) {
          entry.object.visible = entry.visible;
          entry.object.material = entry.material;
        }
        scene.overrideMaterial = saved.overrideMaterial;
        scene.background = saved.background;
        scene.fog = saved.fog;
        renderer.xr.enabled = saved.xr;
        renderer.clippingPlanes = saved.clippingPlanes;
        renderer.localClippingEnabled = saved.localClippingEnabled;
        renderer.autoClear = saved.autoClear;
        renderer.autoClearColor = saved.autoClearColor;
        renderer.autoClearDepth = saved.autoClearDepth;
        renderer.autoClearStencil = saved.autoClearStencil;
        renderer.outputColorSpace = saved.outputColorSpace;
        renderer.toneMapping = saved.toneMapping;
        renderer.toneMappingExposure = saved.toneMappingExposure;
        renderer.debug.onShaderError = saved.shaderError;
        renderer.debug.checkShaderErrors = saved.checkShaderErrors;
        renderer.setClearColor(saved.clearColor, saved.clearAlpha);
        renderer.setRenderTarget(saved.target, saved.cubeFace, saved.mip);
        renderer.setViewport(saved.viewport);
        renderer.setScissor(saved.scissor);
        renderer.setScissorTest(saved.scissorTest);
      }
      await Promise.all(reads);
      checkAvailable();
      const rgbRgba = new Uint8Array(buffers[0].length),
        depthM = new Float32Array(metadata.width * metadata.height),
        classes = new Uint8Array(metadata.width * metadata.height);
      for (let row = 0; row < metadata.height; row++) {
        const source = (metadata.height - 1 - row) * metadata.width;
        const destination = row * metadata.width;
        rgbRgba.set(
          buffers[0].subarray(source * 4, (source + metadata.width) * 4),
          destination * 4,
        );
        for (let column = 0; column < metadata.width; column++) {
          const from = (source + column) * 4,
            to = destination + column;
          if (buffers[1][from + 3] !== 0)
            depthM[to] =
              (buffers[1][from] * 65536 +
                buffers[1][from + 1] * 256 +
                buffers[1][from + 2]) /
              1000;
          classes[to] = buffers[2][from + 3] === 0 ? 0 : buffers[2][from];
        }
      }
      return { ...metadata, rgbRgba, depthM, classes };
    } catch (error) {
      await Promise.allSettled(reads);
      if (contextLost || gl.isContextLost())
        throw new Error("Camera capture failed: WebGL context lost");
      throw error instanceof Error
        ? error
        : new Error("Camera GPU readback failed");
    } finally {
      busy = false;
      if (disposed) disposeResources();
    }
  }
  return {
    captureCamera,
    dispose() {
      if (disposed) return;
      disposed = true;
      renderer.domElement.removeEventListener("webglcontextlost", lost);
      renderer.domElement.removeEventListener("webglcontextrestored", restored);
      if (!busy) disposeResources();
    },
  };
}
