import * as THREE from "three";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js";
import { OBJLoader } from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/loaders/OBJLoader.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const SKY_COLOR = 0x15263f;
const GROUND_COLOR = 0x1e2a3f;

const scene = new THREE.Scene();
scene.background = new THREE.Color(SKY_COLOR);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 100);
camera.position.set(3.5, 2.2, 3.5);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;
controls.target.set(0, 0, 0);
controls.update();

const hemi = new THREE.HemisphereLight(0xf8fafc, 0x0f172a, 0.8);
scene.add(hemi);
const dir = new THREE.DirectionalLight(0xffffff, 0.9);
dir.position.set(5, 8, 4);
scene.add(dir);

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(60, 60),
  new THREE.MeshStandardMaterial({ color: GROUND_COLOR, roughness: 0.85, metalness: 0.05 })
);
ground.receiveShadow = true;
ground.position.z = -0.002;
scene.add(ground);

const grid = new THREE.GridHelper(20, 20, 0x41556f, 0x2a364a);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.001;
scene.add(grid);

function addBuildings() {
  const parcels = [
    { x: 6, y: 4, w: 2.5, d: 2.3, h: 2.8 },
    { x: -5, y: 3.5, w: 2.8, d: 1.8, h: 3.4 },
    { x: 4.5, y: -4.5, w: 3.2, d: 2.6, h: 4.2 },
    { x: -6.5, y: -3.5, w: 2.4, d: 2.7, h: 3.8 },
    { x: 0, y: -7, w: 3.5, d: 3.1, h: 5.0 },
  ];

  const windowColor = new THREE.Color(0x0f172a);
  const roofColors = [0x1f2937, 0x111827, 0x0b0f19];

  parcels.forEach(({ x, y, w, d, h }, idx) => {
    const bodyGeom = new THREE.BoxGeometry(w, d, h);
    const roofGeom = new THREE.BoxGeometry(w * 0.9, d * 0.9, h * 0.15);

    const facade = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.6, metalness: 0.1 });
    const detail = new THREE.MeshStandardMaterial({ color: 0x374151, roughness: 0.55, metalness: 0.15 });

    const building = new THREE.Mesh(bodyGeom, [facade, detail, facade, detail, detail, facade]);
    building.position.set(x, y, h / 2);
    building.castShadow = building.receiveShadow = true;
    scene.add(building);

    const roof = new THREE.Mesh(
      roofGeom,
      new THREE.MeshStandardMaterial({ color: roofColors[idx % roofColors.length], roughness: 0.4, metalness: 0.3 })
    );
    roof.position.set(x, y, h + (h * 0.075));
    roof.castShadow = roof.receiveShadow = true;
    scene.add(roof);

    const windowHeight = h / 6;
    const windowSpacing = windowHeight * 1.05;
    const baseHeight = Math.max(windowHeight * 0.6 + 0.05, h * 0.38);

    for (let row = -2; row <= 2; row++) {
      for (let col = -2; col <= 2; col++) {
        if (Math.abs(row) === 2 && Math.abs(col) === 2) continue;
        const windowGeom = new THREE.PlaneGeometry(w / 6, windowHeight);
        const windowMat = new THREE.MeshStandardMaterial({ color: windowColor, emissive: 0x1e293b, emissiveIntensity: 0.5 });
        const windowMesh = new THREE.Mesh(windowGeom, windowMat);
        const windowZ = baseHeight + row * windowSpacing;
        windowMesh.position.set(x + (w / 2 + 0.01), y + (col * (d / 6)), windowZ);
        windowMesh.rotation.y = Math.PI / 2;
        scene.add(windowMesh);

        const windowMeshBack = windowMesh.clone();
        windowMeshBack.position.set(x - (w / 2 + 0.01), y + (col * (d / 6)), windowZ);
        windowMeshBack.rotation.y = -Math.PI / 2;
        scene.add(windowMeshBack);
      }
    }
  });
}
addBuildings();

const axes = new THREE.AxesHelper(0.5);
axes.position.set(0, 0, 0.005);
axes.visible = false;
scene.add(axes);

const overlayTime = document.getElementById("sim-time");
const viewLabel = document.getElementById("view-mode");
const bodyMeshes = new Map();
const quadLoader = new OBJLoader();
let quadModelPromise = null;
const viewModes = ["global", "fpv"];
let viewModeIndex = 0;
let viewMode = viewModes[viewModeIndex];
const scratchVecA = new THREE.Vector3();
const scratchVecB = new THREE.Vector3();

function applyViewModeSettings() {
  const isGlobal = viewMode === "global";
  controls.enabled = isGlobal;
  controls.enablePan = isGlobal;
  controls.enableRotate = isGlobal;
  controls.enableZoom = isGlobal;
  if (viewLabel) {
    viewLabel.textContent = `View: ${viewMode.toUpperCase()}`;
  }
}

function setViewMode(mode) {
  if (!viewModes.includes(mode)) return;
  viewMode = mode;
  viewModeIndex = viewModes.indexOf(mode);
  applyViewModeSettings();
}

function cycleViewMode() {
  viewModeIndex = (viewModeIndex + 1) % viewModes.length;
  setViewMode(viewModes[viewModeIndex]);
}

function loadQuadModel() {
  if (quadModelPromise) return quadModelPromise;
  const OBJ_URL = "/data/drone_costum.obj";
  quadModelPromise = new Promise((resolve, reject) => {
    quadLoader.load(
      OBJ_URL,
      (obj) => {
        const palette = [0xd1d5db, 0x9ca3af, 0x374151, 0x111827];
        let meshIndex = 0;
        obj.traverse((child) => {
          if (child.isMesh) {
            const color = palette[meshIndex % palette.length];
            child.material = new THREE.MeshStandardMaterial({
              color,
              metalness: 0.35,
              roughness: 0.45,
            });
            child.castShadow = child.receiveShadow = true;
            meshIndex += 1;
          }
        });
        resolve(obj);
      },
      undefined,
      (err) => reject(err)
    );
  });
  return quadModelPromise;
}

function createQuadMesh(body) {
  const group = new THREE.Group();
  const armLength = body.arm_length ?? 0.3;

  const placeholder = new THREE.Group();
  const bodyGeom = new THREE.BoxGeometry(0.18, 0.08, 0.05);
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0x9ca3af, roughness: 0.55, metalness: 0.2 });
  const fuselage = new THREE.Mesh(bodyGeom, bodyMat);
  placeholder.add(fuselage);

  const armGeom = new THREE.BoxGeometry(armLength * 2.2, 0.015, 0.015);
  const armMat = new THREE.MeshStandardMaterial({ color: 0x4b5563, roughness: 0.4, metalness: 0.25 });
  const armX = new THREE.Mesh(armGeom, armMat);
  const armY = armX.clone();
  armY.rotation.z = Math.PI / 2;
  placeholder.add(armX, armY);

  const rotorGeom = new THREE.CylinderGeometry(0.05, 0.05, 0.01, 24);
  const rotorMatA = new THREE.MeshStandardMaterial({ color: 0xe5e7eb, roughness: 0.35, metalness: 0.1 });
  const rotorMatB = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.5, metalness: 0.15 });
  const rotorPositions = [
    new THREE.Vector3(armLength, 0, 0.025),
    new THREE.Vector3(-armLength, 0, 0.025),
    new THREE.Vector3(0, armLength, 0.025),
    new THREE.Vector3(0, -armLength, 0.025),
  ];
  rotorPositions.forEach((position, idx) => {
    const rotor = new THREE.Mesh(rotorGeom, idx % 2 === 0 ? rotorMatA : rotorMatB);
    rotor.rotateX(Math.PI / 2);
    rotor.position.copy(position);
    placeholder.add(rotor);
  });

  group.add(placeholder);

  loadQuadModel()
    .then((model) => {
      const clone = model.clone(true);
      const scale = (armLength ?? 0.3) * 0.4;
      clone.scale.setScalar(scale);
      clone.rotation.x = Math.PI / 2;
      clone.rotation.y = Math.PI / 2;
      group.clear();
      group.add(clone);
    })
    .catch((err) => {
      console.warn("renderer: failed to load quad model", err);
    });

  return group;
}

function createBoxMesh(body) {
  const size = (body.half_height ?? 0.1) * 2;
  const geom = new THREE.BoxGeometry(size, size, size);
  const mat = new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.6 });
  const cube = new THREE.Mesh(geom, mat);
  cube.castShadow = cube.receiveShadow = true;
  return cube;
}

function createGenericMesh() {
  const geom = new THREE.SphereGeometry(0.05, 24, 24);
  const mat = new THREE.MeshStandardMaterial({ color: 0x818cf8, emissive: 0x1f2937, emissiveIntensity: 0.2 });
  return new THREE.Mesh(geom, mat);
}

function ensureMesh(body) {
  const id = String(body.id);
  if (bodyMeshes.has(id)) {
    return bodyMeshes.get(id);
  }

  let mesh;
  switch (body.kind) {
    case "quadrotor":
      mesh = createQuadMesh(body);
      break;
    case "box":
      mesh = createBoxMesh(body);
      break;
    default:
      mesh = createGenericMesh(body);
      break;
  }

  mesh.userData.label = body.label ?? `Body ${body.id}`;
  bodyMeshes.set(id, mesh);
  scene.add(mesh);
  return mesh;
}

function applyTransform(mesh, body) {
  const [x, y, z] = body.position ?? [0, 0, 0];
  const [qx, qy, qz, qw] = body.orientation ?? [0, 0, 0, 1];
  mesh.position.set(x, y, z);
  mesh.quaternion.set(qx, qy, qz, qw);
}

function updateScene(snapshot) {
  const seen = new Set();
  const bodies = Array.isArray(snapshot?.bodies) ? snapshot.bodies : [];
  let primaryMesh = null;

  bodies.forEach((body) => {
    const mesh = ensureMesh(body);
    applyTransform(mesh, body);
    if (body.id === 0 && primaryMesh == null) {
      primaryMesh = mesh;
    }
    seen.add(String(body.id));
  });

  for (const [id, mesh] of bodyMeshes.entries()) {
    if (!seen.has(id)) {
      scene.remove(mesh);
      bodyMeshes.delete(id);
    }
  }

  if (typeof snapshot?.world_time === "number" && overlayTime) {
    overlayTime.textContent = `Simulation t = ${snapshot.world_time.toFixed(2)} s`;
  }

  if (primaryMesh) {
    primaryMesh.updateMatrixWorld(true);
    if (viewMode === "fpv") {
      const eyePos = primaryMesh.localToWorld(scratchVecA.set(0.12, 0, 0.05));
      const lookTarget = primaryMesh.localToWorld(scratchVecB.set(0.35, 0, 0.02));
      camera.position.copy(eyePos);
      camera.up.set(0, 0, 1);
      camera.lookAt(lookTarget);
    } else if (controls.enabled) {
      const target = primaryMesh.getWorldPosition(scratchVecA.set(0, 0, 0));
      controls.target.lerp(target, 0.12);
    }
  }
}

async function pollState() {
  try {
    const response = await fetch("/state", { cache: "no-store" });
    if (response.ok) {
      const data = await response.json();
      updateScene(data);
    }
  } catch (err) {
    console.warn("renderer: state poll failed", err);
  } finally {
    setTimeout(pollState, 50);
  }
}

const trackedKeys = new Set(["w", "a", "s", "d", "left", "right", "up", "down", "x", " "]);
const pressedKeys = new Set();
let inputDirty = false;

function normaliseKey(event) {
  const key = event.key;
  switch (key) {
    case "ArrowUp":
      return "up";
    case "ArrowDown":
      return "down";
    case "ArrowLeft":
      return "left";
    case "ArrowRight":
      return "right";
    case " ":
      return " ";
    case "x":
    case "X":
      return "x";
    default: {
      if (key.length === 1) {
        const lower = key.toLowerCase();
        if (trackedKeys.has(lower)) return lower;
      }
    }
  }
  return null;
}

function markDirty(key, pressed) {
  if (!trackedKeys.has(key)) return;
  const hadKey = pressedKeys.has(key);
  if (pressed && !hadKey) {
    pressedKeys.add(key);
    inputDirty = true;
  } else if (!pressed && hadKey) {
    pressedKeys.delete(key);
    inputDirty = true;
  }
}

window.addEventListener("keydown", (event) => {
  if (event.key === "v" || event.key === "V") {
    cycleViewMode();
    return;
  }
  const key = normaliseKey(event);
  if (key == null) return;
  markDirty(key, true);
});

window.addEventListener("keyup", (event) => {
  if (event.key === "v" || event.key === "V") {
    return;
  }
  const key = normaliseKey(event);
  if (key == null) return;
  markDirty(key, false);
});

window.addEventListener("blur", () => {
  if (pressedKeys.size > 0) {
    pressedKeys.clear();
    inputDirty = true;
  }
});

async function flushInput() {
  if (inputDirty) {
    inputDirty = false;
    try {
      await fetch("/input", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ keys: Array.from(pressedKeys.values()) }),
        keepalive: true,
      });
    } catch (err) {
      console.warn("renderer: input send failed", err);
    }
  }
  setTimeout(flushInput, 80);
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

window.addEventListener("resize", resize);

function animate() {
  requestAnimationFrame(animate);
  if (controls.enabled) {
    controls.update();
  }
  renderer.render(scene, camera);
}

setViewMode(viewMode);
pollState();
flushInput();
animate();


