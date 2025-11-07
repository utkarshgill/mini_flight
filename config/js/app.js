import * as THREE from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";

const SAMPLE_WINDOW = 240;
const POLL_INTERVAL_MS = 40;

const accCtx = document.getElementById("accChart").getContext("2d");
const gyroCtx = document.getElementById("gyroChart").getContext("2d");
const accValuesEl = document.getElementById("accValues");
const gyroValuesEl = document.getElementById("gyroValues");
const attitudeValuesEl = document.getElementById("attitudeValues");
const statusEl = document.getElementById("status");
const vizContainer = document.getElementById("attitudeViz");
const tempQuaternion = new THREE.Quaternion();

class AttitudeVisualizer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();

    const { width, height } = this._getSize();
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    this.camera.position.set(2.5, 1.8, 2.6);
    this.camera.lookAt(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.setSize(width, height);
    container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0xf8fafc, 0.6);
    const directional = new THREE.DirectionalLight(0xffffff, 0.7);
    directional.position.set(5, 8, 6);
    this.scene.add(ambient, directional);

    const grid = new THREE.GridHelper(4, 8, 0x1f2937, 0x111827);
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(1.5);
    axes.material.depthTest = false;
    axes.renderOrder = 1;
    this.scene.add(axes);

    this.drone = new THREE.Group();
    this.scene.add(this.drone);
    this._loader = new OBJLoader();
    this._loadModel();

    this.currentQuaternion = new THREE.Quaternion();
    this.targetQuaternion = new THREE.Quaternion();
    this.initialized = false;

    this._animate = this._animate.bind(this);
    this._onResize = this._onResize.bind(this);
    window.addEventListener("resize", this._onResize);
    this._resizeObserver = new ResizeObserver(this._onResize);
    this._resizeObserver.observe(container);

    requestAnimationFrame(this._animate);
  }

  setOrientation(quaternion) {
    if (!quaternion) return;
    if (!this.initialized) {
      this.currentQuaternion.copy(quaternion);
      this.initialized = true;
    }
    this.targetQuaternion.copy(quaternion);
  }

  async _loadModel() {
    try {
      const raw = await this._loader.loadAsync("/data/drone_costum.obj");
      const material = new THREE.MeshStandardMaterial({
        color: 0x94a3b8,
        metalness: 0.4,
        roughness: 0.55,
      });

      raw.traverse((child) => {
        if (child.isMesh) {
          child.material = material;
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });

      const body = new THREE.Group();
      body.add(raw);

      const preBox = new THREE.Box3().setFromObject(body);
      const size = preBox.getSize(new THREE.Vector3());
      const span = Math.max(size.x, size.y, size.z) || 1;
      body.scale.setScalar(1.6 / span);

      body.rotation.x = Math.PI / 2;
      body.rotation.z = Math.PI;

      const pivot = new THREE.Group();
      pivot.add(body);

      const box = new THREE.Box3().setFromObject(pivot);
      const center = box.getCenter(new THREE.Vector3());
      body.position.sub(center);

      this.scene.remove(this.drone);
      this.drone = pivot;
      this.scene.add(this.drone);
    } catch (err) {
      console.warn("attitude: failed to load OBJ", err);
    }
  }

  _getSize() {
    const width = this.container.clientWidth || this.container.offsetWidth || 320;
    const height = this.container.clientHeight || 280;
    return { width, height };
  }

  _onResize() {
    const { width, height } = this._getSize();
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  _animate() {
    this.currentQuaternion.slerp(this.targetQuaternion, 0.18);
    this.drone.setRotationFromQuaternion(this.currentQuaternion);
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this._animate);
  }
}

const attitudeViz = vizContainer ? new AttitudeVisualizer(vizContainer) : null;

const defaultDataset = () => ({
  labels: Array.from({ length: SAMPLE_WINDOW }, (_, i) => i),
  datasets: [],
});

const chartOptions = (min, max) => ({
  animation: false,
  responsive: true,
  maintainAspectRatio: false,
  scales: {
    x: { display: false },
    y: {
      min,
      max,
      ticks: { color: "#94a3b8" },
      grid: { color: "rgba(148, 163, 184, 0.2)" },
    },
  },
  plugins: {
    legend: { labels: { color: "#cbd5f5" } },
  },
});

const palette = ["#38bdf8", "#4ade80", "#f97316"];

function buildDatasets(labelPrefix) {
  return [0, 1, 2].map((axis) => ({
    label: `${labelPrefix}${axis === 0 ? "x" : axis === 1 ? "y" : "z"}`,
    data: Array(SAMPLE_WINDOW).fill(0),
    borderColor: palette[axis],
    borderWidth: 2,
    tension: 0.25,
    pointRadius: 0,
  }));
}

const accChart = new Chart(accCtx, {
  type: "line",
  data: { ...defaultDataset(), datasets: buildDatasets("a") },
  options: chartOptions(-2, 2),
});

const gyroChart = new Chart(gyroCtx, {
  type: "line",
  data: { ...defaultDataset(), datasets: buildDatasets("g") },
  options: chartOptions(-500, 500),
});

function pushSample(chart, values, throttle) {
  chart.data.datasets.forEach((dataset, idx) => {
    dataset.data.push(values[idx]);
    if (dataset.data.length > SAMPLE_WINDOW) dataset.data.shift();
  });
  if (throttle.tick()) chart.update("none");
}

function renderVector(el, prefix, values, units) {
  el.innerHTML = [
    `<span>${prefix}x: ${values[0].toFixed(3)} ${units}</span>`,
    `<span>${prefix}y: ${values[1].toFixed(3)} ${units}</span>`,
    `<span>${prefix}z: ${values[2].toFixed(3)} ${units}</span>`,
  ].join("");
}

function renderAngles(el, angles) {
  if (!el || !angles) return;
  const [roll, pitch, yaw] = Array.isArray(angles)
    ? angles
    : [angles.roll, angles.pitch, angles.yaw];
  if (![roll, pitch, yaw].every((value) => Number.isFinite(value))) return;
  el.innerHTML = [
    `<span>roll: ${roll.toFixed(1)}°</span>`,
    `<span>pitch: ${pitch.toFixed(1)}°</span>`,
    `<span>yaw: ${yaw.toFixed(1)}°</span>`,
  ].join("");
}

function createThrottle(interval = 4) {
  let counter = 0;
  return {
    tick() {
      counter = (counter + 1) % interval;
      return counter === 0;
    },
  };
}

const accThrottle = createThrottle(1);
const gyroThrottle = createThrottle(1);

async function pollState() {
  try {
    const response = await fetch("/state", { cache: "no-store" });
    if (!response.ok) throw new Error(response.statusText);
    const data = await response.json();
    if (data?.imu) {
      const { accel = [0, 0, 0], gyro = [0, 0, 0], source = "" } = data.imu;
      pushSample(accChart, accel, accThrottle);
      pushSample(gyroChart, gyro, gyroThrottle);
      renderVector(accValuesEl, "a", accel, "g");
      renderVector(gyroValuesEl, "g", gyro, "°/s");

      const attitude = data.attitude;
      if (attitudeViz && attitude?.quaternion?.length === 4) {
        const [w, x, y, z] = attitude.quaternion.map(Number);
        if ([w, x, y, z].every((value) => Number.isFinite(value))) {
          tempQuaternion.set(x, y, z, w);
          attitudeViz.setOrientation(tempQuaternion);
        }
      }
      if (attitude?.euler_deg) {
        renderAngles(attitudeValuesEl, attitude.euler_deg);
      }

      const filterName = attitude?.filter || "unknown";
      statusEl.textContent = `Source: ${source || "unknown"} | Filter: ${filterName}`;
    }
  } catch (err) {
    console.warn("config: state poll failed", err);
    statusEl.textContent = "Connection lost… retrying";
    if (attitudeValuesEl) attitudeValuesEl.textContent = "Waiting…";
  } finally {
    setTimeout(pollState, POLL_INTERVAL_MS);
  }
}

pollState();

document.getElementById("resetYawBtn").addEventListener("click", async () => {
  try {
    await fetch("/reset_yaw", { method: "POST" });
  } catch (err) {
    console.error("config: failed to reset yaw", err);
  }
});

