const SAMPLE_WINDOW = 240;
const POLL_INTERVAL_MS = 40;

const accCtx = document.getElementById("accChart").getContext("2d");
const gyroCtx = document.getElementById("gyroChart").getContext("2d");
const accValuesEl = document.getElementById("accValues");
const gyroValuesEl = document.getElementById("gyroValues");
const statusEl = document.getElementById("status");

const defaultDataset = () => ({ labels: Array.from({ length: SAMPLE_WINDOW }, (_, i) => i), datasets: [] });

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
  el.innerHTML = `${prefix}x: ${values[0].toFixed(3)} ${units}<br>` +
                  `${prefix}y: ${values[1].toFixed(3)} ${units}<br>` +
                  `${prefix}z: ${values[2].toFixed(3)} ${units}`;
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
      statusEl.textContent = `Source: ${source || "unknown"}`;
    }
  } catch (err) {
    console.warn("config: state poll failed", err);
    statusEl.textContent = "Connection lost… retrying";
  } finally {
    setTimeout(pollState, POLL_INTERVAL_MS);
  }
}

pollState();

