import * as THREE from "/static/three.module.js";

const canvas = document.querySelector("#orientation");
const connection = document.querySelector("#connection");
const rateInput = document.querySelector("#rate");
const rateValue = document.querySelector("#rate-value");
const valuesEl = document.querySelector("#values");
const healthEl = document.querySelector("#health");
const fieldInputs = [...document.querySelectorAll("[data-field]")];

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x151b17);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
camera.position.set(3.2, 2.4, 4.2);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const light = new THREE.DirectionalLight(0xffffff, 2.4);
light.position.set(3, 5, 4);
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 0.45));

const body = new THREE.Group();
const cube = new THREE.Mesh(
  new THREE.BoxGeometry(2.1, 0.34, 1.25),
  new THREE.MeshStandardMaterial({ color: 0x56c271, roughness: 0.42, metalness: 0.05 }),
);
body.add(cube);
body.add(new THREE.AxesHelper(1.8));
scene.add(body);

const grid = new THREE.GridHelper(5, 10, 0x536158, 0x28322c);
grid.position.y = -1.1;
scene.add(grid);

let socket;
let latestConfig = collectConfig();

function connect() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/ws`);

  socket.addEventListener("open", () => {
    connection.textContent = "Connected";
    sendConfig();
  });

  socket.addEventListener("close", () => {
    connection.textContent = "Disconnected. Reconnecting...";
    setTimeout(connect, 1000);
  });

  socket.addEventListener("message", (event) => {
    const sample = JSON.parse(event.data);
    updateOrientation(sample.orientation);
    renderValues(sample.values);
    renderHealth(sample.health, sample.config?.rate_hz ?? latestConfig.rate_hz);
    connection.textContent = `Connected | ${sample.hz.toFixed(1)} Hz | I2C 0x${sample.address.toString(16)}`;
  });
}

function collectConfig() {
  const fields = {};
  for (const input of fieldInputs) {
    fields[input.dataset.field] = input.checked;
  }
  return {
    rate_hz: Number(rateInput.value),
    fields,
  };
}

function sendConfig() {
  latestConfig = collectConfig();
  rateValue.textContent = latestConfig.rate_hz;
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(latestConfig));
  }
}

function updateOrientation(orientation) {
  const quat = orientation?.quaternion;
  if (!quat || quat.length !== 4 || quat.some((value) => value === null)) {
    return;
  }

  const [qw, qx, qy, qz] = quat;
  body.quaternion.set(qx, qy, qz, qw).normalize();
}

function renderValues(values) {
  valuesEl.replaceChildren();
  for (const [key, value] of Object.entries(values ?? {})) {
    valuesEl.append(row(key, formatValue(value)));
  }
}

function renderHealth(health, targetRate) {
  healthEl.replaceChildren(
    healthRow("Quat norm", formatNumber(health.quat_norm, 4), health.quat_ok),
    healthRow("Gravity mag", `${formatNumber(health.gravity_mag, 2)} m/s^2`, health.gravity_ok),
    healthRow("Calibration", (health.calibration ?? []).join("/"), (health.calibration ?? []).every((v) => v === 3)),
    healthRow("Rate", `${formatNumber(health.rate_hz, 1)} / ${targetRate} Hz`, Math.abs(health.rate_hz - targetRate) <= targetRate * 0.2),
    row("Temperature", `${health.temperature_c ?? "?"} C`),
  );
}

function row(key, value) {
  const element = document.createElement("div");
  element.className = "row";
  element.innerHTML = `<span class="key"></span><span class="value"></span>`;
  element.children[0].textContent = key;
  element.children[1].textContent = value;
  return element;
}

function healthRow(key, value, ok) {
  const element = row(key, value);
  const badge = document.createElement("span");
  badge.className = ok ? "ok" : "warn";
  badge.textContent = ok ? "OK" : "WARN";
  element.append(badge);
  return element;
}

function formatValue(value) {
  if (Array.isArray(value)) {
    return value.map((item) => formatNumber(item, 4)).join("  ");
  }
  return String(value);
}

function formatNumber(value, digits) {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "nan";
}

function resize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
}

function animate() {
  resize();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

rateInput.addEventListener("input", sendConfig);
for (const input of fieldInputs) {
  input.addEventListener("change", sendConfig);
}

connect();
animate();
