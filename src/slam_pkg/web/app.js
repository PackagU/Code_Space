"use strict";

const token = document.querySelector('meta[name="packagu-token"]').content;
const $ = (id) => document.getElementById(id);
const state = { map: null, selected: null, drive: "stop", driveTimer: null, softwareStopped: false };

async function api(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    cache: "no-store",
    headers: { "Content-Type": "application/json", "X-Packagu-Token": token, ...(options.headers || {}) },
  });
  const payload = await response.json().catch(() => ({ error: `HTTP ${response.status}` }));
  if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload;
}

function notice(message, kind = "") {
  const el = $("notice");
  el.textContent = message;
  el.className = `notice ${kind}`.trim();
}

function setBadge(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = `badge ${kind}`;
}

function freshness(label, seconds, ready = true) {
  if (!Number.isFinite(seconds)) return [label + " 없음", "bad"];
  if (!ready || seconds > 0.5) return [`${label} ${seconds.toFixed(1)}s`, "warn"];
  return [`${label} 정상`, "good"];
}

async function refreshStatus() {
  try {
    const s = await api("/api/status");
    setBadge("connectionBadge", "Jetson 연결", "good");
    setBadge("modeBadge", s.mode_label, s.mode === "idle" ? "neutral" : "active");
    const scan = freshness("LiDAR", s.scan_age_sec); setBadge("scanBadge", ...scan);
    const odom = freshness("Odom", s.odom_age_sec); setBadge("odomBadge", ...odom);
    const drive = freshness("구동", s.drive_age_sec, s.drive_ready); setBadge("driveBadge", ...drive);
    state.softwareStopped = s.software_stop;
    $("emergencyStop").textContent = s.software_stop ? "정지 해제" : "소프트 정지";
    $("emergencyStop").classList.toggle("release", s.software_stop);
    $("mappingState").textContent = s.mapping_running ? "실행 중" : "정지";
    $("navigationState").textContent = s.navigation_running ? s.navigation_status : "정지";
    $("startMapping").disabled = s.mapping_running || s.navigation_running || !s.base_ready;
    $("stopMapping").disabled = !s.mapping_running;
    $("startNavigation").disabled = s.mapping_running || s.navigation_running || !$("mapSelect").value || !s.base_ready;
    $("stopNavigation").disabled = !s.navigation_running;
    if (s.message) notice(s.message, s.error ? "error" : "");
  } catch (error) {
    setBadge("connectionBadge", "연결 끊김", "bad");
    notice(`Jetson 화면 서버 응답 없음: ${error.message}`, "error");
    stopDrive();
  }
}

function decodedMap(map) {
  const raw = atob(map.data_b64);
  const values = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i += 1) values[i] = raw.charCodeAt(i);
  return values;
}

function mapTransform(canvas, map) {
  const pad = 24;
  const sx = (canvas.width - pad * 2) / map.width;
  const sy = (canvas.height - pad * 2) / map.height;
  const scale = Math.max(0.01, Math.min(sx, sy));
  return { scale, x: (canvas.width - map.width * scale) / 2, y: (canvas.height - map.height * scale) / 2 };
}

function worldToCanvas(map, t, x, y) {
  const gx = (x - map.origin_x) / map.resolution;
  const gy = (y - map.origin_y) / map.resolution;
  return [t.x + gx * t.scale, t.y + (map.height - gy) * t.scale];
}

function renderMap() {
  const canvas = $("mapCanvas");
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const nextW = Math.max(320, Math.round(rect.width * ratio));
  const nextH = Math.max(320, Math.round(rect.height * ratio));
  if (canvas.width !== nextW || canvas.height !== nextH) { canvas.width = nextW; canvas.height = nextH; }
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!state.map) return;

  const map = state.map;
  const pixels = decodedMap(map);
  const image = document.createElement("canvas");
  image.width = map.width; image.height = map.height;
  const imageCtx = image.getContext("2d");
  const imageData = imageCtx.createImageData(map.width, map.height);
  for (let gy = 0; gy < map.height; gy += 1) {
    for (let gx = 0; gx < map.width; gx += 1) {
      const value = pixels[gy * map.width + gx];
      const iy = map.height - 1 - gy;
      const idx = (iy * map.width + gx) * 4;
      let color = 92;
      if (value === 255) color = 72;
      else if (value >= 65) color = 18;
      else if (value <= 25) color = 232;
      imageData.data[idx] = color;
      imageData.data[idx + 1] = value === 255 ? color + 10 : color + 4;
      imageData.data[idx + 2] = value === 255 ? color + 23 : color + 8;
      imageData.data[idx + 3] = 255;
    }
  }
  imageCtx.putImageData(imageData, 0, 0);
  const t = mapTransform(canvas, map);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, t.x, t.y, map.width * t.scale, map.height * t.scale);
  ctx.strokeStyle = "rgba(61,228,210,.45)";
  ctx.strokeRect(t.x, t.y, map.width * t.scale, map.height * t.scale);

  if (map.robot_pose) drawPose(ctx, ...worldToCanvas(map, t, map.robot_pose.x, map.robot_pose.y), map.robot_pose.yaw, "#3de4d2", 11 * ratio);
  if (state.selected) drawPose(ctx, ...worldToCanvas(map, t, state.selected.x, state.selected.y), state.selected.yaw, state.selected.tool === "goal" ? "#ffb84d" : "#4a88ff", 10 * ratio);
}

function drawPose(ctx, x, y, yaw, color, size) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw);
  ctx.beginPath(); ctx.moveTo(size * 1.35, 0); ctx.lineTo(-size, size * .8); ctx.lineTo(-size * .55, 0); ctx.lineTo(-size, -size * .8); ctx.closePath();
  ctx.fillStyle = color; ctx.shadowColor = color; ctx.shadowBlur = 12; ctx.fill(); ctx.restore();
}

async function refreshMap() {
  try {
    const map = await api("/api/map");
    state.map = map.available ? map : null;
    $("mapEmpty").classList.toggle("hidden", Boolean(state.map));
    $("mapMeta").textContent = state.map ? `${map.width}×${map.height} · ${map.resolution.toFixed(3)} m/px · ${map.frame_id}` : "지도 없음";
    renderMap();
  } catch (_) { /* status poll reports connection failures */ }
}

async function refreshMaps() {
  try {
    const result = await api("/api/maps");
    const select = $("mapSelect");
    const current = select.value;
    select.innerHTML = '<option value="">저장 지도를 선택하세요</option>';
    result.maps.forEach((map) => {
      const option = document.createElement("option");
      option.value = map.path; option.textContent = `${map.floor} · ${map.name}`; select.appendChild(option);
    });
    if ([...select.options].some((option) => option.value === current)) select.value = current;
  } catch (_) { /* status poll reports connection failures */ }
}

async function post(path, body = {}) {
  const result = await api(path, { method: "POST", body: JSON.stringify(body) });
  notice(result.message || "요청을 처리했습니다.", "success");
  await refreshStatus();
  return result;
}

function driveCommand(direction) {
  if (state.drive === direction && direction !== "stop") return;
  state.drive = direction;
  document.querySelectorAll(".drive-key").forEach((el) => el.classList.toggle("active", el.dataset.drive === direction));
  api("/api/cmd", { method: "POST", body: JSON.stringify({ direction }) }).catch((error) => notice(error.message, "error"));
}

function startDrive(direction) {
  if (direction === "stop") { stopDrive(); return; }
  driveCommand(direction);
  clearInterval(state.driveTimer);
  state.driveTimer = setInterval(() => {
    state.drive = "__refresh__";
    driveCommand(direction);
  }, 150);
}

function stopDrive() {
  clearInterval(state.driveTimer); state.driveTimer = null;
  if (state.drive !== "stop") driveCommand("stop");
}

$("startMapping").addEventListener("click", () => post("/api/mapping/start", { floor: $("floorSelect").value }));
$("stopMapping").addEventListener("click", async () => {
  try {
    const result = await post("/api/mapping/stop", { floor: $("floorSelect").value, name: $("mapName").value.trim() });
    await refreshMaps();
    if (result.map) $("mapSelect").value = result.map;
  } catch (error) { notice(error.message, "error"); }
});
$("startNavigation").addEventListener("click", () => post("/api/navigation/start", { map: $("mapSelect").value }));
$("stopNavigation").addEventListener("click", () => post("/api/navigation/stop"));
$("emergencyStop").addEventListener("click", async () => {
  stopDrive();
  try { await post(state.softwareStopped ? "/api/stop/reset" : "/api/stop"); }
  catch (error) { notice(error.message, "error"); }
});

document.querySelectorAll(".drive-key").forEach((button) => {
  button.addEventListener("pointerdown", (event) => { event.preventDefault(); button.setPointerCapture(event.pointerId); startDrive(button.dataset.drive); });
  button.addEventListener("pointerup", stopDrive);
  button.addEventListener("pointercancel", stopDrive);
  button.addEventListener("lostpointercapture", stopDrive);
});

const keyMap = { w: "forward", a: "left", s: "backward", d: "right", " ": "stop" };
window.addEventListener("keydown", (event) => {
  if (event.target.matches("input,select,textarea")) return;
  const direction = keyMap[event.key.toLowerCase()];
  if (!direction) return;
  event.preventDefault();
  if (!event.repeat) startDrive(direction);
});
window.addEventListener("keyup", (event) => { if (keyMap[event.key.toLowerCase()]) { event.preventDefault(); stopDrive(); } });
window.addEventListener("blur", stopDrive);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });

$("mapCanvas").addEventListener("click", (event) => {
  if (!state.map) return;
  const canvas = $("mapCanvas");
  const rect = canvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * canvas.width / rect.width;
  const py = (event.clientY - rect.top) * canvas.height / rect.height;
  const t = mapTransform(canvas, state.map);
  const gx = (px - t.x) / t.scale;
  const gy = state.map.height - (py - t.y) / t.scale;
  if (gx < 0 || gy < 0 || gx >= state.map.width || gy >= state.map.height) return;
  const tool = document.querySelector('input[name="mapTool"]:checked').value;
  const yaw = Number($("yawInput").value) * Math.PI / 180;
  state.selected = { tool, x: state.map.origin_x + gx * state.map.resolution, y: state.map.origin_y + gy * state.map.resolution, yaw };
  $("targetText").textContent = `${tool === "goal" ? "목표" : "초기 위치"}: x ${state.selected.x.toFixed(2)}, y ${state.selected.y.toFixed(2)}, ${$("yawInput").value}°`;
  $("sendTarget").disabled = false;
  $("pointerMeta").textContent = "선택 지점을 확인한 뒤 적용하세요";
  renderMap();
});

$("sendTarget").addEventListener("click", async () => {
  if (!state.selected) return;
  try {
    const endpoint = state.selected.tool === "goal" ? "/api/navigation/goal" : "/api/navigation/initial-pose";
    await post(endpoint, state.selected);
  } catch (error) { notice(error.message, "error"); }
});

window.addEventListener("resize", renderMap);
setInterval(refreshStatus, 500);
setInterval(refreshMap, 600);
setInterval(refreshMaps, 3000);
refreshStatus(); refreshMap(); refreshMaps();
