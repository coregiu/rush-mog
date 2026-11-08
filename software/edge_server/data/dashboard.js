/* RUSH-MOG digital twin dashboard
 * 数据源：
 *   GET  /api/modules                       ESP32 模块注册表 + Edge 状态
 *   GET  /api/modules/{id}/health           ESP32 代理 ROS/Center 健康接口
 *   GET  /api/modules/{id}/params           ESP32 代理/本地参数描述
 *   POST /api/modules/{id}/params           参数修改
 * 页面只访问 192.168.4.1，避免手机跨域/AP 客户端隔离问题。
 */
'use strict';

const API = {
  modules: '/api/modules',
  edge: '/api/modules/edge',
  edgeHealth: '/api/modules/edge/health',
  edgeParams: '/api/modules/edge/params',
  body: '/api/body',
  module: (id) => `/api/modules/${encodeURIComponent(id)}`,
  health: (id) => `/api/modules/${encodeURIComponent(id)}/health`,
  params: (id) => `/api/modules/${encodeURIComponent(id)}/params`,
};

const MODULE_META = {
  edge: { name: 'EDGE SERVER', subtitle: 'ESP32-S3 · 车体网关 / Wi-Fi AP', glyph: 'E', order: 0 },
  ros: { name: 'ROS 2', subtitle: 'Raspberry Pi · 感知 / 导航 / 健康', glyph: 'R', order: 1 },
  center: { name: 'CENTER SERVER', subtitle: 'SmolVLA · 推理中心', glyph: 'C', order: 2 },
};

const DEVICE_META = {
  camera: { name: '摄像头', glyph: 'CAM' },
  lidar: { name: '雷达', glyph: 'LID' },
  imu: { name: 'IMU', glyph: 'IMU' },
  microphone: { name: '麦克风', glyph: 'MIC' },
  speaker: { name: '音箱', glyph: 'SPK' },
  arm: { name: '机械臂', glyph: 'ARM' },
  base: { name: '底盘 / STM32', glyph: 'BASE' },
  battery: { name: '电池', glyph: 'BAT' },
  odom: { name: '里程计', glyph: 'ODOM' },
};

const DEFAULT_DEVICES = ['camera', 'lidar', 'imu', 'microphone', 'speaker', 'arm', 'base', 'battery', 'odom'];

const BODY_MOTORS = [
  { id: 'fl', name: '左前 FL' },
  { id: 'fr', name: '右前 FR' },
  { id: 'rl', name: '左后 RL' },
  { id: 'rr', name: '右后 RR' },
];

const BODY_LIMITS = {
  maxSpeed: 1.4,
  maxWheel: 1.2,
  wheelRadius: 0.048,
  rotationRadius: 0.19,
  history: 60,
};

const STATUS_LABEL = {
  ok: '正常',
  degraded: '降级',
  error: '故障',
  starting: '启动中',
  unknown: '未知',
  disabled: '禁用',
  offline: '离线',
};

const STATUS_RANK = { ok: 0, disabled: 1, unknown: 2, starting: 3, degraded: 4, error: 5, offline: 6 };

const state = {
  registry: null,
  edge: null,
  modules: {},
  health: {},
  params: {},
  activeModule: null,
  activeTab: 'params',
  lastSync: 0,
  lastError: '',
  polling: false,
  pollTimer: null,
  clockTimer: null,
  bodyPolling: false,
  bodyPollTimer: null,
  bodyFailures: 0,
  bodyTelemetry: null,
  body: {
    lastOdom: null,
    sessionDistance: 0,
    history: [],
    historySync: 0,
    historyKey: '',
    latest: null,
  },
};

function $(selector, root = document) { return root.querySelector(selector); }
function $$(selector, root = document) { return Array.from(root.querySelectorAll(selector)); }

function escapeHtml(value) {
  return String(value == null ? '' : value).replace(/[&<>"']/g, (ch) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  }[ch]));
}

function numberOrNull(value) {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function fmtNumber(value, digits = 1) {
  const n = numberOrNull(value);
  if (n === null) return '--';
  if (Math.abs(n) >= 1000) return n.toFixed(0);
  return n.toFixed(digits).replace(/\.0+$/, '');
}

function fmtBytes(value) {
  const n = numberOrNull(value);
  if (n === null || n < 0) return '--';
  const units = ['B', 'KB', 'MB', 'GB'];
  let v = n;
  let i = 0;
  while (v >= 1024 && i < units.length - 1) { v /= 1024; i += 1; }
  return `${v >= 100 || i === 0 ? v.toFixed(0) : v.toFixed(1)} ${units[i]}`;
}

function fmtDuration(seconds) {
  const n = numberOrNull(seconds);
  if (n === null || n < 0) return '--';
  if (n < 60) return `${Math.floor(n)}s`;
  const m = Math.floor(n / 60);
  const s = Math.floor(n % 60);
  if (m < 60) return `${m}m ${String(s).padStart(2, '0')}s`;
  const h = Math.floor(m / 60);
  return `${h}h ${String(m % 60).padStart(2, '0')}m`;
}

function fmtAge(ms) {
  const n = numberOrNull(ms);
  if (n === null || n < 0) return '--';
  if (n < 1000) return `${Math.floor(n)}ms`;
  return `${Math.floor(n / 1000)}s`;
}

function fmtTime(date = new Date()) {
  return date.toLocaleTimeString('zh-CN', { hour12: false });
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function wrapPi(rad) {
  return Math.atan2(Math.sin(rad), Math.cos(rad));
}

function normalizeDegrees(deg) {
  const value = deg % 360;
  return value < 0 ? value + 360 : value;
}

function fmtDistance(meters) {
  const n = numberOrNull(meters);
  if (n === null) return { value: '--', unit: 'm' };
  if (n >= 1000) return { value: (n / 1000).toFixed(2), unit: 'km' };
  if (n >= 100) return { value: n.toFixed(1), unit: 'm' };
  return { value: n.toFixed(2), unit: 'm' };
}

function statusLabel(status) { return STATUS_LABEL[status] || status || '未知'; }
function statusRank(status) { return STATUS_RANK[status] == null ? STATUS_RANK.unknown : STATUS_RANK[status]; }

function worstStatus(statuses) {
  let worst = 'ok';
  for (const status of statuses) {
    if (statusRank(status) > statusRank(worst)) worst = status;
  }
  return worst;
}

async function fetchJson(url, options = {}, timeoutMs = 4500) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(url, { ...options, cache: 'no-store', signal: controller.signal });
    const text = await response.text();
    let data = null;
    if (text) {
      try { data = JSON.parse(text); } catch { data = { raw: text }; }
    }
    if (!response.ok) {
      const message = data && data.error && data.error.message
        ? data.error.message
        : `HTTP ${response.status}`;
      const error = new Error(message);
      error.status = response.status;
      error.data = data;
      throw error;
    }
    return data;
  } finally {
    clearTimeout(timer);
  }
}

function moduleHealth(id) {
  if (id === 'edge') return state.edge;
  return state.health[id] || null;
}

function moduleRegistry(id) {
  if (id === 'edge') return state.edge;
  return state.modules[id] || null;
}

function moduleInfo(id) {
  const health = moduleHealth(id);
  const registry = moduleRegistry(id);
  if (!health && !registry) return null;
  const status = (health && health.status) || (registry && registry.status) || 'unknown';
  const stale = Boolean(registry && registry.stale && !health);
  return {
    id,
    status: stale ? 'unknown' : status,
    rawStatus: status,
    stale,
    name: (registry && registry.name) || MODULE_META[id].name,
    version: (health && health.version) || (registry && registry.version) || '',
    ip: (registry && registry.ip) || '',
    summary: (health && health.reason) || (registry && registry.summary) || '',
    uptimeSec: health ? health.uptime_sec : null,
    health,
    registry,
  };
}

function moduleIds() { return ['edge', 'ros', 'center']; }

function overallStatus() {
  const statuses = moduleIds().map((id) => {
    const info = moduleInfo(id);
    return info ? info.status : 'unknown';
  });
  if (statuses.includes('error')) return 'error';
  if (statuses.includes('degraded')) return 'degraded';
  if (statuses.includes('starting')) return 'starting';
  if (statuses.includes('unknown') || statuses.includes('offline')) return 'unknown';
  if (statuses.every((s) => s === 'disabled')) return 'disabled';
  return 'ok';
}

async function loadRegistry() {
  const data = await fetchJson(API.modules);
  state.registry = data;
  state.edge = data.edge || null;
  state.modules = {};
  for (const module of data.modules || []) {
    if (module && module.id) state.modules[module.id] = module;
  }
  state.lastError = '';
}

async function fallbackRegistry() {
  const status = await fetchJson('/status');
  state.edge = {
    id: 'edge',
    name: 'Edge Server',
    version: 'legacy',
    ip: location.hostname,
    status: status.camera ? 'ok' : 'degraded',
    summary: `camera=${status.camera ? 'on' : 'off'}, sdcard=${status.sdcard ? 'on' : 'off'}`,
    stale: false,
    params: status,
  };
  state.modules = {};
  state.registry = { ok: true, edge: state.edge, modules: [], legacy: true };
}

async function fetchModuleJson(id, kind) {
  const primary = kind === 'health' ? API.health(id) : API.params(id);
  try {
    return await fetchJson(primary);
  } catch (primaryError) {
    const registry = state.modules[id];
    const direct = registry && (kind === 'health' ? registry.health_url : registry.params_url);
    if (direct) {
      try {
        return await fetchJson(direct);
      } catch (directError) {
        primaryError.directError = directError;
      }
    }
    throw primaryError;
  }
}

async function poll() {
  if (state.polling) return;
  state.polling = true;
  setSyncText('同步中…');
  try {
    try {
      await loadRegistry();
    } catch (error) {
      if (error.status === 404) {
        await fallbackRegistry();
      } else {
        throw error;
      }
    }

    await Promise.all(['ros', 'center'].map(async (id) => {
      if (!state.modules[id]) {
        state.health[id] = null;
        return;
      }
      try {
        state.health[id] = await fetchModuleJson(id, 'health');
      } catch (error) {
        state.health[id] = null;
        state.lastError = `${id}: ${error.message}`;
      }
    }));

    state.lastSync = Date.now();
    renderAll();
  } catch (error) {
    state.lastError = error.message || String(error);
    renderConnectionError();
  } finally {
    state.polling = false;
    updateSyncText();
  }
}

function setSyncText(text) {
  const el = $('#sync-state');
  if (el) el.textContent = text;
}

function updateSyncText() {
  const el = $('#sync-state');
  if (!el) return;
  if (!state.lastSync) {
    el.textContent = state.lastError ? `连接失败：${state.lastError}` : '等待数据';
    return;
  }
  const age = Math.max(0, Date.now() - state.lastSync);
  const count = Object.keys(state.modules).length + (state.edge ? 1 : 0);
  el.textContent = `${count} 个模块 · ${fmtAge(age)}前同步${state.lastError ? ' · ' + state.lastError : ''}`;
}

function mecanumWheelSpeeds(vx, vy, wz) {
  const k = BODY_LIMITS.rotationRadius * wz;
  return [vx - vy - k, vx + vy + k, vx + vy - k, vx - vy + k];
}

function mecanumTwistFromWheels(speeds) {
  const fl = speeds[0];
  const fr = speeds[1];
  const rl = speeds[2];
  const rr = speeds[3];
  return {
    vx: (fl + fr + rl + rr) / 4,
    vy: (-fl + fr + rl - rr) / 4,
    wz: (-fl + fr - rl + rr) / (4 * BODY_LIMITS.rotationRadius),
  };
}

function bodyMotionLabel(data) {
  if (!data.hasOdom || data.speed === null) return '待上报';
  const vx = data.vx == null ? 0 : data.vx;
  const vy = data.vy == null ? 0 : data.vy;
  const wz = data.wz == null ? 0 : data.wz;
  if (Math.abs(wz) > 0.2 && Math.abs(vx) < 0.05 && Math.abs(vy) < 0.05) return '原地转向';
  if (Math.abs(vy) > 0.05 && Math.abs(vx) < 0.05) return '横移';
  if (vx > 0.03) return '前进';
  if (vx < -0.03) return '后退';
  if (data.speed > 0.03) return '移动';
  return '停止';
}

function collectBodyTelemetry(now = Date.now()) {
  const ros = state.health.ros || {};
  const devices = Array.isArray(ros.devices) ? ros.devices : [];
  const byId = {};
  for (const device of devices) {
    if (device && device.id) byId[device.id] = device;
  }
  const odom = byId.odom || null;
  const base = byId.base || null;
  const battery = byId.battery || null;
  const om = (odom && odom.metrics) || {};
  const bm = (base && base.metrics) || {};
  const bat = (battery && battery.metrics) || {};

  // ESP32 缓存了 STM32 编码器遥测；优先用较新的那份（1s 快速轮询或完整轮询）。
  const fastBody = state.bodyTelemetry;
  const edgeBody = state.edge && state.edge.body ? state.edge.body : null;
  let espBody = fastBody;
  if (edgeBody) {
    const fastAge = fastBody ? numberOrNull(fastBody.age_ms) : null;
    const edgeAge = numberOrNull(edgeBody.age_ms);
    if (!fastBody || (edgeAge !== null && (fastAge === null || edgeAge < fastAge))) {
      espBody = edgeBody;
    }
  }
  const bodyWheels = espBody && Array.isArray(espBody.wheel_mm_s) ? espBody.wheel_mm_s : null;
  const bodyLive = Boolean(
    espBody
    && espBody.valid === true
    && espBody.stale !== true
    && bodyWheels
    && bodyWheels.length >= 4
    && bodyWheels.slice(0, 4).every((value) => numberOrNull(value) !== null)
  );
  const bodyWheelSpeeds = bodyLive ? bodyWheels.slice(0, 4).map((value) => Number(value) / 1000) : null;
  const bodyDistanceMm = espBody ? numberOrNull(espBody.distance_mm) : null;

  const x = numberOrNull(om.x);
  const y = numberOrNull(om.y);
  const yaw = numberOrNull(om.yaw);
  let vx = numberOrNull(om.vx);
  let vy = numberOrNull(om.vy);
  let wz = numberOrNull(om.wz);
  let twistSource = 'none';
  if (vx !== null && vy !== null && wz !== null) twistSource = 'telemetry';

  const sampleTime = state.lastSync || now;
  let derivedSpeed = null;
  let derivedVx = null;
  let derivedVy = null;
  let derivedWz = null;

  if (x !== null && y !== null) {
    const last = state.body.lastOdom;
    if (last && sampleTime > last.t) {
      const dt = (sampleTime - last.t) / 1000;
      if (dt >= 0.2 && dt <= 120) {
        const dx = x - last.x;
        const dy = y - last.y;
        const step = Math.hypot(dx, dy);
        if (step < 5) {
          if (numberOrNull(om.distance) === null) {
            state.body.sessionDistance += step;
          }
          const midYaw = yaw === null ? (last.yaw || 0) : yaw;
          const c = Math.cos(midYaw);
          const s = Math.sin(midYaw);
          derivedVx = (dx * c + dy * s) / dt;
          derivedVy = (-dx * s + dy * c) / dt;
          derivedWz = yaw !== null && last.yaw !== null ? wrapPi(yaw - last.yaw) / dt : 0;
          derivedSpeed = Math.hypot(derivedVx, derivedVy);
        }
      }
    }
    state.body.lastOdom = { x, y, yaw, t: sampleTime };
  }

  const telemetrySpeed = numberOrNull(om.speed);
  let speed = telemetrySpeed;
  if (speed === null && vx !== null && vy !== null) speed = Math.hypot(vx, vy);
  if (speed === null) speed = derivedSpeed;
  if (twistSource === 'none' && derivedVx !== null) {
    vx = derivedVx;
    vy = derivedVy;
    wz = derivedWz;
    twistSource = 'derived';
  }

  if (bodyLive) {
    const bodyTwist = mecanumTwistFromWheels(bodyWheelSpeeds);
    vx = bodyTwist.vx;
    vy = bodyTwist.vy;
    wz = bodyTwist.wz;
    speed = Math.hypot(vx, vy);
    twistSource = 'stm32';
  }

  const telemetryDistance = numberOrNull(om.distance);
  let distance = telemetryDistance === null ? state.body.sessionDistance : telemetryDistance;
  let distanceSource = telemetryDistance === null ? 'session' : 'ros';
  if (bodyLive && bodyDistanceMm !== null && bodyDistanceMm >= 0) {
    distance = bodyDistanceMm / 1000;
    distanceSource = 'stm32';
  }

  let wheelSpeeds = null;
  let wheelSource = 'none';
  if (bodyLive) {
    wheelSpeeds = bodyWheelSpeeds;
    wheelSource = 'stm32';
  } else {
    const baseWheels = Array.isArray(bm.wheel_speeds) ? bm.wheel_speeds : null;
    if (baseWheels && baseWheels.length >= 4 && baseWheels.slice(0, 4).every((value) => numberOrNull(value) !== null)) {
      wheelSpeeds = baseWheels.slice(0, 4).map(Number);
      wheelSource = 'base';
    } else if (vx !== null && vy !== null && wz !== null) {
      wheelSpeeds = mecanumWheelSpeeds(vx, vy, wz);
      wheelSource = 'odom';
    }
  }

  const voltage = numberOrNull(bat.voltage) !== null
    ? numberOrNull(bat.voltage)
    : (numberOrNull(bm.battery_mv) !== null ? numberOrNull(bm.battery_mv) / 1000 : null);
  let percent = numberOrNull(bat.percentage);
  if (percent !== null && percent < 0) percent = null;
  const current = numberOrNull(bm.current_ma) === null ? null : numberOrNull(bm.current_ma) / 1000;
  const encoderOk = typeof bm.encoder_ok === 'boolean' ? bm.encoder_ok : null;
  const imuOk = typeof bm.imu_ok === 'boolean' ? bm.imu_ok : null;
  const estop = typeof bm.estop === 'boolean'
    ? bm.estop
    : (ros.emergency_stop == null ? null : Boolean(ros.emergency_stop));

  const hasOdom = Boolean(bodyLive || (odom && (x !== null || y !== null || speed !== null)));
  return {
    odom,
    base,
    battery,
    body: espBody,
    bodyLive,
    bodyDistanceMm,
    status: bodyLive ? 'ok' : (odom ? (odom.status || 'unknown') : 'unknown'),
    hasOdom,
    speed,
    vx,
    vy,
    wz,
    twistSource,
    distance,
    distanceSource,
    wheelSpeeds,
    wheelSource,
    yaw,
    rate: bodyLive ? (numberOrNull(espBody.rate_hz) || 10) : (odom ? numberOrNull(odom.rate_hz) : null),
    voltage,
    percent,
    current,
    encoderOk,
    imuOk,
    estop,
    baseOnline: Boolean(base),
    sampleTime,
  };
}

function initGauge() {
  const ticks = $('#body-gauge-ticks');
  if (!ticks || ticks.childElementCount) return;
  const ns = 'http://www.w3.org/2000/svg';
  for (let i = 0; i <= 6; i += 1) {
    const angle = Math.PI - (i / 6) * Math.PI;
    const inner = i % 2 === 0 ? 84 : 88;
    const line = document.createElementNS(ns, 'line');
    line.setAttribute('x1', (120 + Math.cos(angle) * 92).toFixed(2));
    line.setAttribute('y1', (112 - Math.sin(angle) * 92).toFixed(2));
    line.setAttribute('x2', (120 + Math.cos(angle) * inner).toFixed(2));
    line.setAttribute('y2', (112 - Math.sin(angle) * inner).toFixed(2));
    ticks.appendChild(line);
  }
}

function initMotorCards() {
  const grid = $('#motor-grid');
  if (!grid || grid.childElementCount) return;
  grid.innerHTML = BODY_MOTORS.map((motor) => `
    <article class="motor-card" data-motor="${motor.id}" data-dir="idle">
      <div class="motor-top">
        <span class="motor-name">${escapeHtml(motor.name)}</span>
        <span class="motor-dir" data-motor-dir>--</span>
      </div>
      <div class="motor-body">
        <svg class="motor-wheel" viewBox="0 0 48 48" aria-hidden="true">
          <circle class="mw-tire" cx="24" cy="24" r="20"/>
          <g class="mw-spin">
            <circle class="mw-rim" cx="24" cy="24" r="11"/>
            <g class="mw-spokes"><path d="M24 7 V41 M7 24 H41 M12 12 L36 36 M12 36 L36 12"/></g>
            <circle class="mw-hub" cx="24" cy="24" r="3"/>
          </g>
        </svg>
        <div class="motor-read">
          <div><strong data-motor-rpm>--</strong><span>RPM</span></div>
          <div><strong data-motor-ms>--</strong><span>m/s</span></div>
        </div>
      </div>
      <div class="motor-bar"><i data-motor-bar></i></div>
    </article>`).join('');
}

function setFlag(element, stateClass, label) {
  if (!element) return;
  element.classList.remove('is-on', 'is-ok', 'is-fault');
  if (stateClass) element.classList.add(stateClass);
  if (label) element.textContent = label;
}

function renderSpeedSpark(data) {
  const line = $('#speed-spark-line');
  const area = $('#speed-spark-area');
  const note = $('#body-hint');
  if (note) {
    if (data.bodyLive) note.textContent = '轮速/里程来自 STM32 编码器 · 经 ESP32 上报';
    else if (!data.hasOdom) note.textContent = '等待 STM32 编码器或 ROS /odom 上报';
    else if (data.wheelSource === 'base') note.textContent = '轮速来自 /base/status · 速度/里程来自 /odom';
    else if (data.twistSource === 'telemetry') note.textContent = '速度/轮速来自 /odom 实时 twist · 里程累计自 ROS';
    else note.textContent = '速度/里程由 /odom 位姿推算';
  }
  if (!line || !area) return;
  const values = state.body.history;
  if (values.length < 2) {
    line.setAttribute('points', '');
    area.setAttribute('d', '');
    return;
  }
  const width = 600;
  const height = 56;
  const pad = 4;
  const max = Math.max(0.4, ...values) * 1.12;
  const points = values.map((value, index) => {
    const x = (index / (values.length - 1)) * width;
    const y = height - pad - clamp(value / max, 0, 1) * (height - pad * 2);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  });
  line.setAttribute('points', points.join(' '));
  area.setAttribute('d', `M 0 ${height} L ${points.join(' L ')} L ${width} ${height} Z`);
}

function renderBody() {
  initGauge();
  initMotorCards();
  const data = collectBodyTelemetry();
  state.body.latest = data;

  const panel = $('#body-panel');
  if (panel) panel.classList.toggle('is-empty', !data.hasOdom);

  const led = $('#body-led');
  if (led) led.className = `led status-${data.hasOdom ? data.status : 'unknown'}`;

  const source = $('#body-source');
  if (source) {
    if (data.bodyLive) {
      source.textContent = 'STM32 实时';
    } else if (data.hasOdom) {
      source.textContent = data.twistSource === 'telemetry' ? 'ODOM 实时' : 'ODOM 推算';
    } else {
      source.textContent = '等待 /odom';
    }
    source.className = `status-chip status-${data.hasOdom ? data.status : 'unknown'}`;
  }

  const time = $('#body-time');
  if (time) {
    const bodyAge = data.body ? numberOrNull(data.body.age_ms) : null;
    if (data.bodyLive && bodyAge !== null) {
      time.textContent = fmtTime(new Date(Date.now() - bodyAge));
    } else {
      time.textContent = state.lastSync ? fmtTime(new Date(state.lastSync)) : '--:--:--';
    }
  }

  const speed = data.hasOdom ? data.speed : null;
  const speedEl = $('#body-speed');
  if (speedEl) speedEl.textContent = speed === null ? '--' : speed.toFixed(2);
  const kmhEl = $('#body-speed-kmh');
  if (kmhEl) kmhEl.textContent = speed === null ? '--' : (speed * 3.6).toFixed(2);
  const motionEl = $('#body-motion');
  if (motionEl) motionEl.textContent = bodyMotionLabel(data);

  const pct = speed === null ? 0 : clamp(speed / BODY_LIMITS.maxSpeed, 0, 1);
  const fill = $('#body-gauge-fill');
  if (fill) fill.setAttribute('stroke-dasharray', `${(pct * 100).toFixed(1)} 100`);
  const needle = $('#body-needle');
  if (needle) needle.setAttribute('transform', `rotate(${(pct * 180).toFixed(1)} 120 112)`);

  const rateLabel = $('#body-rate-label');
  if (rateLabel) rateLabel.textContent = data.bodyLive ? 'BODY' : 'ODOM';
  const rateEl = $('#body-odom-rate');
  if (rateEl) rateEl.textContent = data.rate === null ? '--' : `${fmtNumber(data.rate)}Hz`;

  const odo = data.hasOdom ? fmtDistance(data.distance) : { value: '--', unit: 'm' };
  const odoEl = $('#body-odo');
  if (odoEl) odoEl.textContent = odo.value;
  const odoUnit = $('#body-odo-unit');
  if (odoUnit) odoUnit.textContent = odo.unit;
  const odoNote = $('#body-odo-note');
  if (odoNote) {
    if (!data.hasOdom) {
      odoNote.textContent = '等待里程计';
    } else if (data.distanceSource === 'stm32') {
      odoNote.textContent = 'STM32 编码器累计';
    } else if (data.distanceSource === 'ros') {
      odoNote.textContent = 'ROS 启动后累计';
    } else {
      odoNote.textContent = '本次监控累计';
    }
  }

  const headingEl = $('#body-heading');
  if (headingEl) {
    headingEl.textContent = data.yaw === null ? '--' : `${normalizeDegrees(data.yaw * 180 / Math.PI).toFixed(1)}°`;
  }

  const batteryEl = $('#body-battery');
  if (batteryEl) {
    if (data.voltage === null && data.percent === null) {
      batteryEl.textContent = '--';
    } else {
      const volts = data.voltage === null ? '--' : `${data.voltage.toFixed(2)}V`;
      const percentText = data.percent === null ? '--' : `${data.percent.toFixed(0)}%`;
      batteryEl.textContent = `${volts} · ${percentText}`;
    }
  }
  const currentEl = $('#body-current');
  if (currentEl) currentEl.textContent = data.current === null ? '--' : `${data.current.toFixed(2)}A`;
  const encoderEl = $('#body-encoder');
  if (encoderEl) {
    if (data.encoderOk !== null) encoderEl.textContent = data.encoderOk ? 'OK' : 'FAULT';
    else encoderEl.textContent = data.bodyLive ? 'OK' : '--';
  }

  setFlag($('#body-estop'), data.estop === null ? '' : (data.estop ? 'is-on' : 'is-ok'), 'E-STOP');
  setFlag($('#body-imu'), data.imuOk === null ? '' : (data.imuOk ? 'is-ok' : 'is-fault'), 'IMU');
  setFlag($('#body-link'), data.baseOnline ? 'is-ok' : '', '/BASE');

  const wheelSpeeds = data.hasOdom ? data.wheelSpeeds : null;
  BODY_MOTORS.forEach((motor, index) => {
    const card = $(`.motor-card[data-motor="${motor.id}"]`);
    if (!card) return;
    const rpmEl = $('[data-motor-rpm]', card);
    const msEl = $('[data-motor-ms]', card);
    const dirEl = $('[data-motor-dir]', card);
    const barEl = $('[data-motor-bar]', card);
    const raw = wheelSpeeds && wheelSpeeds[index] !== undefined ? numberOrNull(wheelSpeeds[index]) : null;
    if (raw === null) {
      card.dataset.dir = 'idle';
      card.classList.remove('is-spinning');
      if (rpmEl) rpmEl.textContent = '--';
      if (msEl) msEl.textContent = '--';
      if (dirEl) dirEl.textContent = '--';
      if (barEl) barEl.style.width = '0%';
      return;
    }
    const rpm = Math.abs(raw) / (2 * Math.PI * BODY_LIMITS.wheelRadius) * 60;
    const direction = raw > 0.02 ? 'fwd' : (raw < -0.02 ? 'rev' : 'idle');
    card.dataset.dir = direction;
    if (rpmEl) rpmEl.textContent = rpm >= 100 ? rpm.toFixed(0) : rpm.toFixed(1);
    if (msEl) msEl.textContent = raw.toFixed(2);
    if (dirEl) dirEl.textContent = direction === 'fwd' ? '正转' : (direction === 'rev' ? '反转' : '停');
    if (barEl) barEl.style.width = `${clamp(Math.abs(raw) / BODY_LIMITS.maxWheel * 100, 0, 100).toFixed(1)}%`;
    const spinning = Math.abs(raw) > 0.02;
    card.classList.toggle('is-spinning', spinning);
    if (spinning) {
      const period = clamp(0.3 / Math.abs(raw), 0.25, 2.4);
      card.style.setProperty('--spin', `${period.toFixed(2)}s`);
    }
  });

  const historyKey = data.bodyLive && data.body
    ? `b:${data.body.seq}`
    : (state.lastSync ? `o:${state.lastSync}` : '');
  if (historyKey && state.body.historyKey !== historyKey) {
    state.body.historyKey = historyKey;
    if (speed !== null) {
      state.body.history.push(speed);
      if (state.body.history.length > BODY_LIMITS.history) state.body.history.shift();
    }
  }
  renderSpeedSpark(data);
}

function renderAll() {
  renderOverall();
  renderModules();
  renderBody();
  renderTwin();
  renderDevices();
  renderTelemetry();
  renderSyncTime();
}

function renderConnectionError() {
  const pill = $('#overall-pill');
  if (pill) {
    pill.className = 'status-pill status-error';
    $('#overall-text').textContent = '边缘 API 不可达';
  }
  const stage = $('#twin-stage');
  if (stage) stage.dataset.overall = 'error';
  const bodySource = $('#body-source');
  if (bodySource) {
    bodySource.textContent = 'EDGE API 不可达';
    bodySource.className = 'status-chip status-error';
  }
  const bodyLed = $('#body-led');
  if (bodyLed) bodyLed.className = 'led status-error';
  const footer = $('#footer-status');
  if (footer) footer.textContent = 'LINK LOST';
  updateSyncText();
}

function renderOverall() {
  const status = overallStatus();
  const pill = $('#overall-pill');
  const text = $('#overall-text');
  if (pill) pill.className = `status-pill status-${status}`;
  if (text) {
    if (status === 'ok') text.textContent = '系统正常';
    else if (status === 'degraded') text.textContent = '系统降级';
    else if (status === 'error') text.textContent = '系统故障';
    else if (status === 'starting') text.textContent = '系统启动中';
    else if (status === 'disabled') text.textContent = '监控关闭';
    else text.textContent = '部分模块未连接';
  }
  const stage = $('#twin-stage');
  if (stage) stage.dataset.overall = status;
  const led = $('#twin-led');
  if (led) led.className = `led status-${status}`;
  const footer = $('#footer-status');
  if (footer) footer.textContent = `LINK ${status.toUpperCase()}`;
}

function renderSyncTime() {
  const el = $('#twin-time');
  if (el) el.textContent = state.lastSync ? fmtTime(new Date(state.lastSync)) : '--:--:--';
  updateSyncText();
}

function moduleMetrics(id, info) {
  const chips = [];
  if (id === 'edge') {
    const p = (state.edge && state.edge.params) || {};
    chips.push(`HEAP ${fmtBytes(p.free_heap)}`);
    chips.push(`PSRAM ${fmtBytes(p.free_psram)}`);
    chips.push(`CLIENTS ${p.wifi_clients == null ? '--' : p.wifi_clients}`);
    chips.push(`CAM ${p.camera ? 'ON' : 'OFF'}`);
    chips.push(`SD ${p.sdcard ? 'ON' : 'OFF'}`);
  } else if (id === 'ros') {
    const health = info.health;
    const summary = (health && health.summary) || {};
    chips.push(`DEV ${summary.ok || 0}/${summary.total || 0} OK`);
    const ros = (health && health.ros) || {};
    chips.push(`NODES ${ros.node_count == null ? '--' : ros.node_count}`);
    if (ros.missing_nodes && ros.missing_nodes.length) chips.push(`MISS ${ros.missing_nodes.length}`);
    if (health && health.emergency_stop) chips.push('E-STOP');
    if (health && health.ready != null) chips.push(health.ready ? 'READY' : 'NOT READY');
  } else if (id === 'center') {
    const health = info.health;
    const model = (health && health.model) || {};
    chips.push(`MODEL ${model.loaded ? 'LOADED' : 'IDLE'}`);
    if (model.backend) chips.push(String(model.backend).toUpperCase());
    if (model.device) chips.push(String(model.device).toUpperCase());
    if (health && health.ready != null) chips.push(health.ready ? 'READY' : 'NOT READY');
  }
  return chips;
}

function renderModules() {
  const list = $('#module-list');
  if (!list) return;
  let count = 0;
  const cards = moduleIds().map((id) => {
    const meta = MODULE_META[id];
    const info = moduleInfo(id);
    if (!info) {
      return `
        <article class="module-card" data-module="${id}" data-status="unknown">
          <div class="module-top">
            <div class="module-icon">${escapeHtml(meta.glyph)}</div>
            <div class="module-title">
              <h3>${escapeHtml(meta.name)}</h3>
              <p>${escapeHtml(meta.subtitle)}</p>
            </div>
            <span class="status-chip status-unknown">${statusLabel('unknown')}</span>
          </div>
          <p class="module-summary">等待模块连接 RUSH-MOG Wi-Fi 并向 192.168.4.1 上报。</p>
          <div class="module-actions">
            <button class="btn btn-ghost" type="button" data-action="params" data-module="${id}" disabled>参数</button>
            <button class="btn btn-ghost" type="button" data-action="raw" data-module="${id}" disabled>详情</button>
          </div>
        </article>`;
    }
    count += 1;
    const chips = moduleMetrics(id, info).slice(0, 5).map((chip) => `<span class="metric-chip">${escapeHtml(chip)}</span>`).join('');
    const stale = info.stale ? '<span class="module-stale">缓存</span>' : '';
    const version = info.version ? `v${escapeHtml(info.version)}` : '';
    const ip = info.ip ? escapeHtml(info.ip) : '';
    return `
      <article class="module-card" data-module="${id}" data-status="${escapeHtml(info.status)}">
        <div class="module-top">
          <div class="module-icon">${escapeHtml(meta.glyph)}</div>
          <div class="module-title">
            <h3>${escapeHtml(meta.name)} ${stale}</h3>
            <p>${escapeHtml(meta.subtitle)}${version ? ' · ' + version : ''}${ip ? ' · ' + ip : ''}</p>
          </div>
          <span class="status-chip status-${escapeHtml(info.status)}">${escapeHtml(statusLabel(info.status))}</span>
        </div>
        <p class="module-summary">${escapeHtml(info.summary || '无摘要')}</p>
        <div class="module-metrics">${chips}</div>
        <div class="module-actions">
          <button class="btn" type="button" data-action="params" data-module="${id}">参数</button>
          <button class="btn btn-ghost" type="button" data-action="raw" data-module="${id}">详情</button>
        </div>
      </article>`;
  }).join('');

  list.innerHTML = cards || '<div class="empty-state">暂无模块上报</div>';
  const countEl = $('#module-count');
  if (countEl) countEl.textContent = `${count} / 3`;
}

function deviceStatusMap() {
  const map = {};
  for (const id of DEFAULT_DEVICES) map[id] = 'unknown';
  const health = state.health.ros;
  for (const device of (health && health.devices) || []) {
    if (device && device.id) map[device.id] = device.status || 'unknown';
  }
  if (state.edge && map.camera === 'unknown') {
    map.camera = state.edge.params && state.edge.params.camera ? 'ok' : 'degraded';
  }
  return map;
}

function renderTwin() {
  const statuses = {};
  for (const id of moduleIds()) {
    const info = moduleInfo(id);
    statuses[id] = info ? info.status : 'unknown';
  }
  for (const node of $$('.module-node')) {
    const id = node.dataset.module;
    node.dataset.status = statuses[id] || 'unknown';
  }
  for (const link of $$('.data-link')) {
    link.dataset.status = statuses[link.dataset.link] || 'unknown';
  }

  const devices = deviceStatusMap();
  const health = state.health.ros;
  const byId = {};
  for (const device of (health && health.devices) || []) byId[device.id] = device;

  for (const hotspot of $$('.hotspot')) {
    const id = hotspot.dataset.device;
    const status = devices[id] || 'unknown';
    hotspot.dataset.status = status;
    const device = byId[id];
    const detail = device ? `${device.detail || ''}${device.rate_hz >= 0 ? ` · ${fmtNumber(device.rate_hz)}Hz` : ''}` : statusLabel(status);
    const title = hotspot.querySelector('title');
    if (title) title.textContent = `${id} · ${statusLabel(status)} · ${detail}`;
  }

  const vehicle = $('.twin-vehicle');
  if (vehicle) {
    const body = state.body.latest;
    const speed = body && body.hasOdom && body.speed !== null ? Math.abs(body.speed) : 0;
    const moving = speed > 0.02;
    vehicle.classList.toggle('is-moving', moving);
    if (moving) {
      vehicle.style.setProperty('--wheel-spin', `${clamp(0.3 / speed, 0.25, 2.4).toFixed(2)}s`);
    }
  }

  const time = $('#twin-time');
  if (time) time.textContent = state.lastSync ? fmtTime(new Date(state.lastSync)) : '--:--:--';
}

function renderDevices() {
  const grid = $('#device-grid');
  if (!grid) return;
  const health = state.health.ros;
  const byId = {};
  for (const device of (health && health.devices) || []) byId[device.id] = device;
  const cards = DEFAULT_DEVICES.map((id) => {
    const meta = DEVICE_META[id];
    const device = byId[id];
    const status = device ? device.status : (id === 'camera' && state.edge && state.edge.params ? (state.edge.params.camera ? 'ok' : 'degraded') : 'unknown');
    const detail = device ? device.detail : '等待 ROS 上报';
    const chips = [];
    if (device) {
      if (device.rate_hz != null && device.rate_hz >= 0) chips.push(`${fmtNumber(device.rate_hz)}Hz`);
      if (device.age_sec != null && device.age_sec >= 0) chips.push(`AGE ${fmtNumber(device.age_sec)}s`);
      const metrics = device.metrics || {};
      if (metrics.width && metrics.height) chips.push(`${metrics.width}x${metrics.height}`);
      if (metrics.percentage != null && metrics.percentage >= 0) chips.push(`${fmtNumber(metrics.percentage)}%`);
      if (metrics.joint_count != null) chips.push(`${metrics.joint_count} joints`);
      if (metrics.ranges != null) chips.push(`${metrics.ranges} rays`);
      if (metrics.sample_rate) chips.push(`${metrics.sample_rate}Hz`);
    }
    return `
      <article class="device-card" data-status="${escapeHtml(status)}" data-device-id="${id}">
        <div class="device-head">
          <div class="device-glyph">${escapeHtml(meta.glyph)}</div>
          <div class="device-name">
            <strong>${escapeHtml(meta.name)}</strong>
            <span>${escapeHtml(device ? (device.type || id) : id)}</span>
          </div>
          <span class="status-chip status-${escapeHtml(status)}">${escapeHtml(statusLabel(status))}</span>
        </div>
        <p class="device-detail">${escapeHtml(detail || statusLabel(status))}</p>
        <div class="device-metrics">${chips.map((chip) => `<span class="metric-chip">${escapeHtml(chip)}</span>`).join('')}</div>
      </article>`;
  }).join('');
  grid.innerHTML = cards;
  const count = $('#device-count');
  if (count) count.textContent = `${Object.keys(byId).length} 个设备`;
}

function telemetryCard(label, value, sub) {
  return `<div class="telemetry">
    <span class="t-label">${escapeHtml(label)}</span>
    <span class="t-value">${escapeHtml(value)}</span>
    <span class="t-sub">${escapeHtml(sub || '')}</span>
  </div>`;
}

function renderTelemetry() {
  const row = $('#telemetry-row');
  if (!row) return;
  const edge = state.edge || {};
  const edgeParams = edge.params || {};
  const ros = state.health.ros || {};
  const rosSummary = ros.summary || {};
  const rosRuntime = ros.ros || {};
  const center = state.health.center || {};
  const model = center.model || {};
  const cards = [
    telemetryCard('EDGE UPTIME', fmtDuration((edgeParams.uptime_ms || 0) / 1000), `heap ${fmtBytes(edgeParams.free_heap)}`),
    telemetryCard('ROS DEVICES', `${rosSummary.ok || 0}/${rosSummary.total || 0}`, rosRuntime.missing_nodes && rosRuntime.missing_nodes.length ? `missing ${rosRuntime.missing_nodes.length} nodes` : 'all expected nodes'),
    telemetryCard('SAFETY', ros.emergency_stop ? 'E-STOP' : (ros.status ? 'ARMED' : '--'), ros.safety && ros.safety.reason ? ros.safety.reason : 'no safety state'),
    telemetryCard('CENTER MODEL', model.loaded ? 'LOADED' : (center.status ? 'IDLE' : '--'), `${model.backend || '--'} · ${model.device || '--'}`),
    telemetryCard('WIFI CLIENTS', edgeParams.wifi_clients == null ? '--' : String(edgeParams.wifi_clients), `max ${edgeParams.max_clients == null ? '--' : edgeParams.max_clients}`),
    telemetryCard('LAST SYNC', state.lastSync ? fmtTime(new Date(state.lastSync)) : '--', state.lastError || 'edge API'),
  ];
  row.innerHTML = cards.join('');
}

function drawerStatus(status) {
  return `<span class="status-chip status-${escapeHtml(status)}">${escapeHtml(statusLabel(status))}</span>`;
}

function openDrawer(id, tab = 'params') {
  state.activeModule = id;
  state.activeTab = tab;
  const drawer = $('#drawer');
  const scrim = $('#scrim');
  if (!drawer || !scrim) return;
  drawer.classList.add('is-open');
  drawer.setAttribute('aria-hidden', 'false');
  scrim.hidden = false;
  requestAnimationFrame(() => scrim.classList.add('is-open'));
  updateDrawerHeader();
  switchDrawerTab(tab);
  loadDrawer(id);
}

function closeDrawer() {
  const drawer = $('#drawer');
  const scrim = $('#scrim');
  if (!drawer || !scrim) return;
  drawer.classList.remove('is-open');
  drawer.setAttribute('aria-hidden', 'true');
  scrim.classList.remove('is-open');
  setTimeout(() => { scrim.hidden = true; }, 220);
  state.activeModule = null;
}

function updateDrawerHeader() {
  const id = state.activeModule;
  const meta = id ? MODULE_META[id] : null;
  const info = id ? moduleInfo(id) : null;
  const kicker = $('#drawer-kicker');
  const title = $('#drawer-title');
  const subtitle = $('#drawer-subtitle');
  if (kicker) kicker.textContent = id ? (id === 'edge' ? 'EDGE MODULE' : 'MODULE') : 'MODULE';
  if (title) {
    title.innerHTML = meta ? `${escapeHtml(meta.name)} ${info ? drawerStatus(info.status) : ''}` : '模块参数';
  }
  if (subtitle) {
    subtitle.textContent = meta ? `${meta.subtitle}${info && info.ip ? ' · ' + info.ip : ''}` : '--';
  }
}

function switchDrawerTab(tab) {
  state.activeTab = tab;
  for (const button of $$('.drawer-tabs .tab')) {
    const active = button.dataset.tab === tab;
    button.classList.toggle('is-active', active);
    button.setAttribute('aria-selected', active ? 'true' : 'false');
  }
  renderDrawer();
}

async function loadDrawer(id) {
  const body = $('#drawer-body');
  if (!body) return;
  body.innerHTML = '<div class="empty-state">正在读取模块参数…</div>';
  const saveButton = $('#save-params');
  if (saveButton) saveButton.disabled = true;

  let params = null;
  let health = null;
  try {
    if (id === 'edge') {
      params = await fetchJson(API.edgeParams);
      health = state.edge;
    } else {
      const results = await Promise.allSettled([
        fetchModuleJson(id, 'params'),
        fetchModuleJson(id, 'health'),
      ]);
      if (results[0].status === 'fulfilled') params = results[0].value;
      if (results[1].status === 'fulfilled') {
        health = results[1].value;
        state.health[id] = health;
      }
    }
  } catch (error) {
    state.lastError = error.message;
  }

  if (params) state.params[id] = params;
  if (health) state.health[id] = health;
  state.drawerError = params ? '' : '参数接口不可达';
  updateDrawerHeader();
  renderDrawer();
}

function normalizeParams(data, id) {
  if (!data) return [];
  if (Array.isArray(data.params)) return data.params;
  if (data.params && typeof data.params === 'object') {
    return Object.entries(data.params).map(([key, value]) => ({
      key,
      label: key,
      value,
      type: typeof value === 'boolean' ? 'bool' : (typeof value === 'number' ? 'float' : 'string'),
      group: '缓存参数',
      editable: false,
    }));
  }
  if (id === 'edge' && data.motion_keepalive_ms != null) {
    return Object.entries(data).map(([key, value]) => ({
      key, label: key, value,
      type: typeof value === 'boolean' ? 'bool' : (typeof value === 'number' ? 'float' : 'string'),
      group: '缓存参数', editable: false,
    }));
  }
  if (data && typeof data === 'object' && !Array.isArray(data)) {
    const entries = Object.entries(data).filter(([, value]) => value == null || ['string', 'number', 'boolean'].includes(typeof value));
    if (entries.length) {
      return entries.map(([key, value]) => ({
        key, label: key, value,
        type: typeof value === 'boolean' ? 'bool' : (typeof value === 'number' ? 'float' : 'string'),
        group: '缓存参数', editable: false,
      }));
    }
  }
  return [];
}

function renderDrawer() {
  const body = $('#drawer-body');
  if (!body) return;
  const id = state.activeModule;
  if (!id) return;
  const cachedParams = id === 'edge'
    ? (state.edge && state.edge.params)
    : (state.modules[id] && state.modules[id].params);
  const paramsData = state.params[id] || cachedParams;
  const editable = Boolean(paramsData && paramsData.editable);
  const saveButton = $('#save-params');
  if (saveButton) {
    saveButton.disabled = !editable || state.activeTab !== 'params';
    saveButton.style.display = state.activeTab === 'params' ? '' : 'none';
  }
  const hint = $('#drawer-hint');
  if (hint) {
    hint.textContent = state.drawerError
      ? state.drawerError
      : (editable ? '修改后立即生效；标注“重启”的参数需重启模块。' : '参数只读或模块未上报。');
  }

  if (state.activeTab === 'raw') {
    const health = state.health[id] || (id === 'edge' ? state.edge : null);
    const registry = id === 'edge' ? state.edge : state.modules[id];
    body.innerHTML = `
      <div class="raw-block"><h4>HEALTH</h4><pre>${escapeHtml(JSON.stringify(health || {}, null, 2))}</pre></div>
      <div class="raw-block"><h4>REGISTRY</h4><pre>${escapeHtml(JSON.stringify(registry || {}, null, 2))}</pre></div>
      <div class="raw-block"><h4>PARAMS</h4><pre>${escapeHtml(JSON.stringify(paramsData || {}, null, 2))}</pre></div>`;
    return;
  }

  const params = normalizeParams(paramsData, id);
  if (!params.length) {
    body.innerHTML = `<div class="empty-state">${escapeHtml(state.drawerError || '模块尚未上报参数。')}</div>`;
    return;
  }

  const groups = new Map();
  for (const param of params) {
    const group = param.group || '参数';
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(param);
  }

  const html = [];
  for (const [group, items] of groups) {
    html.push(`<section class="param-group"><h3 class="param-group-title">${escapeHtml(group)}</h3>`);
    for (const param of items) html.push(renderParamRow(param));
    html.push('</section>');
  }
  body.innerHTML = html.join('');
}

function renderParamRow(param) {
  const key = String(param.key || '');
  const type = String(param.type || (typeof param.value === 'boolean' ? 'bool' : (typeof param.value === 'number' ? 'float' : 'string')));
  const editable = Boolean(param.editable);
  const disabled = editable ? '' : 'disabled';
  const label = param.label || key;
  const unit = param.unit ? `<span class="param-unit">${escapeHtml(param.unit)}</span>` : '';
  const badge = param.requires_restart ? '<span class="param-badge">重启</span>' : '';
  const help = param.help ? `<p class="param-help">${escapeHtml(param.help)}</p>` : '';
  let control = '';

  if (type === 'bool') {
    control = `
      <label class="param-toggle">
        <input type="checkbox" data-param-key="${escapeHtml(key)}" data-param-type="bool" ${param.value ? 'checked' : ''} ${disabled}>
        <span class="toggle-track"><span class="toggle-thumb"></span></span>
      </label>`;
  } else if (type === 'enum' || (Array.isArray(param.options) && param.options.length)) {
    const options = (param.options || []).map((option) => {
      const value = typeof option === 'object' && option !== null ? option.value : option;
      const text = typeof option === 'object' && option !== null ? (option.label || option.value) : option;
      return `<option value="${escapeHtml(value)}" ${String(option) === String(param.value) || (option && option.value === param.value) ? 'selected' : ''}>${escapeHtml(text)}</option>`;
    }).join('');
    control = `<select data-param-key="${escapeHtml(key)}" data-param-type="enum" ${disabled}>${options}</select>`;
  } else {
    const inputType = type === 'string' ? 'text' : 'number';
    const step = param.step != null ? param.step : (type === 'int' ? 1 : 'any');
    const min = param.min != null ? `min="${escapeHtml(param.min)}"` : '';
    const max = param.max != null ? `max="${escapeHtml(param.max)}"` : '';
    control = `<input type="${inputType}" data-param-key="${escapeHtml(key)}" data-param-type="${escapeHtml(type)}" value="${escapeHtml(param.value == null ? '' : param.value)}" step="${escapeHtml(step)}" ${min} ${max} ${disabled}>`;
  }

  return `
    <div class="param-row">
      <div class="param-label">
        <strong>${escapeHtml(label)}${badge}</strong>
        <small>${escapeHtml(key)}</small>
      </div>
      <div class="param-control">${control}${unit}</div>
      ${help}
    </div>`;
}

function collectUpdates() {
  const updates = {};
  for (const input of $$('#drawer-body [data-param-key]')) {
    if (input.disabled) continue;
    const key = input.dataset.paramKey;
    const type = input.dataset.paramType;
    if (type === 'bool') {
      updates[key] = input.checked;
    } else if (type === 'int' || type === 'float') {
      const value = Number(input.value);
      if (!Number.isFinite(value)) throw new Error(`${key} 不是有效数字`);
      updates[key] = type === 'int' ? Math.round(value) : value;
    } else {
      updates[key] = input.value;
    }
  }
  return updates;
}

async function saveParams() {
  const id = state.activeModule;
  if (!id) return;
  let updates;
  try {
    updates = collectUpdates();
  } catch (error) {
    toast(error.message, 'error');
    return;
  }
  if (!Object.keys(updates).length) {
    toast('没有可保存的修改', 'error');
    return;
  }

  const button = $('#save-params');
  if (button) {
    button.disabled = true;
    button.textContent = '保存中…';
  }
  try {
    let result;
    if (id === 'edge') {
      const body = new URLSearchParams();
      for (const [key, value] of Object.entries(updates)) body.append(key, String(value));
      result = await fetchJson(API.edgeParams, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8' },
        body: body.toString(),
      });
    } else {
      const payload = JSON.stringify({ updates });
      const directUrl = state.modules[id] && state.modules[id].params_url;
      try {
        result = await fetchJson(API.params(id), {
          method: 'POST',
          headers: { 'Content-Type': 'text/plain;charset=UTF-8' },
          body: payload,
        });
      } catch (proxyError) {
        if (!directUrl) throw proxyError;
        result = await fetchJson(directUrl, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: payload,
        });
      }
    }
    toast(result && result.message ? result.message : '参数已保存', 'ok');
    await loadDrawer(id);
    await poll();
  } catch (error) {
    toast(`保存失败：${error.message}`, 'error');
  } finally {
    if (button) {
      button.textContent = '保存修改';
      button.disabled = false;
    }
  }
}

function toast(message, kind = '') {
  const element = $('#toast');
  if (!element) return;
  element.textContent = message;
  element.className = `toast is-visible${kind ? ' is-' + kind : ''}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => {
    element.classList.remove('is-visible');
  }, 3600);
}

async function pollBody() {
  if (state.bodyPolling) return;
  state.bodyPolling = true;
  try {
    const data = await fetchJson(API.body, {}, 2500);
    if (data && data.body) {
      state.bodyTelemetry = data.body;
      state.bodyFailures = 0;
      renderBody();
      renderTwin();
    }
  } catch (error) {
    // 旧固件没有 /api/body 时退化为完整轮询里的 edge.body。
    state.bodyFailures += 1;
    if (state.bodyTelemetry) {
      state.bodyTelemetry = { ...state.bodyTelemetry, stale: true };
    }
    if (state.bodyFailures >= 3 && state.bodyPollTimer) {
      clearInterval(state.bodyPollTimer);
      state.bodyPollTimer = null;
    }
  } finally {
    state.bodyPolling = false;
  }
}

function startPolling() {
  stopPolling();
  state.bodyFailures = 0;
  state.pollTimer = setInterval(() => {
    if (!document.hidden) poll();
  }, 5000);
  state.bodyPollTimer = setInterval(() => {
    if (!document.hidden) pollBody();
  }, 1000);
  pollBody();
  poll();
}

function stopPolling() {
  if (state.pollTimer) clearInterval(state.pollTimer);
  state.pollTimer = null;
  if (state.bodyPollTimer) clearInterval(state.bodyPollTimer);
  state.bodyPollTimer = null;
}

function bindEvents() {
  const refresh = $('#refresh-btn');
  if (refresh) refresh.addEventListener('click', () => { pollBody(); poll(); });

  const auto = $('#auto-refresh');
  if (auto) {
    auto.addEventListener('change', () => {
      if (auto.checked) startPolling();
      else stopPolling();
    });
  }

  const list = $('#module-list');
  if (list) {
    list.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-action]');
      if (!button) return;
      const id = button.dataset.module;
      const tab = button.dataset.action === 'raw' ? 'raw' : 'params';
      openDrawer(id, tab);
    });
  }

  const close = $('#drawer-close');
  if (close) close.addEventListener('click', closeDrawer);
  const scrim = $('#scrim');
  if (scrim) scrim.addEventListener('click', closeDrawer);

  for (const tab of $$('.drawer-tabs .tab')) {
    tab.addEventListener('click', () => switchDrawerTab(tab.dataset.tab));
  }

  const save = $('#save-params');
  if (save) save.addEventListener('click', saveParams);

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') closeDrawer();
  });

  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) { pollBody(); poll(); }
  });

  window.addEventListener('online', () => { pollBody(); poll(); });
}

function init() {
  bindEvents();
  initGauge();
  initMotorCards();
  state.clockTimer = setInterval(updateSyncText, 1000);
  startPolling();
}

document.addEventListener('DOMContentLoaded', init);
