/*
 * HMBC 配置工具 — renderer
 *
 * 与固件的契约（见 ble_console.h）：
 *   Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX 6E400002-…  写入 ASCII 命令行：cr / cn / ctl / ctr / p / sp / sr
 *   TX 6E400003-…  notify，UTF-8 日志流（按 MTU-3 分片，需要按行重组）
 *   广播名 HMBC-Console
 */

const NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
const NUS_RX      = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
const NUS_TX      = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
const DEVICE_NAME_PREFIX = 'HMBC'

const $ = (id) => document.getElementById(id)

let device = null
let rxChar = null
let txChar = null
let rxTextBuf = ''            // TX 分片重组缓冲（按 \n 切行）
const logLines = []           // {ts, text, cls}
const gestureCounts = { NOD: 0, LOOK_UP: 0, TILT_LEFT: 0, TILT_RIGHT: 0 }
const GESTURE_LABEL = {
  REST: '静止', NOD: '点头', LOOK_UP: '抬头', TILT_LEFT: '左倾', TILT_RIGHT: '右倾'
}

/* 扫描期间累积发现的设备（按 deviceId 去重） */
let scanDevices = []
let scanTimeoutTimer = null

/* Web KeyboardEvent.code → HID keycode 映射（与固件 ascii_to_hid 对应） */
const WEB_CODE_TO_HID = {
  KeyA: 4, KeyB: 5, KeyC: 6, KeyD: 7, KeyE: 8, KeyF: 9, KeyG: 10,
  KeyH: 11, KeyI: 12, KeyJ: 13, KeyK: 14, KeyL: 15, KeyM: 16, KeyN: 17,
  KeyO: 18, KeyP: 19, KeyQ: 20, KeyR: 21, KeyS: 22, KeyT: 23, KeyU: 24,
  KeyV: 25, KeyW: 26, KeyX: 27, KeyY: 28, KeyZ: 29,
  Digit1: 30, Digit2: 31, Digit3: 32, Digit4: 33, Digit5: 34,
  Digit6: 35, Digit7: 36, Digit8: 37, Digit9: 38, Digit0: 39,
  Enter: 40, Escape: 41, Backspace: 42, Tab: 43, Space: 44,
  Minus: 45, Equal: 46, BracketLeft: 47, BracketRight: 48, Backslash: 49,
  Semicolon: 50, Quote: 51, Backquote: 52, Comma: 53, Period: 54, Slash: 55,
  CapsLock: 56,
  F1: 57, F2: 58, F3: 59, F4: 60, F5: 61, F6: 62,
  F7: 63, F8: 64, F9: 65, F10: 66, F11: 67, F12: 68,
  PrintScreen: 69, ScrollLock: 70, Pause: 71,
  Insert: 72, Home: 73, PageUp: 74, Delete: 75, End: 76, PageDown: 77,
  ArrowRight: 78, ArrowLeft: 79, ArrowDown: 80, ArrowUp: 81,
  NumLock: 82, NumpadDivide: 83, NumpadMultiply: 84, NumpadSubtract: 85,
  NumpadAdd: 86, NumpadEnter: 87,
  Numpad1: 88, Numpad2: 89, Numpad3: 90, Numpad4: 91, Numpad5: 92,
  Numpad6: 93, Numpad7: 94, Numpad8: 95, Numpad9: 96, Numpad0: 97, NumpadDecimal: 98
}

/* HID keycode → 人类可读名称（US-QWERTY 布局） */
const HID_KEY_NAME = {
  4: 'A', 5: 'B', 6: 'C', 7: 'D', 8: 'E', 9: 'F', 10: 'G',
  11: 'H', 12: 'I', 13: 'J', 14: 'K', 15: 'L', 16: 'M', 17: 'N',
  18: 'O', 19: 'P', 20: 'Q', 21: 'R', 22: 'S', 23: 'T', 24: 'U',
  25: 'V', 26: 'W', 27: 'X', 28: 'Y', 29: 'Z',
  30: '1', 31: '2', 32: '3', 33: '4', 34: '5',
  35: '6', 36: '7', 37: '8', 38: '9', 39: '0',
  40: 'Enter', 41: 'Esc', 42: 'Backspace', 43: 'Tab', 44: 'Space',
  45: '-', 46: '=', 47: '[', 48: ']', 49: '\\', 50: ';', 51: "'",
  52: '`', 53: ',', 54: '.', 55: '/',
  56: 'CapsLock', 57: 'F1', 58: 'F2', 59: 'F3', 60: 'F4',
  61: 'F5', 62: 'F6', 63: 'F7', 64: 'F8', 65: 'F9', 66: 'F10',
  67: 'F11', 68: 'F12', 69: 'PrintScreen', 70: 'ScrollLock', 71: 'Pause',
  72: 'Insert', 73: 'Home', 74: 'PageUp', 75: 'Delete', 76: 'End', 77: 'PageDown',
  78: 'Right', 79: 'Left', 80: 'Down', 81: 'Up',
  82: 'NumLock', 83: 'KP /', 84: 'KP *', 85: 'KP -', 86: 'KP +', 87: 'KP Enter',
  88: 'KP 1', 89: 'KP 2', 90: 'KP 3', 91: 'KP 4', 92: 'KP 5',
  93: 'KP 6', 94: 'KP 7', 95: 'KP 8', 96: 'KP 9', 97: 'KP 0', 98: 'KP .'
}

/* ── 状态显示 ─────────────────────────────────────────────────────────────── */

function setStatus (text, state) {
  $('status').textContent = text
  const dot = $('conn-dot')
  dot.className = 'dot' + (state ? ' ' + state : '')
}

function setConnected (on) {
  document.querySelectorAll('.need-conn').forEach((b) => { b.disabled = !on })
  $('btn-connect').disabled = on
  $('btn-disconnect').disabled = !on
  // 配置面板按钮（同步到设备、从设备读取需要连接）
  const cfgBtns = ['btn-cfg-sync', 'btn-cfg-read']
  cfgBtns.forEach((id) => { const el = $(id); if (el) el.disabled = !on })
}

/* ── 日志 ─────────────────────────────────────────────────────────────────── */

function classify (line) {
  if (/^GESTURE /.test(line)) return 'gesture'
  if (/^DETECT /.test(line)) return 'gesture'
  if (/^═══/.test(line)) return 'cal'
  if (/(failed|error|abort|ESP_ERR|unknown command)/i.test(line)) return 'err'
  if (/result:\s*OK|^boot complete/.test(line)) return 'ok'
  if (/^==|calibration|校准/i.test(line)) return 'cal'
  return ''
}

function pad (n) { return String(n).padStart(2, '0') }

function stamp (d) {
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.` +
         String(d.getMilliseconds()).padStart(3, '0')
}

function pushLog (text, cls) {
  logLines.push({ ts: stamp(new Date()), text, cls: cls || classify(text) })
  if (logLines.length > 5000) logLines.splice(0, logLines.length - 5000)
  renderLog()
}

function renderLog () {
  const filter = $('log-filter').value.trim().toLowerCase()
  const showTs = $('chk-ts').checked
  const el = $('log')
  const frag = document.createDocumentFragment()
  let shown = 0
  for (const l of logLines) {
    if (filter && !l.text.toLowerCase().includes(filter)) continue
    shown++
    const line = document.createElement('span')
    line.className = 'l ' + l.cls
    if (showTs) {
      const t = document.createElement('span')
      t.className = 't'
      t.textContent = l.ts + '  '
      line.appendChild(t)
    }
    line.appendChild(document.createTextNode(l.text))
    frag.appendChild(line)
  }
  el.replaceChildren(frag)
  $('log-count').textContent = filter
    ? `${shown} / ${logLines.length} 行`
    : `${logLines.length} 行`
  if ($('chk-autoscroll').checked) el.scrollTop = el.scrollHeight
}

/* ── 校准状态机（从日志文本推导 UI 状态） ─────────────────────────────────── */

const STEP_DURATION_MS = { 1: 2000, 2: 4000, 3: 4000, 4: 4000 }
let calTimer = null

function resetSteps () {
  document.querySelectorAll('.step').forEach((s) => { s.className = 'step' })
  $('cal-bar').style.width = '0'
  $('cal-result').textContent = ''
  $('cal-result').className = 'result'
  if (calTimer) { clearInterval(calTimer); calTimer = null }
}

function markStep (n, state) {
  const el = document.querySelector(`.step[data-step="${n}"]`)
  if (el) el.className = 'step ' + state
}

function startStep (n) {
  for (let i = 1; i < n; i++) {
    const el = document.querySelector(`.step[data-step="${i}"]`)
    if (el && !el.classList.contains('fail')) markStep(i, 'done')
  }
  markStep(n, 'active')
  if (calTimer) clearInterval(calTimer)
  const total = STEP_DURATION_MS[n] || 4000
  const t0 = Date.now()
  calTimer = setInterval(() => {
    const pct = Math.min(100, ((Date.now() - t0) / total) * 100)
    $('cal-bar').style.width = pct + '%'
    if (pct >= 100) { clearInterval(calTimer); calTimer = null }
  }, 60)
}

function finishCal (ok, msg) {
  if (calTimer) { clearInterval(calTimer); calTimer = null }
  $('cal-bar').style.width = ok ? '100%' : $('cal-bar').style.width
  $('cal-result').textContent = msg
  $('cal-result').className = 'result ' + (ok ? 'ok' : 'err')
}

/* 固件输出的提示串：
 *   "calibrating gesture NOD for 4000 ms (rest→peak)..."
 *   "gesture NOD: axis=[x y z] peak=25.3° ..."
 */
function trackCalibration (line) {
  if (/triggering guided calibration/i.test(line)) { resetSteps(); return }

  let m
  /* 4-step gesture calibration: REST → NOD → TILT_LEFT → TILT_RIGHT */
  if (/calibrating REST|calibrating REST/i.test(line))  { resetSteps(); startStep(1); return }
  if (/calibrating gesture NOD|calibrating NOD/i.test(line))       { resetSteps(); startStep(2); return }
  if (/calibrating gesture TILT_LEFT|calibrating TILT_LEFT/i.test(line)) { resetSteps(); startStep(3); return }
  if (/calibrating gesture TILT_RIGHT|calibrating TILT_RIGHT/i.test(line)){ resetSteps(); startStep(4); return }

  /* Old 3-step calibration markers (compat) */
  m = line.match(/^==\s*calibration\s*(\d)\/3/i)
  if (m) { startStep(Number(m[1])); return }
  if (/nod calibration only/i.test(line))  { resetSteps(); startStep(2); return }
  if (/tilt calibration only/i.test(line)) { resetSteps(); startStep(3); return }

  /* Calibration results — new format: gesture NOD: axis=[...] peak=XX° */
  m = line.match(/gesture (NOD|LOOK_UP|TILT_LEFT|TILT_RIGHT):\s*axis=\[([^\]]*)\]\s*peak=([\d.]+)/i)
  if (m) {
    const gestName = m[1].toUpperCase()
    const stepMap = { NOD: 2, TILT_LEFT: 3, TILT_RIGHT: 4 }
    const step = stepMap[gestName] || 2
    markStep(step, 'done')
    const label = GESTURE_LABEL[gestName] || gestName
    finishCal(true, `${label} 校准完成 (peak=${m[3]}°)`)
    sendCmd('p', true)
    return
  }

  /* Calibration results — old format: NOD: pn_mean=... sig=[...] */
  m = line.match(/^(NOD|LOOK_UP|TILT_LEFT|TILT_RIGHT)[:\s].*(sig=|axis=)/i)
  if (m) {
    const gestName = m[1].toUpperCase()
    const stepMap = { NOD: 2, TILT_LEFT: 3, TILT_RIGHT: 4 }
    const step = stepMap[gestName] || 2
    markStep(step, 'done')
    const label = GESTURE_LABEL[gestName] || gestName
    finishCal(true, `${label} 校准完成`)
    sendCmd('p', true)
    return
  }

  /* Failure messages */
  m = line.match(/gesture cal (NOD|LOOK_UP|TILT_LEFT|TILT_RIGHT):\s*(.*)/i)
  if (m && /fail|error|too few|degenerate/i.test(m[2])) {
    const gestName = m[1].toUpperCase()
    const stepMap = { NOD: 2, TILT_LEFT: 3, TILT_RIGHT: 4 }
    const step = stepMap[gestName] || 2
    markStep(step, 'fail')
    const label = GESTURE_LABEL[gestName] || gestName
    finishCal(false, `${label} 采集失败：${m[2]}`)
    return
  }

  /* Auto-sign inference */
  if (/signs inferred:/i.test(line)) {
    const sp = line.match(/sign_pitch=(\d)/)
    const sr = line.match(/sign_roll=(\d)/)
    if (sp) paramValues.sign_pitch = sp[1]
    if (sr) paramValues.sign_roll = sr[1]
    renderParams()
    finishCal(true, '全部手势校准完成，符号已自动推断')
    return
  }

  /* main.c BLE output: "REST calibration: OK" or "NOD calibration: OK" etc. */
  m = line.match(/^(REST|NOD|LOOK_UP|TILT_LEFT|TILT_RIGHT)\s+calibration:\s*(\S+)/i)
  if (m) {
    const gestName = m[1].toUpperCase()
    const ok = m[2].toUpperCase() === 'OK'
    const stepMap = { REST: 1, NOD: 2, TILT_LEFT: 3, TILT_RIGHT: 4 }
    const step = stepMap[gestName] || 1
    markStep(step, ok ? 'done' : 'fail')
    const label = GESTURE_LABEL[gestName] || gestName
    finishCal(ok, ok ? `${label} 校准完成` : `${label} 校准失败：${m[2]}`)
    if (ok) sendCmd('p', true)   // 成功后自动回读参数
    return
  }

  /* Legacy 3-step calibration results */
  m = line.match(/^(nod |tilt )?calibration result:\s*(\S+)/i)
  if (m) {
    const ok = m[2].toUpperCase() === 'OK'
    const step = m[1] ? (m[1].trim() === 'nod' ? 2 : 3) : 3
    markStep(step, ok ? 'done' : 'fail')
    if (ok && !m[1]) { markStep(1, 'done'); markStep(2, 'done') }
    finishCal(ok, ok ? '校准完成，参数已写入 NVS' : '校准失败：' + m[2])
    if (ok) sendCmd('p', true)   // 成功后自动回读参数
  }
}

/* ── 参数解析 ─────────────────────────────────────────────────────────────── */

const PARAM_LABEL = {
  trigger: '触发角度 (°)',
  vel: '触发角速度 (°/s)',
  zone: '中立区 (°)',
  debounce: '防抖 (ms)',
  sign_pitch: '俯仰符号 (1=正为点头)',
  sign_roll: '左右符号 (1=正为右倾)',
  q_neutral: '基准四元数 w x y z',
  nod_axis: '点头轴 (3D)',
  look_axis: '仰头轴 (3D)',
  tiltL_axis: '左倾轴 (3D)',
  tiltR_axis: '右倾轴 (3D)',
  sig_mask: '签名状态'
}

const paramValues = {}

function renderParams () {
  const body = $('params').querySelector('tbody')
  const rows = Object.keys(PARAM_LABEL)
    .filter((k) => k in paramValues)
    .map((k) => {
      const tr = document.createElement('tr')
      const a = document.createElement('td')
      a.textContent = PARAM_LABEL[k]
      const b = document.createElement('td')
      b.textContent = paramValues[k]
      tr.append(a, b)
      return tr
    })
  body.replaceChildren(...rows)
  $('params-empty').classList.toggle('hidden', rows.length > 0)
}

/* 固件输出（main.c handle_command "p"）：
 *   params: trigger=20.0 vel=50.0 zone=6.0 debounce=500 sign_pitch=1 sign_roll=1
 *     q_neutral=[..] nod=[..] tilt=[..]
 *   sign_pitch flipped -> positive_pitch_is_nod=0
 *   sign_roll flipped -> positive_roll_is_right=1
 */
function trackParams (line) {
  if (/^params:/.test(line)) {
    for (const m of line.matchAll(/(\w+)=(-?[\d.]+)/g)) paramValues[m[1]] = m[2]
    renderParams()
    return
  }
  const vec = line.match(/q_neutral=\[([^\]]*)\].*?nod=\[([^\]]*)\].*?tilt=\[([^\]]*)\]/)
  if (vec) {
    paramValues.q_neutral = vec[1].trim()
    paramValues.nod = vec[2].trim()
    paramValues.tilt = vec[3].trim()
    renderParams()
    return
  }
  let m = line.match(/positive_pitch_is_nod=(\d)/)
  if (m) { paramValues.sign_pitch = m[1]; renderParams(); return }
  m = line.match(/positive_roll_is_right=(\d)/)
  if (m) { paramValues.sign_roll = m[1]; renderParams(); return }

  /* 3D axis lines from "p" command:
   *   nod_axis   =[+.080 -.990 -.100] spread=8.2°
   *   look_axis  =[+.980 +.010 -.190] spread=12.1°
   *   tiltL_axis =[-.990 +.060 +.100] spread=6.5°
   *   tiltR_axis =[+.990 -.050 -.100] spread=7.0°
   *   sig_mask=0x0f
   */
  m = line.match(/nod_axis\s*=\[([^\]]*)\]\s*spread=(-?[\d.]+)/)
  if (m) { paramValues.nod_axis = m[1].trim(); renderParams(); return }
  m = line.match(/look_axis\s*=\[([^\]]*)\]\s*spread=(-?[\d.]+)/)
  if (m) { paramValues.look_axis = m[1].trim(); renderParams(); return }
  m = line.match(/tiltL_axis\s*=\[([^\]]*)\]\s*spread=(-?[\d.]+)/)
  if (m) { paramValues.tiltL_axis = m[1].trim(); renderParams(); return }
  m = line.match(/tiltR_axis\s*=\[([^\]]*)\]\s*spread=(-?[\d.]+)/)
  if (m) { paramValues.tiltR_axis = m[1].trim(); renderParams(); return }
  m = line.match(/sig_mask=(0x[0-9a-fA-F]+)/)
  if (m) {
    const mask = parseInt(m[1], 16)
    const names = []
    if (mask & 1) names.push('NOD')
    if (mask & 2) names.push('LOOK')
    if (mask & 4) names.push('T.L')
    if (mask & 8) names.push('T.R')
    paramValues.sig_mask = mask === 0x0f ? '✓ 全部完成' : names.join('+') || '未校准'
    renderParams()
  }
}

/* ── 手势事件 ─────────────────────────────────────────────────────────────── */

function renderGestures () {
  const box = $('gesture-badges')
  const cards = Object.keys(gestureCounts).map((k) => {
    const d = document.createElement('div')
    d.className = 'badge'
    d.dataset.g = k
    const n = document.createElement('span')
    n.className = 'n'
    n.textContent = gestureCounts[k]
    d.append(n, document.createTextNode(GESTURE_LABEL[k]))
    return d
  })
  box.replaceChildren(...cards)
}

function clearGestures () {
  for (const k of Object.keys(gestureCounts)) gestureCounts[k] = 0
  renderGestures()
  $('gesture-last').textContent = '等待手势…'
}

/* "GESTURE NOD ts=12345 peak=23.4 vel=88 conf=[NOD=0.70 LK=0.82 TL=0.85 TR=0.70] best=2" */
function trackGesture (line) {
  const m = line.match(/^GESTURE\s+(\w+)\s+ts=(\d+)\s+peak=(-?[\d.]+)\s+vel=(-?[\d.]+)\s+conf=\[NOD=(-?[\d.]+)\s+LK=(-?[\d.]+)\s+TL=(-?[\d.]+)\s+TR=(-?[\d.]+)\]\s*best=(-?\d+)/)
  if (!m) return
  const [, name, ts, peak, vel, cNod, cLk, cTl, cTr, bestIdx] = m
  const confArr = [parseFloat(cNod), parseFloat(cLk), parseFloat(cTl), parseFloat(cTr)]

  if (name in gestureCounts) {
    gestureCounts[name]++
    renderGestures()
    const card = document.querySelector(`.badge[data-g="${name}"]`)
    if (card) {
      card.classList.add('hit')
      setTimeout(() => card.classList.remove('hit'), 400)
    }
  }

  const confStr = confArr.map((v, i) => {
    const label = ['NOD', 'LK', 'TL', 'TR'][i]
    const pct = (v * 100).toFixed(0)
    const isBest = i === parseInt(bestIdx)
    return `${label}:${pct}%${isBest ? '*' : ''}`
  }).join('  ')

  $('gesture-last').textContent =
    `最近：${GESTURE_LABEL[name] || name}  峰值 ${peak}°  角速度 ${vel}°/s  [${confStr}]  (ts=${ts})`
}

/* ── BLE ──────────────────────────────────────────────────────────────────── */

/* Helper: connect GATT, discover NUS, subscribe notifications.
 * Disconnects first to flush stale GATT cache that causes
 * getPrimaryService() to hang after a reconnect. */
async function gattConnectAndSubscribe (dev) {
  /* Force-clear any cached GATT state from a previous session.
   * disconnect() on an already-disconnected device is a no-op. */
  if (dev.gatt.connected) {
    dev.gatt.disconnect()
  }
  /* Small delay so the controller finishes any pending teardown. */
  await new Promise((r) => setTimeout(r, 200))

  const server = await dev.gatt.connect()
  pushLog(`[CONN] GATT server connected`, 'sys')

  /* getPrimaryService can hang on stale cache — race with a 4 s timeout. */
  const service = await Promise.race([
    server.getPrimaryService(NUS_SERVICE),
    new Promise((_, rej) => setTimeout(() => rej(new Error('getPrimaryService timeout')), 4000))
  ])
  pushLog(`[CONN] got service ${NUS_SERVICE}`, 'sys')
  const rx = await service.getCharacteristic(NUS_RX)
  const tx = await service.getCharacteristic(NUS_TX)
  pushLog(`[CONN] RX props: ${JSON.stringify(rx.properties)}`, 'sys')
  pushLog(`[CONN] TX props: ${JSON.stringify(tx.properties)}`, 'sys')

  await tx.startNotifications()
  tx.addEventListener('characteristicvaluechanged', onTxChunk)

  rxChar = rx
  txChar = tx
}

function onTxChunk (event) {
  const bytes = new Uint8Array(event.target.value.buffer)
  rxTextBuf += new TextDecoder('utf-8').decode(bytes)
  const parts = rxTextBuf.split('\n')
  rxTextBuf = parts.pop()                  // 末尾不完整的一段留到下次
  for (const raw of parts) {
    const line = raw.replace(/\r$/, '')
    if (line === '') continue
    pushLog(line)
    trackCalibration(line)
    trackParams(line)
    trackGesture(line)
    trackConfigResponse(line)
    trackMouseMode(line)
  }
}

async function connect () {
  const useAll = $('chk-all').checked
  scanDevices = []  // 重置扫描列表
  setStatus('搜索设备…', 'busy')

  /* Try reconnecting to a previously authorized device first.
   * navigator.bluetooth.getDevices() returns devices the origin has
   * previously called requestDevice() for (cached across app restarts).
   * This avoids the scan popup when the device is already known. */
  if (!useAll) {
    try {
      if (navigator.bluetooth.getDevices) {
        const knownDevices = await navigator.bluetooth.getDevices()
        pushLog(`getDevices: 找到 ${knownDevices.length} 个已授权设备`, 'sys')
        const match = knownDevices.find(
          (d) => d.name && d.name.startsWith(DEVICE_NAME_PREFIX))
        if (match) {
          pushLog(`发现已授权设备 ${match.name || match.id}，正在重连…`, 'sys')
          setStatus('重连中…', 'busy')
          try {
            device = match
            device.addEventListener('gattserverdisconnected', onDisconnected)
            await gattConnectAndSubscribe(device)
            $('device-list').classList.add('hidden')
            setStatus(`已连接 ${device.name || device.id}`, 'on')
            setConnected(true)
            pushLog('已订阅日志通道', 'sys')
            sendCmd('p', true)
            return
          } catch (reconnErr) {
            pushLog(`已授权设备连接失败：${reconnErr.message}，回退到扫描…`, 'sys')
            if (device) {
              device.removeEventListener('gattserverdisconnected', onDisconnected)
              device = null
            }
          }
        } else {
          pushLog('未找到 HMBC 已授权设备，将进行扫描', 'sys')
        }
      } else {
        pushLog('getDevices 不支持，将进行扫描', 'sys')
      }
    } catch (getDevErr) {
      pushLog(`getDevices 出错：${getDevErr.message}，使用扫描`, 'sys')
    }
  }

  /* Fallback: normal requestDevice() scan with popup. */
  try {
    device = await navigator.bluetooth.requestDevice(
      useAll
        ? { acceptAllDevices: true, optionalServices: [NUS_SERVICE] }
        : { filters: [{ namePrefix: DEVICE_NAME_PREFIX }], optionalServices: [NUS_SERVICE] })

    device.addEventListener('gattserverdisconnected', onDisconnected)
    pushLog(`选中设备 ${device.name || device.id}`, 'sys')
    setStatus('连接中…', 'busy')

    await gattConnectAndSubscribe(device)

    $('device-list').classList.add('hidden')
    setStatus(`已连接 ${device.name || device.id}`, 'on')
    setConnected(true)
    pushLog('已订阅日志通道', 'sys')
    sendCmd('p', true)
  } catch (err) {
    setStatus('连接失败', null)
    pushLog(`连接错误：${err.message || err}`, 'err')
    setConnected(false)
  }
}

function onDisconnected () {
  setStatus('已断开', null)
  setConnected(false)
  /* Clean up stale device reference so the next connect() attempt
   * goes through getDevices()/requestDevice() fresh. */
  if (device) {
    device.removeEventListener('gattserverdisconnected', onDisconnected)
    device = null
  }
  rxChar = txChar = null
  rxTextBuf = ''
  pushLog('设备已断开', 'sys')
  if (calTimer) { clearInterval(calTimer); calTimer = null }
}

function disconnect () {
  window.electronAPI.cancelBluetoothRequest()
  $('device-list').classList.add('hidden')
  $('device-list').replaceChildren()
  if (device && device.gatt.connected) {
    /* gatt.disconnect() fires 'gattserverdisconnected' → onDisconnected()
     * which handles full cleanup (removing listener, nulling device, etc.) */
    device.gatt.disconnect()
  } else {
    onDisconnected()
  }
}

const BLE_CHUNK_SIZE = 18   /* safe payload for MTU 23 (20 − 2 headroom) */

async function sendCmd (cmd, quiet) {
  if (!rxChar) { pushLog('未连接，无法发送命令', 'err'); return }
  const data = new TextEncoder().encode(cmd + '\n')
  pushLog(`[TX] cmd len=${cmd.length}, encoded=${data.length} bytes, chunks=${Math.ceil(data.length / BLE_CHUNK_SIZE)}`, 'sys')
  try {
    if (data.length <= BLE_CHUNK_SIZE) {
      /* Short command — single write */
      pushLog(`[TX] single write ${data.length} bytes`, 'sys')
      await rxChar.writeValue(data)
    } else {
      /* Long command — chunk into BLE_CHUNK_SIZE pieces.
       * The firmware reassembles on its side (buffer until '\\n'). */
      const totalChunks = Math.ceil(data.length / BLE_CHUNK_SIZE)
      for (let off = 0; off < data.length; off += BLE_CHUNK_SIZE) {
        const chunk = data.slice(off, Math.min(off + BLE_CHUNK_SIZE, data.length))
        const seq = Math.floor(off / BLE_CHUNK_SIZE) + 1
        pushLog(`[TX] chunk ${seq}/${totalChunks}: ${chunk.length} bytes`, 'sys')
        await rxChar.writeValue(chunk)
        if (off + BLE_CHUNK_SIZE < data.length) {
          await new Promise((r) => setTimeout(r, 30))
        }
      }
    }
    pushLog(`[TX] OK: "${cmd.substring(0, 60)}${cmd.length > 60 ? '...' : ''}"`, 'ok')
    if (!quiet) pushLog('> ' + cmd, 'tx')
  } catch (err) {
    pushLog(`[TX] FAILED: ${err.message || err}`, 'err')
    pushLog(`发送 "${cmd}" 失败：${err.message || err}`, 'err')
  }
}

/* ── 设备选择列表（Electron select-bluetooth-device） ──────────────────────── */

function renderDeviceList (devices) {
  const box = $('device-list')
  box.classList.remove('hidden')
  box.replaceChildren()

  if (!devices || devices.length === 0) {
    const h = document.createElement('h2')
    h.textContent = '搜索中…'
    box.appendChild(h)
    const tip = document.createElement('p')
    tip.className = 'hint'
    tip.textContent = '正在扫描 BLE 设备，请稍候…'
    box.appendChild(tip)
    return
  }

  const h = document.createElement('h2')
  h.textContent = `发现 ${devices.length} 个设备`
  box.appendChild(h)

  devices.forEach((d) => {
    const btn = document.createElement('button')
    btn.textContent = d.deviceName || `ID: ${d.deviceId}`
    btn.addEventListener('click', () => {
      window.electronAPI.bluetoothDeviceSelected(d.deviceId)
      box.classList.add('hidden')
    })
    box.appendChild(btn)
  })
}

function onDeviceListUpdate (devices) {
  // 按 deviceId 去重累积
  for (const d of (devices || [])) {
    if (!scanDevices.find((x) => x.deviceId === d.deviceId)) {
      scanDevices.push(d)
    }
  }
  // 清除之前的超时定时器
  if (scanTimeoutTimer) { clearTimeout(scanTimeoutTimer); scanTimeoutTimer = null }

  if (scanDevices.length > 0) {
    renderDeviceList(scanDevices)
  } else {
    // 还没扫到：显示”搜索中”
    renderDeviceList([])
  }

  // 如果收到空列表且 10 秒内无新设备，显示超时提示
  if (!devices || devices.length === 0) {
    scanTimeoutTimer = setTimeout(() => {
      scanTimeoutTimer = null
      if (scanDevices.length === 0) {
        const box = $('device-list')
        box.classList.remove('hidden')
        box.replaceChildren()
        const h = document.createElement('h2')
        h.textContent = '未发现设备'
        box.appendChild(h)
        const tip = document.createElement('p')
        tip.className = 'hint'
        tip.textContent =
          '10 秒内没扫到广播。请检查：① 设备已上电；' +
          `② 广播名应为 ${DEVICE_NAME_PREFIX}-Console，若固件改过名字请勾选”显示所有设备”；` +
          '③ Electron 必须运行在原生 Windows/Linux 主机上，WSL 里没有蓝牙适配器；' +
          '④ Windows 上先确认系统蓝牙已开启。' +
          '⑤ 如曾连接过，可尝试点击"连接设备"直接重连已授权设备。'
        box.appendChild(tip)
        pushLog('扫描超时：未发现任何 BLE 设备', 'err')
        setStatus('未发现设备', null)
      }
    }, 10000)
  }
}

/* ── 事件绑定 ─────────────────────────────────────────────────────────────── */

$('btn-connect').addEventListener('click', connect)
$('btn-disconnect').addEventListener('click', disconnect)

document.querySelectorAll('button[data-cmd]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd
    if (cmd === 'cr' || cmd === 'cn' || cmd === 'ctl' || cmd === 'ctr') resetSteps()
    sendCmd(cmd)
  })
})

$('btn-send').addEventListener('click', () => {
  const v = $('cmd-input').value.trim()
  if (v) { sendCmd(v); $('cmd-input').value = '' }
})
$('cmd-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') $('btn-send').click()
})

$('log-filter').addEventListener('input', renderLog)
$('chk-ts').addEventListener('change', renderLog)
$('btn-clear').addEventListener('click', () => { logLines.length = 0; renderLog() })
$('btn-copy').addEventListener('click', async () => {
  await navigator.clipboard.writeText(logLines.map((l) => `${l.ts}  ${l.text}`).join('\n'))
  pushLog('日志已复制到剪贴板', 'sys')
})
$('btn-save').addEventListener('click', async () => {
  const res = await window.electronAPI.saveLog(
    logLines.map((l) => `${l.ts}  ${l.text}`).join('\n'))
  pushLog(res && res.path ? '日志已保存：' + res.path : '保存已取消', 'sys')
})

window.electronAPI.onBluetoothDeviceList(onDeviceListUpdate)

window.electronAPI.bluetoothPairingRequest((details) => {
  const response = {}
  switch (details && details.pairingKind) {
    case 'confirm':
      response.confirmed = window.confirm(`是否连接设备 ${details.deviceId}？`)
      break
    case 'confirmPin':
      response.confirmed = window.confirm(`设备 ${details.deviceId} 上显示的 PIN 是 ${details.pin} 吗？`)
      break
    case 'providePin': {
      const pin = window.prompt(`请输入 ${details.deviceId} 的配对码。`)
      response.confirmed = !!pin
      if (pin) response.pin = pin
      break
    }
    default:
      response.confirmed = true
  }
  window.electronAPI.bluetoothPairingResponse(response)
})

/* ── 命令配置管理 ─────────────────────────────────────────────────────────── */

const GESTURE_IDS = [
  { id: 1, name: '点头',   label: 'NOD' },
  { id: 2, name: '抬头',   label: 'LOOK_UP' },
  { id: 3, name: '头左倾', label: 'TILT_LEFT' },
  { id: 4, name: '头右倾', label: 'TILT_RIGHT' }
]
const GESTURE_MAX = 4  /* 最多同时选择的手势数量 */

/* 配置结构：{name, commands: [{id, triggers: [], steps: []}]} */
let configName = ''
let commands = []   // [{id, triggers:[], steps:[]}]
let cmdIdCounter = 0
let cfgResponseBuf = []

/* ── Step 描述文本 ────────────────────────────────────────────────────────── */

function stepDesc (s) {
  switch (s.kind) {
    case 'sleep': return `等待 ${s.ms} ms`
    case 'key': {
      const mods = []
      if (s.mod & 1) mods.push('Ctrl')
      if (s.mod & 2) mods.push('Shift')
      if (s.mod & 4) mods.push('Alt')
      if (s.mod & 8) mods.push('Win')
      const kname = HID_KEY_NAME[s.kc] || `0x${s.kc.toString(16)}`
      return `按键 ${mods.length ? mods.join('+') + '+' : ''}${kname}`
    }
    case 'type': return `输入 "${s.text}"`
    case 'click': {
      const names = { 1: '左键', 2: '右键', 4: '中键' }
      return `鼠标${names[s.button] || '?'}`
    }
    case 'scroll': {
      return `滚轮 ${s.clicks > 0 ? '↑' : '↓'}`
    }
    default: return '?'
  }
}

function stepToSeqText (s) {
  switch (s.kind) {
    case 'sleep': return `sleep ${s.ms}`
    case 'key': return `key ${s.mod} ${s.kc}`
    case 'type': return `type ${s.text}`
    case 'click': return `click ${s.button}`
    case 'scroll': return `scroll ${s.clicks}`
    default: return ''
  }
}

function commandToSeqText (cmd) {
  return (cmd.steps || []).map(stepToSeqText).join('; ')
}

/* 手势 bitmask ↔ ID 数组互转 */
function triggersToBitmask (triggers) {
  return triggers.reduce((m, id) => m | (1 << id), 0)
}

function bitmaskToTriggers (bm) {
  const r = []
  for (let i = 1; i <= 4; i++) { if (bm & (1 << i)) r.push(i) }
  return r
}

/* ── 渲染配置面板 ────────────────────────────────────────────────────────── */

let configCreated = false  /* 标记是否已点击"新建配置" */

function renderConfigs () {
  const box = $('cfg-list')
  box.replaceChildren()

  /* 显示配置头部：名称输入 + 新建命令按钮 */
  if (configCreated) {
    const header = document.createElement('div')
    header.className = 'cfg-config-header'
    const nameInput = document.createElement('input')
    nameInput.className = 'cfg-name-input'
    nameInput.value = configName
    nameInput.placeholder = '配置名称，如：我的快捷键'
    nameInput.addEventListener('input', () => { configName = nameInput.value })

    const addBtn = document.createElement('button')
    addBtn.textContent = '+ 新建命令'
    addBtn.className = 'primary'
    addBtn.addEventListener('click', addNewCommand)

    header.append(nameInput, addBtn)
    box.appendChild(header)
  }

  /* 命令列表 */
  if (commands.length === 0) {
    if (configCreated) {
      $('cfg-empty').classList.add('hidden')
    } else {
      $('cfg-empty').classList.remove('hidden')
    }
    return
  }
  $('cfg-empty').classList.add('hidden')

  for (const cmd of commands) {
    const card = document.createElement('div')
    card.className = 'cfg-card'
    card.dataset.id = cmd.id

    /* 第一行：编号 + 删除 */
    const row1 = document.createElement('div')
    row1.className = 'cfg-card-row1'
    const numSpan = document.createElement('span')
    numSpan.className = 'cfg-cmd-num'
    numSpan.textContent = `#${cmd.id}`
    const delBtn = document.createElement('button')
    delBtn.className = 'btn-del'
    delBtn.textContent = '✕ 删除'
    delBtn.addEventListener('click', async () => {
      // 从本地列表移除
      commands = commands.filter((x) => x.id !== cmd.id)
      renderConfigs()
      // 如果已连接，也从设备删除
      if (rxChar) {
        pushLog(`正在从设备删除配置 #${cmd.id}…`, 'sys')
        await sendCmd(`cmd del ${cmd.id}`)
      }
    })
    row1.append(numSpan, delBtn)

    /* 第二行：触发方式 */
    const row2 = document.createElement('div')
    row2.className = 'cfg-card-row2'

    const trigLabel = document.createElement('span')
    trigLabel.className = 'cfg-row-label'
    trigLabel.textContent = '触发方式：'

    const trigField = document.createElement('span')
    trigField.className = 'cfg-trig-field'

    function renderTrigTags () {
      trigField.replaceChildren()
      if (cmd.triggers.length === 0) {
        const hint = document.createElement('span')
        hint.className = 'cfg-trig-hint'
        hint.textContent = '（点击右侧按钮添加）'
        trigField.appendChild(hint)
      } else {
        for (const gid of cmd.triggers) {
          const g = GESTURE_IDS.find((x) => x.id === gid)
          const tag = document.createElement('span')
          tag.className = 'cfg-tag'
          tag.textContent = g ? g.name : `#${gid}`
          const x = document.createElement('span')
          x.className = 'cfg-tag-x'
          x.textContent = ' ×'
          x.addEventListener('click', () => {
            cmd.triggers = cmd.triggers.filter((t) => t !== gid)
            renderTrigTags()
          })
          tag.appendChild(x)
          trigField.appendChild(tag)
        }
      }
    }
    renderTrigTags()

    const trigBtns = document.createElement('span')
    trigBtns.className = 'cfg-trig-btns'
    for (const g of GESTURE_IDS) {
      const btn = document.createElement('button')
      btn.textContent = g.name
      btn.addEventListener('click', () => {
        if (cmd.triggers.includes(g.id)) {
          cmd.triggers = cmd.triggers.filter((t) => t !== g.id)
        } else if (cmd.triggers.length < GESTURE_MAX) {
          cmd.triggers.push(g.id)
        } else {
          pushLog(`最多选择 ${GESTURE_MAX} 个手势`, 'err')
          return
        }
        renderTrigTags()
      })
      trigBtns.appendChild(btn)
    }
    row2.append(trigLabel, trigField, trigBtns)

    /* 第三行：触发目标（步骤列表） */
    const row3 = document.createElement('div')
    row3.className = 'cfg-card-row3'

    const targetLabel = document.createElement('span')
    targetLabel.className = 'cfg-row-label'
    targetLabel.textContent = '触发目标：'

    const targetField = document.createElement('div')
    targetField.className = 'cfg-target-field'

    const stepsList = document.createElement('ul')
    stepsList.className = 'cfg-steps'
    renderStepsList(stepsList, cmd)

    const addBar = document.createElement('div')
    addBar.className = 'cfg-add-steps'

    const btnRun = document.createElement('button')
    btnRun.textContent = '▶ 运行程序'
    btnRun.addEventListener('click', () => addRunProgram(cmd))

    const btnKey = document.createElement('button')
    btnKey.textContent = '⌨ 输入按键'
    btnKey.addEventListener('click', () => openKeyDialog(cmd))

    const btnSleep = document.createElement('button')
    btnSleep.textContent = '⏱ 等待'
    btnSleep.addEventListener('click', () => addSleep(cmd))

    const btnUrl = document.createElement('button')
    btnUrl.textContent = '🔗 打开网址'
    btnUrl.addEventListener('click', () => addOpenUrl(cmd))

    addBar.append(btnRun, btnKey, btnSleep, btnUrl)
    targetField.append(stepsList, addBar)
    row3.append(targetLabel, targetField)

    card.append(row1, row2, row3)
    box.appendChild(card)
  }
}

function renderStepsList (ul, cmd) {
  ul.replaceChildren()
  ;(cmd.steps || []).forEach((s, i) => {
    const li = document.createElement('li')
    li.className = 'cfg-step'

    const num = document.createElement('span')
    num.className = 'step-num'
    num.textContent = `${i + 1}.`

    const desc = document.createElement('span')
    desc.className = 'step-desc'
    desc.textContent = stepDesc(s)

    const del = document.createElement('button')
    del.className = 'step-del'
    del.textContent = '✕'
    del.addEventListener('click', () => {
      cmd.steps.splice(i, 1)
      renderStepsList(ul, cmd)
    })

    li.append(num, desc, del)
    ul.appendChild(li)
  })
}

/* ── 添加步骤 ────────────────────────────────────────────────────────────── */

function addRunProgram (cmd) {
  console.log('[addRunProgram] called, cmd id:', cmd.id)
  window.electronAPI.openExeDialog().then((res) => {
    console.log('[addRunProgram] dialog result:', res)
    if (!res || res.canceled || !res.path) {
      console.log('[addRunProgram] dialog canceled or no path, returning')
      return
    }
    console.log('[addRunProgram] adding steps for path:', res.path)
    if (!cmd.steps) cmd.steps = []
    cmd.steps.push(
      { kind: 'key', mod: 8, kc: 21 },       // Win+R (mod 8=Win, kc 21=R)
      { kind: 'sleep', ms: 350 },
      { kind: 'type', text: res.path },
      { kind: 'key', mod: 0, kc: 40 }        // Enter
    )
    console.log('[addRunProgram] steps now:', JSON.stringify(cmd.steps))
    renderConfigs()
    console.log('[addRunProgram] renderConfigs done')
  }).catch((err) => {
    console.error('[addRunProgram] error:', err)
  })
}

function addSleep (cmd) {
  sleepDialogTarget = cmd
  $('sleep-input').value = '350'
  $('sleep-dialog').classList.remove('hidden')
  $('sleep-input').focus()
  $('sleep-input').select()
}

/* ── 等待时间对话框 ──────────────────────────────────────────────────────── */

let sleepDialogTarget = null

function closeSleepDialog () {
  $('sleep-dialog').classList.add('hidden')
  sleepDialogTarget = null
}

function confirmSleepDialog () {
  if (!sleepDialogTarget) return
  const v = parseInt($('sleep-input').value, 10)
  if (isNaN(v) || v <= 0 || v > 60000) { pushLog('无效的等待时间', 'err'); return }
  if (!sleepDialogTarget.steps) sleepDialogTarget.steps = []
  sleepDialogTarget.steps.push({ kind: 'sleep', ms: v })
  closeSleepDialog()
  renderConfigs()
}

$('sleep-dialog-ok').addEventListener('click', confirmSleepDialog)
$('sleep-dialog-cancel').addEventListener('click', closeSleepDialog)
$('sleep-dialog').addEventListener('click', (e) => {
  if (e.target === $('sleep-dialog')) closeSleepDialog()
})
$('sleep-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') confirmSleepDialog()
  if (e.key === 'Escape') closeSleepDialog()
})

/* ── 打开网址对话框 ─────────────────────────────────────────────────────── */

let urlDialogTarget = null

function addOpenUrl (cmd) {
  urlDialogTarget = cmd
  $('url-input').value = ''
  $('url-dialog').classList.remove('hidden')
  $('url-input').focus()
}

function closeUrlDialog () {
  $('url-dialog').classList.add('hidden')
  urlDialogTarget = null
}

function confirmUrlDialog () {
  if (!urlDialogTarget) return
  const url = $('url-input').value.trim()
  if (!url) { pushLog('请输入网址', 'err'); return }
  if (!urlDialogTarget.steps) urlDialogTarget.steps = []
  urlDialogTarget.steps.push(
    { kind: 'key', mod: 8, kc: 21 },       // Win+R
    { kind: 'sleep', ms: 350 },
    { kind: 'type', text: url },
    { kind: 'key', mod: 0, kc: 40 }        // Enter
  )
  closeUrlDialog()
  renderConfigs()
}

$('url-dialog-ok').addEventListener('click', confirmUrlDialog)
$('url-dialog-cancel').addEventListener('click', closeUrlDialog)
$('url-dialog').addEventListener('click', (e) => {
  if (e.target === $('url-dialog')) closeUrlDialog()
})
$('url-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') confirmUrlDialog()
  if (e.key === 'Escape') closeUrlDialog()
})

/* ── 新建配置 ────────────────────────────────────────────────────────────── */

function addNewConfig () {
  configName = ''
  commands = []
  cmdIdCounter = 0
  configCreated = true
  renderConfigs()
  const nameInput = $('cfg-list').querySelector('.cfg-name-input')
  if (nameInput) nameInput.focus()
}

/* ── 新建命令 ────────────────────────────────────────────────────────────── */

function addNewCommand () {
  cmdIdCounter++
  commands.push({
    id: cmdIdCounter,
    triggers: [],
    steps: []
  })
  renderConfigs()
  const card = document.querySelector(`.cfg-card[data-id="${cmdIdCounter}"]`)
  if (card) card.scrollIntoView({ behavior: 'smooth', block: 'nearest' })
}

/* ── 键盘按键捕获对话框 ──────────────────────────────────────────────────── */

let keyDialogTarget = null
let keyDialogKeys = {}
let keyDialogLastCombo = null  /* 松开按键后保存的最终组合 */
let keyDialogMousePending = null  /* 鼠标事件临时存储（无需等待松开） */

function openKeyDialog (cmd) {
  keyDialogTarget = cmd
  keyDialogKeys = {}
  keyDialogLastCombo = null
  keyDialogMousePending = null
  $('key-status').textContent = '请按下键盘按键或鼠标按钮…'
  $('key-status').classList.remove('dialog-status-warn')
  $('key-result').classList.add('hidden')
  $('key-dialog-ok').disabled = true
  $('key-dialog').classList.remove('hidden')
  document.addEventListener('keydown', onKeyDialogDown)
  document.addEventListener('keyup', onKeyDialogUp)
  document.addEventListener('mousedown', onKeyDialogMouseDown)
  document.addEventListener('wheel', onKeyDialogWheel, { passive: false })
}

function closeKeyDialog () {
  $('key-dialog').classList.add('hidden')
  document.removeEventListener('keydown', onKeyDialogDown)
  document.removeEventListener('keyup', onKeyDialogUp)
  document.removeEventListener('mousedown', onKeyDialogMouseDown)
  document.removeEventListener('wheel', onKeyDialogWheel)
  keyDialogTarget = null
  keyDialogKeys = {}
  keyDialogLastCombo = null
  keyDialogMousePending = null
}

function onKeyDialogDown (e) {
  e.preventDefault()
  e.stopPropagation()
  keyDialogKeys[e.code] = e
  keyDialogMousePending = null
  updateKeyDisplay()
}

function onKeyDialogUp (e) {
  e.preventDefault()
  e.stopPropagation()
  delete keyDialogKeys[e.code]
  if (Object.keys(keyDialogKeys).length === 0 && !keyDialogMousePending) {
    /* keyDialogLastCombo 已由 updateKeyDisplay 在按下时保存 */
    $('key-dialog-ok').disabled = false
  }
}

const MOUSE_BTN_LABEL = { 0: '鼠标左键', 2: '鼠标右键', 1: '鼠标中键' }
const MOUSE_BTN_HID   = { 0: { kind: 'click', button: 1 },
                           1: { kind: 'click', button: 4 },
                           2: { kind: 'click', button: 2 } }

function onKeyDialogMouseDown (e) {
  e.preventDefault()
  e.stopPropagation()
  /* 忽略对话框内的确认/取消按钮 */
  if (e.target.closest('#key-dialog-ok, #key-dialog-cancel, .dialog-close')) return
  const info = MOUSE_BTN_HID[e.button]
  if (!info) return
  keyDialogKeys = {}
  keyDialogMousePending = { ...info }
  $('key-status').textContent = `松开鼠标后点击确认`
  $('key-status').classList.add('dialog-status-warn')
  $('key-result').classList.remove('hidden')
  $('key-display').textContent = MOUSE_BTN_LABEL[e.button] || `鼠标按钮 ${e.button}`
  $('key-detail').textContent = `HID: click ${info.button}`
  keyDialogLastCombo = keyDialogMousePending
  $('key-dialog-ok').disabled = false
}

function onKeyDialogWheel (e) {
  e.preventDefault()
  e.stopPropagation()
  keyDialogKeys = {}
  const clicks = e.deltaY < 0 ? 1 : -1   /* 上滚=1, 下滚=-1 */
  keyDialogMousePending = { kind: 'scroll', clicks }
  $('key-status').textContent = '滚轮已识别，点击确认'
  $('key-status').classList.add('dialog-status-warn')
  $('key-result').classList.remove('hidden')
  $('key-display').textContent = clicks > 0 ? '滚轮 ↑' : '滚轮 ↓'
  $('key-detail').textContent = `HID: scroll ${clicks}`
  keyDialogLastCombo = keyDialogMousePending
  $('key-dialog-ok').disabled = false
}

function updateKeyDisplay () {
  const codes = Object.keys(keyDialogKeys)
  if (codes.length === 0) {
    $('key-status').textContent = '请按下键盘按键…'
    $('key-status').classList.remove('dialog-status-warn')
    $('key-result').classList.add('hidden')
    $('key-dialog-ok').disabled = true
    return
  }

  $('key-status').textContent = '松开所有按键后点击确认'
  $('key-status').classList.add('dialog-status-warn')
  $('key-result').classList.remove('hidden')

  let mod = 0
  const mods = []
  const first = Object.values(keyDialogKeys)[0]
  if (first.ctrlKey)  { mod |= 1; mods.push('Ctrl') }
  if (first.shiftKey) { mod |= 2; mods.push('Shift') }
  if (first.altKey)   { mod |= 4; mods.push('Alt') }
  if (first.metaKey)  { mod |= 8; mods.push('Win') }

  let mainHidKc = 0
  let mainName = ''
  for (const [code, ev] of Object.entries(keyDialogKeys)) {
    if (!isModifier(code)) {
      mainHidKc = WEB_CODE_TO_HID[code] || 0
      mainName = HID_KEY_NAME[mainHidKc] || code
      break
    }
  }

  /* 保存当前按键信息，供松开后 OK 按钮使用 */
  keyDialogLastCombo = { _modifierSnapshot: { ctrl: first.ctrlKey, shift: first.shiftKey, alt: first.altKey, meta: first.metaKey }, _mainKeyCode: mainHidKc }

  const displayName = mods.length
    ? (mainName ? mods.join(' + ') + ' + ' + mainName : mods.join(' + '))
    : (mainName || '?')

  $('key-display').textContent = displayName
  $('key-detail').textContent = `HID 键码: ${mainHidKc}`
}

function isModifier (code) {
  return code === 'ControlLeft' || code === 'ControlRight' ||
         code === 'ShiftLeft' || code === 'ShiftRight' ||
         code === 'AltLeft' || code === 'AltRight' ||
         code === 'MetaLeft' || code === 'MetaRight'
}

$('key-dialog-ok').addEventListener('click', () => {
  if (!keyDialogTarget || !keyDialogLastCombo) return

  /* 鼠标点击 / 滚轮事件：keyDialogLastCombo 带有 kind 属性 */
  if (keyDialogLastCombo.kind === 'click' || keyDialogLastCombo.kind === 'scroll') {
    if (!keyDialogTarget.steps) keyDialogTarget.steps = []
    keyDialogTarget.steps.push({ ...keyDialogLastCombo })
    closeKeyDialog()
    renderConfigs()
    return
  }

  /* 键盘按键事件 */
  const snap = keyDialogLastCombo._modifierSnapshot || {}
  let mod = 0
  if (snap.ctrl)  mod |= 1
  if (snap.shift) mod |= 2
  if (snap.alt)   mod |= 4
  if (snap.meta)  mod |= 8

  /* 从 updateKeyDisplay 保存的显示信息中取主键 */
  const kc = keyDialogLastCombo._mainKeyCode || 0

  if (kc === 0) { pushLog('请至少按下一个非修饰键', 'err'); return }

  if (!keyDialogTarget.steps) keyDialogTarget.steps = []
  keyDialogTarget.steps.push({ kind: 'key', mod, kc })
  closeKeyDialog()
  renderConfigs()
})

$('key-dialog-cancel').addEventListener('click', closeKeyDialog)
$('key-dialog').addEventListener('click', (e) => {
  if (e.target === $('key-dialog')) closeKeyDialog()
})

/* ── 同步到设备 / 从设备读取 ──────────────────────────────────────────────── */

async function syncConfigsToDevice () {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  pushLog(`[SYNC] 同步 ${commands.length} 个命令到设备…`, 'sys')
  for (const cmd of commands) {
    const bitmask = triggersToBitmask(cmd.triggers)
    const seq = commandToSeqText(cmd)
    const c = `cmd set ${cmd.id} ${configName || 'cmd' + cmd.id} gesture ${bitmask} ${seq}`
    pushLog(`[SYNC] 命令 #${cmd.id}: triggers=${JSON.stringify(cmd.triggers)} bitmask=${bitmask}`, 'sys')
    pushLog(`[SYNC] seq: ${seq}`, 'sys')
    await sendCmd(c, true)
    pushLog(`[SYNC] 等待 100ms...`, 'sys')
    await new Promise((r) => setTimeout(r, 100))
  }
  pushLog('同步完成', 'ok')
}

async function readConfigsFromDevice () {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  cfgResponseBuf = []
  pushLog('正在从设备读取配置…', 'sys')
  await sendCmd('cmd list', true)
  /* Also read mouse mode status */
  await sendCmd('mouse status', true)
  setTimeout(() => processConfigList(), 500)
}

async function deleteConfigFromDevice (id) {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  cfgResponseBuf = []
  pushLog(`正在删除配置 #${id}…`, 'sys')
  await sendCmd(`cmd del ${id}`)
  // 等待响应
  await new Promise((r) => setTimeout(r, 300))
  // 重新读取配置列表
  await readConfigsFromDevice()
}

async function deleteAllConfigsFromDevice () {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  if (!confirm('确定要删除设备上的所有配置吗？此操作不可撤销。')) return
  pushLog('正在删除设备所有配置…', 'sys')
  // 先读取当前配置
  cfgResponseBuf = []
  await sendCmd('cmd list', true)
  await new Promise((r) => setTimeout(r, 500))
  // 解析并删除每个配置
  for (const line of cfgResponseBuf) {
    const m = line.match(/cfg: id=(\d+)/)
    if (m) {
      const id = m[1]
      pushLog(`删除配置 #${id}…`, 'sys')
      await sendCmd(`cmd del ${id}`)
      await new Promise((r) => setTimeout(r, 100))
    }
  }
  commands = []
  renderConfigs()
  pushLog('已删除所有配置', 'ok')
}

function trackConfigResponse (line) {
  if (/^cfg:/.test(line)) cfgResponseBuf.push(line)
}

/* ── 鼠标模式状态追踪 ─────────────────────────────────────────────────── */

let mouseModeEnabled = false
let mouseModeActive = false
let mouseFourDir = false

function trackMouseMode (line) {
  /* Machine-parseable: "mouse_mode: enabled=1 active=0 fourdir=0 dz=2.0 ref=8.0 max=60 dwell=0" */
  const m = line.match(/^mouse_mode:\s*enabled=(\d)\s+active=(\d)\s+fourdir=(\d)\s+dz=([\d.]+)\s+ref=([\d.]+)\s+max=([\d.]+)\s+dwell=(\d+)/)
  if (m) {
    mouseModeEnabled = m[1] === '1'
    mouseModeActive = m[2] === '1'
    mouseFourDir = m[3] === '1'
    $('mouse-dz').textContent = m[4]
    $('mouse-ref').textContent = m[5]
    $('mouse-max').textContent = m[6]
    $('mouse-dwell').textContent = m[7] === '0' ? '立即' : m[7] + 'ms'
    $('mouse-params').classList.remove('hidden')
    renderMouseStatus()
    return
  }
  /* Real-time toggle events from firmware BLE log */
  if (/\[MOUSE\]\s*ACTIVATED/.test(line)) {
    mouseModeActive = true
    renderMouseStatus()
  } else if (/\[MOUSE\]\s*DEACTIVATED/.test(line)) {
    mouseModeActive = false
    renderMouseStatus()
  }
}

function renderMouseStatus () {
  const el = $('mouse-status-text')
  const btn = $('btn-mouse-toggle')
  if (mouseModeEnabled) {
    el.textContent = mouseModeActive ? '● 已激活' : '○ 已启用（等待手势触发）'
    el.style.color = mouseModeActive ? '#4caf50' : ''
    btn.textContent = '关闭'
    btn.classList.add('active')
  } else {
    el.textContent = '○ 已禁用'
    el.style.color = ''
    btn.textContent = '开启'
    btn.classList.remove('active')
  }
  const btn4 = $('btn-mouse-fourdir')
  if (btn4) {
    btn4.textContent = mouseFourDir ? '四向移动: 开' : '四向移动: 关'
    if (mouseFourDir) {
      btn4.classList.add('active')
    } else {
      btn4.classList.remove('active')
    }
  }
}

function processConfigList () {
  if (cfgResponseBuf.length === 0) {
    pushLog('未收到配置数据', 'err')
    return
  }
  commands = []
  for (const line of cfgResponseBuf) {
    const m = line.match(/cfg: id=(\d+) name="([^"]*)" trigger=(\S+) n_steps=(\d+)/)
    if (!m) continue
    const [, id, name, trigger, nSteps] = m
    let triggers = []
    if (trigger.startsWith('gesture:')) {
      const bm = Number(trigger.slice(8))
      triggers = bitmaskToTriggers(bm)
    }
    commands.push({
      id: Number(id),
      triggers,
      steps: []
    })
  }
  cmdIdCounter = commands.length > 0 ? Math.max(...commands.map((c) => c.id)) : 0
  configCreated = true
  fetchConfigSeqs(0)
}

async function fetchConfigSeqs (idx) {
  if (idx >= commands.length) {
    renderConfigs()
    pushLog(`已读取 ${commands.length} 个命令`, 'ok')
    return
  }
  cfgResponseBuf = []
  await sendCmd(`cmd get ${commands[idx].id}`, true)
  setTimeout(() => {
    for (const line of cfgResponseBuf) {
      const sm = line.match(/cfg: steps=(.*)/)
      if (sm) commands[idx].steps = parseSeqTextToSteps(sm[1])
    }
    fetchConfigSeqs(idx + 1)
  }, 200)
}

function parseSeqTextToSteps (text) {
  const CLICK_NAME_MAP = { left: 1, right: 2, middle: 4, 1: 1, 2: 2, 4: 4 }
  const steps = []
  for (const seg of text.split(';')) {
    const t = seg.trim()
    if (!t) continue
    const parts = t.split(/\s+/)
    const kw = parts[0]
    if (kw === 'sleep' && parts[1]) {
      steps.push({ kind: 'sleep', ms: parseInt(parts[1], 10) })
    } else if (kw === 'key' && parts[1] && parts[2]) {
      steps.push({ kind: 'key', mod: parseInt(parts[1], 10), kc: parseInt(parts[2], 10) })
    } else if (kw === 'type') {
      steps.push({ kind: 'type', text: parts.slice(1).join(' ') })
    } else if (kw === 'click' && parts[1]) {
      const btn = CLICK_NAME_MAP[parts[1]] || parseInt(parts[1], 10)
      steps.push({ kind: 'click', button: btn })
    } else if (kw === 'scroll' && parts[1]) {
      steps.push({ kind: 'scroll', clicks: parseInt(parts[1], 10) })
    }
  }
  return steps
}

/* ── 配置文件 I/O（PC 端 JSON） ────────────────────────────────────────────── */

async function saveConfigsToFile () {
  const data = { name: configName, commands }
  const json = JSON.stringify(data, null, 2)
  const res = await window.electronAPI.saveConfigs(json)
  pushLog(res && res.path ? '配置已保存：' + res.path : '保存已取消', 'sys')
}

async function loadConfigsFromFile () {
  const res = await window.electronAPI.loadConfigs()
  if (!res || res.canceled) { pushLog('加载已取消', 'sys'); return }
  if (res.error) { pushLog('加载失败：' + res.error, 'err'); return }
  try {
    const data = JSON.parse(res.data)
    configName = data.name || ''
    commands = data.commands || []
    cmdIdCounter = commands.length > 0 ? Math.max(...commands.map((c) => c.id)) : 0
    configCreated = true
    renderConfigs()
    pushLog(`已加载配置"${configName}"，${commands.length} 个命令`, 'ok')
  } catch (e) {
    pushLog('JSON 解析失败：' + e.message, 'err')
  }
}

/* ── 配置面板事件绑定 ──────────────────────────────────────────────────────── */

$('btn-cfg-new').addEventListener('click', addNewConfig)
$('btn-cfg-sync').addEventListener('click', syncConfigsToDevice)
$('btn-cfg-read').addEventListener('click', readConfigsFromDevice)
$('btn-cfg-del-all').addEventListener('click', deleteAllConfigsFromDevice)
$('btn-cfg-save').addEventListener('click', saveConfigsToFile)
$('btn-cfg-load').addEventListener('click', loadConfigsFromFile)
$('btn-gesture-clear').addEventListener('click', clearGestures)

/* ── 触发难度设置 ────────────────────────────────────────────────────────── */

$('btn-set-conf').addEventListener('click', async () => {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  const v = parseInt($('conf-input').value, 10)
  if (isNaN(v) || v < 0 || v > 100) {
    pushLog('触发难度必须在 0–100 之间', 'err')
    return
  }
  /* 发送 cmd fuzzy 只给已存在的配置槽位。 */
  $('conf-result').textContent = '发送中…'
  for (const cmd of commands) {
    await sendCmd(`cmd fuzzy ${cmd.id} 0 0 ${v}`, true)
    await new Promise((r) => setTimeout(r, 50))
  }
  $('conf-result').textContent = `已设置 (conf=${v})`
  pushLog(`触发难度已设置为 ${v}`, 'ok')
})
$('btn-mouse-toggle').addEventListener('click', async () => {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  const cmd = mouseModeEnabled ? 'mouse off' : 'mouse on'
  await sendCmd(cmd)
  mouseModeEnabled = !mouseModeEnabled
  renderMouseStatus()
})
$('btn-mouse-fourdir').addEventListener('click', async () => {
  if (!rxChar) { pushLog('未连接', 'err'); return }
  const cmd = mouseFourDir ? 'mouse fourdir off' : 'mouse fourdir on'
  await sendCmd(cmd)
  mouseFourDir = !mouseFourDir
  renderMouseStatus()
  pushLog(mouseFourDir ? '四向移动模式已开启' : '四向移动模式已关闭', 'ok')
})

/* ── 初始化 ─────────────────────────────────────────────────────────────── */

renderGestures()
renderParams()
renderConfigs()
renderLog()
setStatus('未连接', null)
setConnected(false)
pushLog('就绪。点击“连接设备”搜索 HMBC-Console。', 'sys')
