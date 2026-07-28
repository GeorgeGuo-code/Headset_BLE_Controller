/*
 * HMBC 配置工具 — renderer
 *
 * 与固件的契约（见 ble_console.h）：
 *   Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX 6E400002-…  写入 ASCII 命令行：c / ca / ct / p / sp / sr
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
  NOD: '点头', LOOK_UP: '抬头', TILT_LEFT: '左倾', TILT_RIGHT: '右倾'
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
}

/* ── 日志 ─────────────────────────────────────────────────────────────────── */

function classify (line) {
  if (/^GESTURE /.test(line)) return 'gesture'
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

const STEP_DURATION_MS = { 1: 2000, 2: 4000, 3: 4000 }
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

/* 固件输出的提示串（main.c: run_guided_calibration / handle_command）：
 *   "== calibration 1/3: keep your head STILL =="
 *   "== calibration 2/3: do a few slow NODS now =="
 *   "== calibration 3/3: do slow LEFT and RIGHT tilts now =="
 *   "neutral capture failed: … — aborting" / "nod-axis capture failed: …"
 *   "calibration result: OK" / "<err name>"
 *   "nod calibration result: …" / "tilt calibration result: …"
 */
function trackCalibration (line) {
  let m = line.match(/^==\s*calibration\s*(\d)\/3/i)
  if (m) { startStep(Number(m[1])); return }

  if (/triggering guided calibration/i.test(line)) { resetSteps(); return }
  if (/nod calibration only/i.test(line))  { resetSteps(); startStep(2); return }
  if (/tilt calibration only/i.test(line)) { resetSteps(); startStep(3); return }

  if (/neutral capture failed/i.test(line))  { markStep(1, 'fail'); finishCal(false, '基准姿态采集失败：' + line); return }
  if (/nod-axis capture failed/i.test(line)) { markStep(2, 'fail'); finishCal(false, '点头轴采集失败：' + line); return }

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
  nod: '点头轴',
  tilt: '倾斜轴'
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
  if (m) { paramValues.sign_roll = m[1]; renderParams() }
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

/* "GESTURE NOD ts=12345 peak=23.4 vel=88.1" */
function trackGesture (line) {
  const m = line.match(/^GESTURE\s+(\w+)\s+ts=(\d+)\s+peak=(-?[\d.]+)\s+vel=(-?[\d.]+)/)
  if (!m) return
  const [, name, ts, peak, vel] = m
  if (name in gestureCounts) {
    gestureCounts[name]++
    renderGestures()
    const card = document.querySelector(`.badge[data-g="${name}"]`)
    if (card) {
      card.classList.add('hit')
      setTimeout(() => card.classList.remove('hit'), 400)
    }
  }
  $('gesture-last').textContent =
    `最近：${GESTURE_LABEL[name] || name}  峰值 ${peak}°  角速度 ${vel}°/s  (ts=${ts})`
}

/* ── BLE ──────────────────────────────────────────────────────────────────── */

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
  }
}

async function connect () {
  const useAll = $('chk-all').checked
  setStatus('搜索设备…', 'busy')
  try {
    device = await navigator.bluetooth.requestDevice(
      useAll
        ? { acceptAllDevices: true, optionalServices: [NUS_SERVICE] }
        : { filters: [{ namePrefix: DEVICE_NAME_PREFIX }], optionalServices: [NUS_SERVICE] })

    device.addEventListener('gattserverdisconnected', onDisconnected)
    pushLog(`选中设备 ${device.name || device.id}`, 'sys')
    setStatus('连接中…', 'busy')

    const server = await device.gatt.connect()
    const service = await server.getPrimaryService(NUS_SERVICE)
    rxChar = await service.getCharacteristic(NUS_RX)
    txChar = await service.getCharacteristic(NUS_TX)

    await txChar.startNotifications()
    txChar.addEventListener('characteristicvaluechanged', onTxChunk)

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
  rxChar = txChar = null
  rxTextBuf = ''
  pushLog('设备已断开', 'sys')
  if (calTimer) { clearInterval(calTimer); calTimer = null }
}

function disconnect () {
  window.electronAPI.cancelBluetoothRequest()
  $('device-list').classList.add('hidden')
  $('device-list').replaceChildren()
  if (device && device.gatt.connected) device.gatt.disconnect()
  else onDisconnected()
}

async function sendCmd (cmd, quiet) {
  if (!rxChar) { pushLog('未连接，无法发送命令', 'err'); return }
  try {
    await rxChar.writeValue(new TextEncoder().encode(cmd + '\n'))
    if (!quiet) pushLog('> ' + cmd, 'tx')
  } catch (err) {
    pushLog(`发送 "${cmd}" 失败：${err.message || err}`, 'err')
  }
}

/* ── 设备选择列表（Electron select-bluetooth-device） ──────────────────────── */

function renderDeviceList (devices) {
  const box = $('device-list')
  box.classList.remove('hidden')
  box.replaceChildren()
  const h = document.createElement('h2')
  h.textContent = devices && devices.length ? '选择设备' : '未发现设备'
  box.appendChild(h)

  if (!devices || devices.length === 0) {
    const tip = document.createElement('p')
    tip.className = 'hint'
    tip.textContent =
      '10 秒内没扫到广播。请检查：① 设备已上电且未被其它中心设备占用（连接后会停止广播）；' +
      `② 广播名应为 ${DEVICE_NAME_PREFIX}-Console，若固件改过名字请勾选“显示所有设备”；` +
      '③ Electron 必须运行在原生 Windows/Linux 主机上，WSL 里没有蓝牙适配器；' +
      '④ Windows 上先确认系统蓝牙已开启，并在“设置 → 蓝牙”里能看到该设备。'
    box.appendChild(tip)
    pushLog('扫描超时：未发现任何 BLE 设备', 'err')
    setStatus('未发现设备', null)
  }

  ;(devices || []).forEach((d) => {
    const btn = document.createElement('button')
    btn.textContent = d.deviceName || `ID: ${d.deviceId}`
    btn.addEventListener('click', () => {
      window.electronAPI.bluetoothDeviceSelected(d.deviceId)
      box.classList.add('hidden')
    })
    box.appendChild(btn)
  })
}

/* ── 事件绑定 ─────────────────────────────────────────────────────────────── */

$('btn-connect').addEventListener('click', connect)
$('btn-disconnect').addEventListener('click', disconnect)

document.querySelectorAll('button[data-cmd]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd
    if (cmd === 'c' || cmd === 'ca' || cmd === 'ct') resetSteps()
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

window.electronAPI.onBluetoothDeviceList(renderDeviceList)

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

renderGestures()
renderParams()
renderLog()
setStatus('未连接', null)
setConnected(false)
pushLog('就绪。点击“连接设备”搜索 HMBC-Console。', 'sys')
