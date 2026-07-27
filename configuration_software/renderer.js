// Add your ESP32 custom service UUID(s) here if they are non-standard 128-bit.
// Example: '4fafc201-1fb5-459e-8fcc-c5c9c331914b'
// Without listing them, getPrimaryServices() may refuse to expose them.
const OPTIONAL_SERVICES = [
  0x00FF,
  0xFF01,
  0xFF02
]

let connectedDevice = null

async function testIt () {
  document.getElementById('device-name').innerHTML = ''
  document.getElementById('services').innerHTML = ''
  setStatus('Requesting device...')
  try {
    const device = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
      optionalServices: OPTIONAL_SERVICES
    })
    console.log('[bluetooth] device selected:', device)
    document.getElementById('device-name').innerHTML = device.name || `ID: ${device.id}`
    document.getElementById('device-list').innerHTML = ''
    setStatus(`Selected ${device.name || device.id}, connecting...`)

    device.addEventListener('gattserverdisconnected', () => {
      console.log('[bluetooth] gattserverdisconnected')
      setStatus('Disconnected')
      connectedDevice = null
    })

    const server = await device.gatt.connect()
    connectedDevice = device
    console.log('[bluetooth] connected, server:', server)
    setStatus('Connected. Discovering services...')

    const services = await server.getPrimaryServices()
    console.log('[bluetooth] services:', services)
    setStatus(`Connected. ${services.length} service(s) found.`)
    await renderServices(services)
  } catch (err) {
    console.error('[bluetooth] error:', err)
    setStatus(`Error: ${err.message || err}`)
  }
}

document.getElementById('clickme').addEventListener('click', testIt)

function cancelRequest () {
  window.electronAPI.cancelBluetoothRequest()
  document.getElementById('device-list').innerHTML = ''
  document.getElementById('services').innerHTML = ''
  document.getElementById('device-name').innerHTML = ''
  if (connectedDevice && connectedDevice.gatt.connected) {
    connectedDevice.gatt.disconnect()
  }
  connectedDevice = null
  setStatus('Cancelled')
}

document.getElementById('cancel').addEventListener('click', cancelRequest)

function setStatus (text) {
  document.getElementById('status').textContent = text
}

function renderDeviceList (devices) {
  const container = document.getElementById('device-list')
  container.innerHTML = ''
  if (!devices || devices.length === 0) {
    container.textContent = 'No devices found.'
    return
  }
  const heading = document.createElement('p')
  heading.textContent = 'Select a device:'
  container.appendChild(heading)
  devices.forEach((device) => {
    const btn = document.createElement('button')
    btn.textContent = device.deviceName || `ID: ${device.deviceId}`
    btn.style.display = 'block'
    btn.style.margin = '4px 0'
    btn.addEventListener('click', () => {
      window.electronAPI.bluetoothDeviceSelected(device.deviceId)
      container.innerHTML = ''
    })
    container.appendChild(btn)
  })
}

window.electronAPI.onBluetoothDeviceList(renderDeviceList)

async function renderServices (services) {
  const container = document.getElementById('services')
  container.innerHTML = ''
  if (services.length === 0) {
    container.textContent = 'No services found.'
    return
  }
  for (const service of services) {
    const serviceDiv = document.createElement('div')
    serviceDiv.style.border = '1px solid #aaa'
    serviceDiv.style.margin = '8px 0'
    serviceDiv.style.padding = '8px'
    serviceDiv.style.borderRadius = '4px'

    const title = document.createElement('h3')
    title.textContent = `Service: ${service.uuid}`
    serviceDiv.appendChild(title)

    container.appendChild(serviceDiv)

    try {
      const characteristics = await service.getCharacteristics()
      console.log(`[bluetooth] service ${service.uuid} has ${characteristics.length} characteristics`)
      if (characteristics.length === 0) {
        const empty = document.createElement('p')
        empty.textContent = '(no characteristics)'
        empty.style.color = '#888'
        serviceDiv.appendChild(empty)
        continue
      }
      for (const char of characteristics) {
        serviceDiv.appendChild(renderCharacteristic(char))
      }
    } catch (err) {
      const errDiv = document.createElement('p')
      errDiv.textContent = `Error reading characteristics: ${err.message || err}`
      errDiv.style.color = 'red'
      serviceDiv.appendChild(errDiv)
    }
  }
}

function formatValue (value) {
  const bytes = value instanceof DataView
    ? new Uint8Array(value.buffer, value.byteOffset, value.byteLength)
    : value
  const hex = Array.from(bytes).map(b => b.toString(16).padStart(2, '0')).join(' ')
  let text = ''
  try {
    const buf = value instanceof DataView ? value : new Uint8Array(value)
    text = new TextDecoder('utf-8', { fatal: false }).decode(buf)
    text = Array.from(text).map(c => {
      const code = c.charCodeAt(0)
      return (code >= 32 && code !== 127) ? c : '.'
    }).join('')
  } catch (e) {
    text = '(decode failed)'
  }
  return `hex: ${hex}\ntext: ${text}`
}

function parseHexInput (input) {
  const hex = (input || '').trim().replace(/[\s,]/g, '')
  if (hex.length === 0 || hex.length % 2 !== 0 || !/^[0-9a-fA-F]+$/.test(hex)) {
    throw new Error('invalid hex: ' + input)
  }
  const bytes = new Uint8Array(hex.length / 2)
  for (let i = 0; i < bytes.length; i++) {
    bytes[i] = parseInt(hex.substr(i * 2, 2), 16)
  }
  return bytes
}

function renderCharacteristic (char) {
  const div = document.createElement('div')
  div.style.margin = '6px 0'
  div.style.padding = '6px'
  div.style.borderLeft = '3px solid #4a90e2'
  div.style.background = '#f9f9f9'

  const props = []
  if (char.properties.read) props.push('read')
  if (char.properties.write) props.push('write')
  if (char.properties.writeWithoutResponse) props.push('writeWithoutResponse')
  if (char.properties.notify) props.push('notify')
  if (char.properties.indicate) props.push('indicate')

  const header = document.createElement('div')
  header.innerHTML = `<strong>Characteristic ${char.uuid}</strong> <span style="color:#666">[${props.join(', ') || 'no properties'}]</span>`
  div.appendChild(header)

  const valueDiv = document.createElement('pre')
  valueDiv.style.fontFamily = 'monospace'
  valueDiv.style.padding = '6px'
  valueDiv.style.background = '#fff'
  valueDiv.style.margin = '4px 0'
  valueDiv.style.border = '1px dashed #ccc'
  valueDiv.style.whiteSpace = 'pre-wrap'
  valueDiv.textContent = '(no value yet)'
  div.appendChild(valueDiv)

  const buttonRow = document.createElement('div')

  if (char.properties.read) {
    const readBtn = document.createElement('button')
    readBtn.textContent = 'Read'
    readBtn.addEventListener('click', async () => {
      try {
        const value = await char.readValue()
        valueDiv.textContent = formatValue(value)
      } catch (err) {
        valueDiv.textContent = `Read error: ${err.message || err}`
      }
    })
    buttonRow.appendChild(readBtn)
  }

  if (char.properties.write || char.properties.writeWithoutResponse) {
    const modeSelect = document.createElement('select')
    modeSelect.style.marginLeft = '8px'
    for (const opt of [
      { value: 'utf-8', label: 'utf-8' },
      { value: 'hex', label: 'hex' }
    ]) {
      const o = document.createElement('option')
      o.value = opt.value
      o.textContent = opt.label
      modeSelect.appendChild(o)
    }
    buttonRow.appendChild(modeSelect)

    const writeInput = document.createElement('input')
    writeInput.placeholder = 'utf-8 text'
    writeInput.style.marginLeft = '4px'
    buttonRow.appendChild(writeInput)

    modeSelect.addEventListener('change', () => {
      writeInput.placeholder = modeSelect.value === 'hex' ? 'hex bytes (e.g. 0100)' : 'utf-8 text'
    })

    const writeBtn = document.createElement('button')
    writeBtn.textContent = 'Write'
    writeBtn.style.marginLeft = '4px'
    writeBtn.addEventListener('click', async () => {
      try {
        const data = modeSelect.value === 'hex'
          ? parseHexInput(writeInput.value)
          : new TextEncoder().encode(writeInput.value)
        if (char.properties.writeWithoutResponse && !char.properties.write) {
          await char.writeValueWithoutResponse(data)
          valueDiv.textContent = formatValue(data) + '\n(wrote without response)'
        } else {
          await char.writeValue(data)
          valueDiv.textContent = formatValue(data) + '\n(wrote)'
        }
      } catch (err) {
        valueDiv.textContent = `Write error: ${err.message || err}`
      }
    })
    buttonRow.appendChild(writeBtn)
  }

  if (char.properties.notify || char.properties.indicate) {
    const notifyBtn = document.createElement('button')
    notifyBtn.textContent = 'Subscribe'
    notifyBtn.style.marginLeft = '4px'
    let subscribed = false
    let handler = null
    notifyBtn.addEventListener('click', async () => {
      try {
        if (!subscribed) {
          await char.startNotifications()
          handler = (event) => {
            valueDiv.textContent = formatValue(event.target.value) + '\n(live)'
          }
          char.addEventListener('characteristicvaluechanged', handler)
          subscribed = true
          notifyBtn.textContent = 'Unsubscribe'
        } else {
          char.removeEventListener('characteristicvaluechanged', handler)
          await char.stopNotifications()
          handler = null
          subscribed = false
          notifyBtn.textContent = 'Subscribe'
        }
      } catch (err) {
        valueDiv.textContent = `Notify error: ${err.message || err}`
      }
    })
    buttonRow.appendChild(notifyBtn)
  }

  div.appendChild(buttonRow)
  return div
}

window.electronAPI.bluetoothPairingRequest((event, details) => {
  const response = {}

  switch (details.pairingKind) {
    case 'confirm': {
      response.confirmed = window.confirm(`Do you want to connect to device ${details.deviceId}?`)
      break
    }
    case 'confirmPin': {
      response.confirmed = window.confirm(`Does the pin ${details.pin} match the pin displayed on device ${details.deviceId}?`)
      break
    }
    case 'providePin': {
      const pin = window.prompt(`Please provide a pin for ${details.deviceId}.`)
      if (pin) {
        response.pin = pin
        response.confirmed = true
      } else {
        response.confirmed = false
      }
    }
  }

  window.electronAPI.bluetoothPairingResponse(response)
})

setStatus('Idle. Click "Test Bluetooth" to start.')