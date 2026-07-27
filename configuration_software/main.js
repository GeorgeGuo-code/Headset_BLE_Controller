const { app, BrowserWindow, ipcMain, dialog } = require('electron/main')
const path = require('node:path')
const fs = require('node:fs/promises')

let bluetoothPinCallback
let selectBluetoothCallback

function createWindow () {
  const mainWindow = new BrowserWindow({
    width: 1040,
    height: 800,
    title: 'HMBC 配置工具',
    backgroundColor: '#14171c',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  })

  // 保存日志到文件（渲染进程没有 fs 权限）。createWindow 可能被 'activate'
  // 再次调用，所以先摘掉旧 handler，避免重复注册抛错。
  ipcMain.removeHandler('save-log')
  ipcMain.handle('save-log', async (_event, text) => {
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)
    const { canceled, filePath } = await dialog.showSaveDialog(mainWindow, {
      defaultPath: `hmbc-calibration-${stamp}.log`,
      filters: [{ name: '日志文件', extensions: ['log', 'txt'] }]
    })
    if (canceled || !filePath) return { canceled: true }
    await fs.writeFile(filePath, String(text), 'utf-8')
    return { path: filePath }
  })

  // Chromium 边扫描边回调，前几次 deviceList 常常是空的。空列表不要直接推给
  // 渲染进程（否则界面立刻显示“未发现设备”），改为等真的有设备、或超时后再推。
  let emptyScanTimer = null

  mainWindow.webContents.on('select-bluetooth-device', (event, deviceList, callback) => {
    event.preventDefault()
    console.log('[bluetooth] select-bluetooth-device fired, devices:', deviceList)
    selectBluetoothCallback = callback

    if (emptyScanTimer) { clearTimeout(emptyScanTimer); emptyScanTimer = null }

    if (deviceList && deviceList.length > 0) {
      mainWindow.webContents.send('bluetooth-device-list', deviceList)
      return
    }
    // 还没扫到东西：保持“搜索中”，10 s 后仍为空才报告。
    emptyScanTimer = setTimeout(() => {
      emptyScanTimer = null
      mainWindow.webContents.send('bluetooth-device-list', [])
    }, 10000)
  })

  ipcMain.removeAllListeners('cancel-bluetooth-request')
  ipcMain.removeAllListeners('bluetooth-device-selected')
  ipcMain.removeAllListeners('bluetooth-pairing-response')

  ipcMain.on('cancel-bluetooth-request', (event) => {
    if (emptyScanTimer) { clearTimeout(emptyScanTimer); emptyScanTimer = null }
    if (selectBluetoothCallback) {
      selectBluetoothCallback('')
      selectBluetoothCallback = null
    }
  })

  ipcMain.on('bluetooth-device-selected', (event, deviceId) => {
    if (emptyScanTimer) { clearTimeout(emptyScanTimer); emptyScanTimer = null }
    if (selectBluetoothCallback) {
      selectBluetoothCallback(deviceId)
      selectBluetoothCallback = null
    }
  })

  // Listen for a message from the renderer to get the response for the Bluetooth pairing.
  ipcMain.on('bluetooth-pairing-response', (event, response) => {
    bluetoothPinCallback(response)
  })

  mainWindow.webContents.session.setBluetoothPairingHandler((details, callback) => {
    bluetoothPinCallback = callback
    // Send a message to the renderer to prompt the user to confirm the pairing.
    mainWindow.webContents.send('bluetooth-pairing-request', details)
  })

  mainWindow.loadFile('index.html')

  // mainWindow.webContents.openDevTools()
}

app.whenReady().then(() => {
  createWindow()

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })
})

app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit()
})
