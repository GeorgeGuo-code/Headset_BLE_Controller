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

  // 保存配置到 JSON 文件
  ipcMain.removeHandler('save-configs')
  ipcMain.handle('save-configs', async (_event, json) => {
    const { canceled, filePath } = await dialog.showSaveDialog(mainWindow, {
      defaultPath: '我的配置.json',
      filters: [{ name: 'JSON 配置文件', extensions: ['json'] }]
    })
    if (canceled || !filePath) return { canceled: true }
    await fs.writeFile(filePath, String(json), 'utf-8')
    return { path: filePath }
  })

  // 从 JSON 文件加载配置
  ipcMain.removeHandler('load-configs')
  ipcMain.handle('load-configs', async () => {
    const { canceled, filePath } = await dialog.showOpenDialog(mainWindow, {
      filters: [{ name: 'JSON 配置文件', extensions: ['json'] }],
      properties: ['openFile']
    })
    if (canceled || !filePath || !filePath[0]) return { canceled: true }
    try {
      const data = await fs.readFile(filePath[0], 'utf-8')
      return { data }
    } catch (e) {
      return { error: e.message }
    }
  })

  // 选择 .exe 文件（用于"运行程序"步骤）
  ipcMain.removeHandler('open-exe-dialog')
  ipcMain.handle('open-exe-dialog', async () => {
    console.log('[main] open-exe-dialog handler called')
    const { canceled, filePaths } = await dialog.showOpenDialog(mainWindow, {
      filters: [
        { name: '可执行文件', extensions: ['exe', 'bat', 'cmd', 'lnk'] },
        { name: '所有文件', extensions: ['*'] }
      ],
      properties: ['openFile']
    })
    console.log('[main] open-exe-dialog result:', { canceled, filePaths })
    if (canceled || !filePaths || filePaths.length === 0) return { canceled: true }
    return { path: filePaths[0] }
  })

  // Chromium 边扫描边回调，每次有新设备都推给渲染进程（渲染端去重累积）。
  // 10 秒后仍为空才报告”未发现设备”。
  let emptyScanTimer = null

  mainWindow.webContents.on('select-bluetooth-device', (event, deviceList, callback) => {
    event.preventDefault()
    console.log('[bluetooth] select-bluetooth-device fired, devices:', deviceList)
    selectBluetoothCallback = callback

    if (emptyScanTimer) { clearTimeout(emptyScanTimer); emptyScanTimer = null }

    // 始终把当前列表推给渲染进程（渲染端按 deviceId 去重累积）
    mainWindow.webContents.send('bluetooth-device-list', deviceList || [])

    if (!deviceList || deviceList.length === 0) {
      emptyScanTimer = setTimeout(() => {
        emptyScanTimer = null
        mainWindow.webContents.send('bluetooth-device-list', [])
      }, 10000)
    }
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
