const { app, BrowserWindow, ipcMain } = require('electron/main')
const path = require('node:path')

let bluetoothPinCallback
let selectBluetoothCallback

function createWindow () {
  const mainWindow = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  })

  mainWindow.webContents.on('select-bluetooth-device', (event, deviceList, callback) => {
    event.preventDefault()
    console.log('[bluetooth] select-bluetooth-device fired, devices:', deviceList)
    selectBluetoothCallback = callback
    // Send the list to the renderer so the user can pick a device.
    mainWindow.webContents.send('bluetooth-device-list', deviceList)
  })

  ipcMain.on('cancel-bluetooth-request', (event) => {
    if (selectBluetoothCallback) {
      selectBluetoothCallback('')
      selectBluetoothCallback = null
    }
  })

  ipcMain.on('bluetooth-device-selected', (event, deviceId) => {
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

  mainWindow.webContents.openDevTools()
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
