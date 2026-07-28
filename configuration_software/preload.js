const { contextBridge, ipcRenderer } = require('electron/renderer')

contextBridge.exposeInMainWorld('electronAPI', {
  cancelBluetoothRequest: () => ipcRenderer.send('cancel-bluetooth-request'),
  bluetoothDeviceSelected: (deviceId) => ipcRenderer.send('bluetooth-device-selected', deviceId),
  onBluetoothDeviceList: (callback) =>
    ipcRenderer.on('bluetooth-device-list', (_event, devices) => callback(devices)),
  // 注意：details 必须转发给渲染进程，否则无法显示 PIN / deviceId。
  bluetoothPairingRequest: (callback) =>
    ipcRenderer.on('bluetooth-pairing-request', (_event, details) => callback(details)),
  bluetoothPairingResponse: (response) => ipcRenderer.send('bluetooth-pairing-response', response),
  saveLog: (text) => ipcRenderer.invoke('save-log', text)
})
