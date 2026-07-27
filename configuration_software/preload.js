const { contextBridge, ipcRenderer } = require('electron/renderer')

contextBridge.exposeInMainWorld('electronAPI', {
  cancelBluetoothRequest: () => ipcRenderer.send('cancel-bluetooth-request'),
  bluetoothDeviceSelected: (deviceId) => ipcRenderer.send('bluetooth-device-selected', deviceId),
  onBluetoothDeviceList: (callback) =>
    ipcRenderer.on('bluetooth-device-list', (_event, devices) => callback(devices)),
  bluetoothPairingRequest: (callback) => ipcRenderer.on('bluetooth-pairing-request', () => callback()),
  bluetoothPairingResponse: (response) => ipcRenderer.send('bluetooth-pairing-response', response)
})
