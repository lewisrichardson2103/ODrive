
import { contextBridge, ipcRenderer } from 'electron';

contextBridge.exposeInMainWorld('odriveElectron', {
    startServer: () => {
        ipcRenderer.send('start-server');
    },

    stopServer: () => {
        ipcRenderer.send('kill-server');
    },

    onServerStdout: (callback) => {
        ipcRenderer.on('server-stdout', (_event, data) => {
            callback(data);
        });
    },

    onServerStderr: (callback) => {
        ipcRenderer.on('server-stderr', (_event, data) => {
            callback(data);
        });
    }
});
