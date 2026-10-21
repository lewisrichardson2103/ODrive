'use strict'

import { app, protocol, BrowserWindow, ipcMain } from 'electron'
import { createProtocol } from 'vue-cli-plugin-electron-builder/lib'
import installExtension, { VUEJS_DEVTOOLS } from 'electron-devtools-installer'
const isDevelopment = process.env.NODE_ENV !== 'production'

const { spawnSync, execSync } = require('child_process');
const spawn = require('child_process').spawn;
const path = require('path');

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let win;

// var for python server
let server;

// Scheme must be registered before the app is ready
protocol.registerSchemesAsPrivileged([
  { scheme: 'app', privileges: { secure: true, standard: true } }
])

// function to get determine correct command for python
function getPyCmd() {
  // call both 'python' and 'python3' to figure out the correct command
  let spawnRet = spawnSync('python', ['-V']);
  let success = spawnRet.status != null;
  let outputString;
  let cmd = '';
  if (success) {
    if (spawnRet.stdout.toString().length > 1) {
      outputString = spawnRet.stdout.toString();
    }
    else {
      outputString = spawnRet.stderr.toString();
    }
    if (outputString.includes("Python 3")) {
      cmd = 'python';
    }
  }
  if (cmd == '') {
    spawnRet = spawnSync('python3', ['-V']);
    success = spawnRet.status != null;
    if (success) {
      if (spawnRet.stdout.toString().length > 1) {
        outputString = spawnRet.stdout.toString();
      }
      else {
        outputString = spawnRet.stderr.toString();
      }
      if (outputString.includes("Python 3")) {
        cmd = 'python3';
      }
    }
  }
  return cmd;
}

function createWindow() {

  // Create the browser window.
  win = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      // Use pluginOptions.nodeIntegration, leave this alone
      // See nklayman.github.io/vue-cli-plugin-electron-builder/guide/security.html#node-integration for more info
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js')
    },
    icon: path.join(__static, 'icon.png')
  })
  win.maximize();
  win.setMenu(null);

  if (process.env.WEBPACK_DEV_SERVER_URL) {
    // Load the url of the dev server if in development mode
    win.loadURL(process.env.WEBPACK_DEV_SERVER_URL)
    if (!process.env.IS_TEST) win.webContents.openDevTools()
  } else {
    createProtocol('app')
    // Load the index.html when not in development
    win.loadURL('app://./index.html')
  }

  win.on('closed', () => {
    win = null
  })
}

// Quit when all windows are closed.
app.on('window-all-closed', () => {
  // kill flask server
  if (process.platform !== 'win32') {
    execSync('kill $(ps aux | grep \'[o]drive_server.py\' | awk \'{print $2}\')');
    console.log("killed flask server");
  }
  // On macOS it is common for applications and their menu bar
  // to stay active until the user quits explicitly with Cmd + Q
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  // On macOS it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  if (win === null) {
    createWindow()
  }
})

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on('ready', async () => {
    if (isDevelopment && !process.env.IS_TEST) {
        try {
            await installExtension(VUEJS_DEVTOOLS);
        } catch (e) {
            console.error(
                'Vue Devtools failed to install:',
                e.toString()
            );
        }
    }

    // Resolve local paths
    const scriptFilename = path.resolve(
        __dirname,
        '../server/odrive_server.py'
    );

    const toolsPath = path.resolve(
        __dirname,
        '../../tools'
    );

    const effectiveCommand = [
        scriptFilename,
        toolsPath
    ];

    console.log('Python backend:', scriptFilename);
    console.log('ODrive tools:', toolsPath);

    // Register IPC handler BEFORE creating the window
    ipcMain.on('start-server', () => {
        console.log('Received start-server event');

        if (server && server.exitCode === null) {
            console.log('Python server already running');
            return;
        }

        const pythonCommand = getPyCmd();

        console.log('Launching Python:', pythonCommand);
        console.log('Arguments:', effectiveCommand);

        server = spawn(pythonCommand, effectiveCommand, {
            cwd: path.dirname(scriptFilename),
            env: {
                ...process.env,
                PYTHONUNBUFFERED: '1'
            }
        });

        server.on('error', (err) => {
            console.error('Python startup failed:', err);
        });

        server.on('exit', (code, signal) => {
            console.log(
                `Python exited: ${code}, signal: ${signal}`
            );
        });

        server.stdout.on('data', (data) => {
            console.log(data.toString('utf8'));

            if (win && !win.isDestroyed()) {
                win.webContents.send(
                    'server-stdout',
                    data.toString('utf8')
                );
            }
        });

        server.stderr.on('data', (data) => {
            console.error(data.toString('utf8'));

            if (win && !win.isDestroyed()) {
                win.webContents.send(
                    'server-stderr',
                    data.toString('utf8')
                );
            }
        });
    });

    ipcMain.on('kill-server', () => {
        if (server && server.exitCode === null) {
            server.kill();
        }
    });

    // Create the GUI only once all IPC handlers exist
    createWindow();
});

// Exit cleanly on request from parent process in development mode.
if (isDevelopment) {
  if (process.platform === 'win32') {
    process.on('message', (data) => {
      if (data === 'graceful-exit') {
        server.kill('SIGINT');
        app.quit()
      }
    })
  } else {
    process.on('SIGTERM', () => {
      server.kill('SIGINT');
      app.quit()
    })
  }
}
