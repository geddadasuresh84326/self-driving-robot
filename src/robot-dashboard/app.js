// --- Global State ---
let ros = null;
let imageSubscriber = null;

// Telemetry counters
let lastFrameTime = performance.now();
let frameCount = 0;

// DOM Elements
const statusElement = document.getElementById('connection-status');
const wsUrlInput = document.getElementById('ws-url');
const connectBtn = document.getElementById('connect-btn');
const cameraFeed = document.getElementById('camera-feed');
const fpsDisplay = document.getElementById('fps-counter');
const imgSizeDisplay = document.getElementById('img-size');
const logConsole = document.getElementById('log-console');

// --- Helper Functions ---
function logMessage(msg, type = 'system-msg') {
    const logEntry = document.createElement('p');
    logEntry.className = type;
    const time = new Date().toLocaleTimeString();
    logEntry.textContent = `[${time}] ${msg}`;
    logConsole.appendChild(logEntry);
    logConsole.scrollTop = logConsole.scrollHeight; // Auto-scroll
}

function resetMetrics() {
    fpsDisplay.textContent = '0';
    imgSizeDisplay.textContent = '0 KB';
    cameraFeed.src = '';
    frameCount = 0;
}

// --- ROS Connection Management ---
function connectROS() {
    const url = wsUrlInput.value.trim();
    logMessage(`Connecting to ${url}...`);

    ros = new ROSLIB.Ros({ url: url });

    ros.on('connection', () => {
        statusElement.textContent = 'Connected';
        statusElement.className = 'connected';
        connectBtn.textContent = 'Disconnect';
        logMessage('Connected to rosbridge server.', 'success-msg');

        subscribeToCamera();
    });

    ros.on('error', (error) => {
        logMessage(`ROS Error: ${error}`, 'error-msg');
    });

    ros.on('close', () => {
        statusElement.textContent = 'Disconnected';
        statusElement.className = 'disconnected';
        connectBtn.textContent = 'Connect';
        logMessage('Disconnected from rosbridge server.', 'error-msg');
        resetMetrics();
    });
}

function disconnectROS() {
    if (imageSubscriber) {
        imageSubscriber.unsubscribe();
    }
    if (ros) {
        ros.close();
    }
}

connectBtn.addEventListener('click', () => {
    if (ros && ros.isConnected) {
        disconnectROS();
    } else {
        connectROS();
    }
});

// --- Camera Subscription ---
function subscribeToCamera() {
    imageSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/camera/image_raw/compressed',
        messageType: 'sensor_msgs/msg/CompressedImage'
    });

    imageSubscriber.subscribe((message) => {
        // Render JPEG frame directly from ROS Base64 payload
        cameraFeed.src = 'data:image/jpeg;base64,' + message.data;

        // Calculate size in KB
        const sizeInKB = (message.data.length * (3 / 4) / 1024).toFixed(1);
        imgSizeDisplay.textContent = `${sizeInKB} KB`;

        // Calculate live FPS
        frameCount++;
        const now = performance.now();
        if (now - lastFrameTime >= 1000) {
            fpsDisplay.textContent = frameCount;
            frameCount = 0;
            lastFrameTime = now;
        }
    });

    logMessage('Subscribed to /camera/image_raw/compressed', 'system-msg');
}