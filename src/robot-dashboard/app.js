// --- Global State ---
let ros = null;
let rawCameraSub = null;
let arucoCameraSub = null;

// Telemetry state tracking
const rawTelemetry = {
    feed: null,
    fpsDisplay: null,
    sizeDisplay: null,
    lastFrameTime: performance.now(),
    frameCount: 0
};

const arucoTelemetry = {
    feed: null,
    fpsDisplay: null,
    sizeDisplay: null,
    lastFrameTime: performance.now(),
    frameCount: 0
};

// DOM Elements
const statusElement = document.getElementById('connection-status');
const wsUrlInput = document.getElementById('ws-url');
const connectBtn = document.getElementById('connect-btn');
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

function resetMetrics(streamObj) {
    if (streamObj.fpsDisplay) streamObj.fpsDisplay.textContent = '0';
    if (streamObj.sizeDisplay) streamObj.sizeDisplay.textContent = '0 KB';
    if (streamObj.feed) streamObj.feed.src = '';
    streamObj.frameCount = 0;
}

function initDOMReferences() {
    rawTelemetry.feed = document.getElementById('raw-camera-feed');
    rawTelemetry.fpsDisplay = document.getElementById('raw-fps-counter');
    rawTelemetry.sizeDisplay = document.getElementById('raw-img-size');

    arucoTelemetry.feed = document.getElementById('aruco-camera-feed');
    arucoTelemetry.fpsDisplay = document.getElementById('aruco-fps-counter');
    arucoTelemetry.sizeDisplay = document.getElementById('aruco-img-size');
}

function updateFrameStats(message, telemetryObj) {
    // Render Base64 encoded JPEG payload
    telemetryObj.feed.src = 'data:image/jpeg;base64,' + message.data;

    // Calculate size in KB
    const sizeInKB = (message.data.length * (3 / 4) / 1024).toFixed(1);
    telemetryObj.sizeDisplay.textContent = `${sizeInKB} KB`;

    // Calculate live FPS
    telemetryObj.frameCount++;
    const now = performance.now();
    if (now - telemetryObj.lastFrameTime >= 1000) {
        telemetryObj.fpsDisplay.textContent = telemetryObj.frameCount;
        telemetryObj.frameCount = 0;
        telemetryObj.lastFrameTime = now;
    }
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

        subscribeToStreams();
    });

    ros.on('error', (error) => {
        logMessage(`ROS Error: ${error}`, 'error-msg');
    });

    ros.on('close', () => {
        statusElement.textContent = 'Disconnected';
        statusElement.className = 'disconnected';
        connectBtn.textContent = 'Connect';
        logMessage('Disconnected from rosbridge server.', 'error-msg');
        resetMetrics(rawTelemetry);
        resetMetrics(arucoTelemetry);
    });
}

function disconnectROS() {
    if (rawCameraSub) rawCameraSub.unsubscribe();
    if (arucoCameraSub) arucoCameraSub.unsubscribe();
    if (ros) ros.close();
}

connectBtn.addEventListener('click', () => {
    if (ros && ros.isConnected) {
        disconnectROS();
    } else {
        connectROS();
    }
});

// --- Camera Subscriptions ---
function subscribeToStreams() {
    // 1. Raw Camera Stream Subscriber
    rawCameraSub = new ROSLIB.Topic({
        ros: ros,
        name: '/line_detection/stream/compressed',
        messageType: 'sensor_msgs/msg/CompressedImage'
    });

    rawCameraSub.subscribe((message) => {
        updateFrameStats(message, rawTelemetry);
    });
    logMessage('Subscribed to /line_detection/stream/compressed', 'system-msg');

    // 2. ArUco Detection Stream Subscriber
    arucoCameraSub = new ROSLIB.Topic({
        ros: ros,
        name: '/aruco_detection/stream/compressed',
        messageType: 'sensor_msgs/msg/CompressedImage'
    });

    arucoCameraSub.subscribe((message) => {
        updateFrameStats(message, arucoTelemetry);
    });
    logMessage('Subscribed to /aruco_detection/stream/compressed', 'system-msg');
}

// Initialize DOM element tracking on script load
window.onload = () => {
    initDOMReferences();
};