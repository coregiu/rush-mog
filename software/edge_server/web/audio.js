// ==================== 语音识别控制功能 ====================
const voiceBtn = document.getElementById('voice-btn');
let recognition = null;
let voiceEnabled = false;
let voiceStopTimer = null;
let voiceScenarioPollTimer = null;

function stopVoiceMotion() {
    if (voiceStopTimer) {
        clearTimeout(voiceStopTimer);
        voiceStopTimer = null;
    }
}

// ==================== 语音找东西（场景编排） ====================
// 语音识别仍然在浏览器完成；识别出的文本 POST /api/voice，
// 由树莓派上的 voice_scenario_node 解析并编排“搜索 -> 确认 -> 接近 -> 播报”。
const FIND_UTTERANCE_RE = /(找|寻找|搜索|look\s*for|find|where\s+is|在哪)/i;
const SCENARIO_CANCEL_RE = /(别找|不要找|不用找|不找|停止寻找|停止搜索|取消寻找|取消搜索|stop\s+looking|cancel\s+search)/i;

function isFindUtterance(text) {
    return FIND_UTTERANCE_RE.test(String(text || ''));
}

function isScenarioCancel(text) {
    return SCENARIO_CANCEL_RE.test(String(text || ''));
}

function setVoiceScenarioStatus(text) {
    if (typeof yoloStatusText !== 'undefined' && yoloStatusText) {
        yoloStatusText.textContent = text;
    }
}

async function sendVoiceScenario(text) {
    try {
        const response = await fetch('/api/voice', {
            method: 'POST',
            headers: { 'Content-Type': 'text/plain; charset=utf-8' },
            body: text
        });
        let payload = {};
        try { payload = await response.json(); } catch (e) { payload = {}; }
        if (!response.ok) {
            const message = (payload.error && payload.error.message) || `HTTP ${response.status}`;
            throw new Error(message);
        }
        updateStatus(`🔎 ${text}`);
        if (payload.scenario_online === false) {
            setVoiceScenarioStatus('场景节点未在线');
            updateStatus('⚠️ 场景节点未在线');
        } else {
            setVoiceScenarioStatus(`🔎 ${text}`);
            startVoiceScenarioPolling();
        }
        return true;
    } catch (error) {
        console.error('语音场景指令失败:', error);
        updateStatus('⚠️ 语音场景不可用');
        setVoiceScenarioStatus('语音场景不可用');
        return false;
    }
}

function stopVoiceScenarioPolling() {
    if (voiceScenarioPollTimer) {
        clearInterval(voiceScenarioPollTimer);
        voiceScenarioPollTimer = null;
    }
}

function startVoiceScenarioPolling() {
    stopVoiceScenarioPolling();
    let polls = 0;
    voiceScenarioPollTimer = setInterval(async () => {
        polls += 1;
        if (polls > 150) { // 最长约 5 分钟
            stopVoiceScenarioPolling();
            return;
        }
        try {
            const response = await fetch('/api/voice', { cache: 'no-store' });
            if (!response.ok) return;
            const payload = await response.json();
            const scenario = payload.scenario;
            if (!scenario) return;
            const display = scenario.display_name || scenario.target_label || '东西';
            const state = scenario.state;
            if (state === 'searching') {
                setVoiceScenarioStatus(`🔎 正在找${display}…`);
            } else if (state === 'succeeded') {
                updateStatus(`✅ 找到${display}了`);
                setVoiceScenarioStatus(`✅ 找到${display}了`);
                stopVoiceScenarioPolling();
            } else if (state === 'failed') {
                updateStatus(`❌ 没找到${display}`);
                setVoiceScenarioStatus(`❌ 没找到${display}`);
                stopVoiceScenarioPolling();
            } else if (state === 'canceled') {
                updateStatus('⏹ 已停止寻找');
                setVoiceScenarioStatus('⏹ 已停止寻找');
                stopVoiceScenarioPolling();
            }
        } catch (error) {
            // 网络抖动时继续轮询，不打断寻找
        }
    }, 2000);
}

// 语音命令映射表（支持中英文）
const voiceCommands = {
    // 方向命令
    '前进': { key: 'up', type: 'down' },
    'forward': { key: 'up', type: 'down' },
    'go': { key: 'up', type: 'down' },
    '后退': { key: 'down', type: 'down' },
    'backward': { key: 'down', type: 'down' },
    'back': { key: 'down', type: 'down' },
    '向左': { key: 'left', type: 'down' },
    'left': { key: 'left', type: 'down' },
    '向右': { key: 'right', type: 'down' },
    'right': { key: 'right', type: 'down' },
    
    // 停止命令
    '停止': { key: 'stop', type: 'stop' },
    'stop': { key: 'stop', type: 'stop' },
    '停': { key: 'stop', type: 'stop' },
    
    // 动作按钮
    '三角': { key: 'triangle', type: 'press' },
    'triangle': { key: 'triangle', type: 'press' },
    '方块': { key: 'square', type: 'press' },
    'square': { key: 'square', type: 'press' },
    '圆形': { key: 'circle', type: 'press' },
    'circle': { key: 'circle', type: 'press' },
    '叉': { key: 'cross', type: 'press' },
    'cross': { key: 'cross', type: 'press' },
    'x': { key: 'cross', type: 'press' },
    
    // 肩键
    'L1': { key: 'l1', type: 'press' },
    'R1': { key: 'r1', type: 'press' },
    'L2': { key: 'l2', type: 'press' },
    'R2': { key: 'r2', type: 'press' },
    
    // 功能键
    '开始': { key: 'start', type: 'press' },
    'start': { key: 'start', type: 'press' },
    '选择': { key: 'select', type: 'press' },
    'select': { key: 'select', type: 'press' },
};

// 执行语音命令
function executeVoiceCommand(command) {
    const cmd = command.toLowerCase().trim();
    let matched = null;
    
    // 查找匹配的命令
    for (const [keyword, action] of Object.entries(voiceCommands)) {
        if (cmd.includes(keyword.toLowerCase())) {
            matched = action;
            break;
        }
    }
    
    if (matched) {
        if (matched.type === 'stop') {
            // 停止所有动作（服务端已支持 button:stop:down）
            stopVoiceMotion();
            sendButtonCommand('stop', 'down');
            updateStatus('🎙️ 停止');
        } else if (matched.type === 'press') {
            // 瞬时按键（按下后立即松开）
            sendButtonCommand(matched.key, 'down');
            setTimeout(() => {
                sendButtonCommand(matched.key, 'up');
            }, 200);
            updateStatus(`🎙️ ${matched.key.toUpperCase()}`);
        } else {
            // 方向键只在按下时发一条；STM32 会保持该状态，
            // 2 秒后由定时器发停止，期间不再发 HTTP 心跳
            stopVoiceMotion();
            sendButtonCommand(matched.key, matched.type);
            voiceStopTimer = setTimeout(() => {
                stopVoiceMotion();
                sendButtonCommand('stop', 'down');
            }, 2000);
            updateStatus(`🎙️ ${matched.key.toUpperCase()}`);
        }
        return true;
    }
    
    return false;
}

// 发送按钮命令（通过 80 端口 POST /cmd）
function sendCommandButton(key, type) {
    sendButtonCommand(key, type);
}

// 发送方向命令（通过 80 端口 POST /cmd）
function sendCommand(cmd) {
    sendButtonCommand((cmd === '0' || cmd === 'stop') ? 'stop' : cmd, 'down');
}

// 初始化语音识别
function initVoiceRecognition() {
    // 检查浏览器支持
    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    
    if (!SpeechRecognition) {
        alert('您的浏览器不支持语音识别功能。\n\n请使用 Chrome、Edge 或 Firefox 浏览器。');
        return false;
    }
    
    recognition = new SpeechRecognition();
    recognition.continuous = true; // 持续识别
    recognition.interimResults = false; // 只要最终结果
    recognition.lang = 'zh-CN'; // 默认中文
    
    // 识别结果处理
    recognition.onresult = (event) => {
        const lastResult = event.results[event.results.length - 1];
        const transcript = lastResult[0].transcript;
        const confidence = lastResult[0].confidence;
        
        console.log(`语音识别: "${transcript}" (置信度: ${Math.round(confidence * 100)}%)`);
        
        // 只显示高置信度的结果
        if (confidence > 0.6) {
            const plainText = String(transcript || '').trim();
            const isStopWord = /^(停|停下|停止|stop)$/i.test(plainText);
            // “找东西 / 停止寻找”交给场景编排层；先停车避免手动运动抢占导航。
            if (isScenarioCancel(plainText) || (isStopWord && voiceScenarioPollTimer)) {
                stopVoiceMotion();
                sendButtonCommand('stop', 'down');
                sendVoiceScenario(plainText);
                setVoiceScenarioStatus(`🎙️ ${plainText}`);
            } else if (isFindUtterance(plainText)) {
                stopVoiceMotion();
                sendButtonCommand('stop', 'down');
                sendVoiceScenario(plainText);
            } else if (executeVoiceCommand(plainText)) {
                setVoiceScenarioStatus(`"${plainText}"`);
            } else {
                setVoiceScenarioStatus(`未识别: "${plainText}"`);
            }
        }
    };
    
    recognition.onerror = (event) => {
        console.error('语音识别错误:', event.error);
        if (event.error === 'not-allowed') {
            alert('麦克风权限被拒绝。请在浏览器设置中允许麦克风访问。');
            voiceEnabled = false;
            voiceBtn.classList.remove('is-active');
            voiceBtn.classList.remove('animate-pulse');
        } else if (event.error === 'no-speech') {
            // 没有检测到语音，忽略
        } else {
            updateStatus('🎙️ Error: ' + event.error);
        }
    };
    
    recognition.onend = () => {
        if (voiceEnabled) {
            // 如果语音控制仍然启用，重新启动识别
            try {
                recognition.start();
            } catch (e) {
                console.log('语音识别重启失败:', e);
            }
        }
    };
    
    recognition.onstart = () => {
        updateStatus('🎙️ Listening...');
    };
    
    return true;
}

// 切换语音控制
function toggleVoiceControl() {
    if (!voiceEnabled) {
        // 开启语音控制
        if (!recognition) {
            const initialized = initVoiceRecognition();
            if (!initialized) return;
        }
        
        try {
            recognition.start();
            voiceEnabled = true;
            voiceBtn.classList.add('is-active'); // 激活态由 ps2.css 控制
            voiceBtn.classList.add('animate-pulse');
            updateStatus('🎙️ Voice ON');
        } catch (e) {
            console.error('启动语音识别失败:', e);
        }
    } else {
        // 关闭语音控制
        voiceEnabled = false;
        if (recognition) {
            recognition.stop();
        }
        stopVoiceMotion();
        // 关闭语音时兜底停车，避免上一条方向命令一直生效
        sendButtonCommand('stop', 'down');
        voiceBtn.classList.remove('is-active');
        voiceBtn.classList.remove('animate-pulse');
        updateStatus('🎙️ Voice OFF');
    }
}

// 语音按钮事件
voiceBtn.addEventListener('mousedown', () => {
    voiceBtn.classList.add('button-press');
    toggleVoiceControl();
});
voiceBtn.addEventListener('mouseup', () => {
    voiceBtn.classList.remove('button-press');
});
voiceBtn.addEventListener('mouseleave', () => {
    voiceBtn.classList.remove('button-press');
});

// 触摸事件
voiceBtn.addEventListener('touchstart', (e) => {
    e.preventDefault();
    voiceBtn.classList.add('button-press');
    toggleVoiceControl();
});
voiceBtn.addEventListener('touchend', (e) => {
    e.preventDefault();
    voiceBtn.classList.remove('button-press');
});