// ==================== 语音识别控制功能 ====================
const voiceBtn = document.getElementById('voice-btn');
let recognition = null;
let voiceEnabled = false;

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
            // 停止所有动作
            sendCommand('0');
            updateStatus('🎙️ 停止');
        } else if (matched.type === 'press') {
            // 瞬时按键（按下后立即松开）
            sendCommandButton(matched.key, 'down');
            setTimeout(() => {
                sendCommandButton(matched.key, 'up');
            }, 200);
            updateStatus(`🎙️ ${matched.key.toUpperCase()}`);
        } else {
            // 方向键（按下）
            sendCommandButton(matched.key, matched.type);
            updateStatus(`🎙️ ${matched.key.toUpperCase()}`);
        }
        return true;
    }
    
    return false;
}

// 发送按钮命令（通过 WebSocket 长连接）
function sendCommandButton(key, type) {
    sendButtonCommand(key, type);
}

// 发送方向命令（通过 WebSocket 长连接）
function sendCommand(cmd) {
    sendButtonCommand(cmd, 'down');
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
            if (executeVoiceCommand(transcript)) {
                yoloStatusText.textContent = `"${transcript}"`;
            } else {
                yoloStatusText.textContent = `未识别: "${transcript}"`;
            }
        }
    };
    
    recognition.onerror = (event) => {
        console.error('语音识别错误:', event.error);
        if (event.error === 'not-allowed') {
            alert('麦克风权限被拒绝。请在浏览器设置中允许麦克风访问。');
            voiceEnabled = false;
            voiceBtn.style.backgroundColor = '';
            voiceBtn.classList.remove('bg-red-500');
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
            voiceBtn.style.backgroundColor = '#FF6B6B'; // 红色表示开启
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
        voiceBtn.style.backgroundColor = ''; // 恢复蓝色
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