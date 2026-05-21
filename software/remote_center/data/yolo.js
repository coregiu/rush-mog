// ==================== YOLO 物体识别功能 ====================
const yoloBtn = document.getElementById('yolo-btn');
const detectionCanvas = document.getElementById('detection-canvas');
const yoloStatusDot = document.getElementById('yolo-status-dot');
const yoloStatusText = document.getElementById('yolo-status-text');
const ctx = detectionCanvas.getContext('2d');

let yoloEnabled = false;
let yoloModel = null;
let yoloDetectionInterval = null;
let yoloLoading = false;

// 颜色映射表（为不同类别分配不同颜色）
const colorMap = {};
const baseColors = [
    '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7',
    '#DDA0DD', '#98D8C8', '#F7DC6F', '#BB8FCE', '#85C1E9',
    '#F1948A', '#82E0AA', '#F8C471', '#AED6F1', '#A3E4D7'
];

function getColorForClass(className) {
    if (!colorMap[className]) {
        const index = Object.keys(colorMap).length % baseColors.length;
        colorMap[className] = baseColors[index];
    }
    return colorMap[className];
}

// 动态加载 TensorFlow.js 依赖脚本（仅加载一次）
// 注意：必须按顺序加载！coco-ssd.js 依赖 TensorFlow.js，所以先加载 tf.min.js，再加载 coco-ssd.js
let scriptsLoaded = false;
function loadDependencyScripts() {
    return new Promise((resolve, reject) => {
        if (scriptsLoaded) {
            resolve();
            return;
        }

        function loadScript(src) {
            return new Promise((res, rej) => {
                const existing = document.querySelector(`script[src="${src}"]`);
                if (existing) {
                    res();
                    return;
                }
                const script = document.createElement('script');
                script.src = src;
                script.onload = () => res();
                script.onerror = () => rej(new Error(`Failed to load: ${src}`));
                document.body.appendChild(script);
            });
        }

        // 按顺序加载：先 TensorFlow.js，再 COCO-SSD
        loadScript('/sdcard/tf.min.js')
            .then(() => loadScript('/sdcard/coco-ssd.js'))
            .then(() => {
                scriptsLoaded = true;
                resolve();
            })
            .catch(reject);
    });
}

// 加载YOLO模型（使用COCO-SSD本地模型）
// 先动态加载 TensorFlow.js 和 COCO-SSD 依赖，再加载模型
async function loadYoloModel() {
    if (yoloModel) return yoloModel;
    
    yoloLoading = true;
    yoloStatusDot.className = 'w-2 h-2 rounded-full bg-yellow-500 animate-pulse';
    yoloStatusText.textContent = 'Loading dependencies...';
    
    try {
        // 第一步：动态加载 TensorFlow.js 和 COCO-SSD
        await loadDependencyScripts();
        
        yoloStatusText.textContent = 'Loading model...';
        
        // 第二步：使用本地模型路径加载COCO-SSD模型
        // 必须指向 model.json 文件本身，而不是目录
        yoloModel = await cocoSsd.load({
            modelUrl: '/sdcard/models/coco-ssd/model.json'
        });
        console.log('YOLO/COCO-SSD model loaded successfully');
        
        yoloStatusDot.className = 'w-2 h-2 rounded-full bg-green-500';
        yoloStatusText.textContent = 'YOLO Ready';
        yoloLoading = false;
        
        return yoloModel;
    } catch (error) {
        console.error('Failed to load YOLO model:', error);
        yoloStatusDot.className = 'w-2 h-2 rounded-full bg-red-500';
        yoloStatusText.textContent = 'Model load failed - check network';
        yoloLoading = false;
        
        // 显示详细的错误信息和解决方案
        alert(
            'YOLO模型加载失败！\n\n' +
            '请检查模型文件是否存在于正确路径：/sdcard/models/coco-ssd/model.json\n\n' +
            '解决方案：\n' +
            '1. 确认模型文件已正确上传到设备\n' +
            '2. 检查文件路径是否正确\n' +
            '3. 如果使用PlatformIO，请确保模型文件通过data目录上传\n\n' +
            '错误详情：' + error.message
        );
        
        return null;
    }
}

// 执行物体检测
async function performDetection() {
    if (!yoloEnabled || !yoloModel || !isStreaming) return;
    
    try {
        // 检测物体
        const predictions = await yoloModel.detect(cameraImg);
        
        // 清除Canvas
        ctx.clearRect(0, 0, detectionCanvas.width, detectionCanvas.height);
        
        // 获取实际显示尺寸与Canvas尺寸的缩放比例
        const displayWidth = cameraImg.clientWidth;
        const displayHeight = cameraImg.clientHeight;
        const scaleX = detectionCanvas.width / displayWidth;
        const scaleY = detectionCanvas.height / displayHeight;
        
        // 绘制每个检测结果
        predictions.forEach(prediction => {
            const [x, y, width, height] = prediction.bbox;
            const color = getColorForClass(prediction.class);
            
            // 缩放坐标到Canvas尺寸
            const canvasX = x * scaleX;
            const canvasY = y * scaleY;
            const canvasW = width * scaleX;
            const canvasH = height * scaleY;
            
            // 绘制检测框
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.strokeRect(canvasX, canvasY, canvasW, canvasH);
            
            // 绘制标签背景
            const label = `${prediction.class} ${Math.round(prediction.score * 100)}%`;
            ctx.font = 'bold 10px Arial';
            const textWidth = ctx.measureText(label).width;
            const labelHeight = 16;
            
            ctx.fillStyle = color;
            ctx.fillRect(canvasX, canvasY - labelHeight, textWidth + 8, labelHeight);
            
            // 绘制标签文字
            ctx.fillStyle = '#FFFFFF';
            ctx.fillText(label, canvasX + 4, canvasY - 4);
        });
        
        // 更新状态
        if (predictions.length > 0) {
            const classNames = [...new Set(predictions.map(p => p.class))];
            yoloStatusText.textContent = `${predictions.length} objects: ${classNames.slice(0, 3).join(', ')}`;
        } else {
            yoloStatusText.textContent = 'No objects detected';
        }
        
    } catch (error) {
        console.error('Detection error:', error);
    }
}

// 切换YOLO开关
async function toggleYolo() {
    if (yoloLoading) return;
    
    if (!yoloEnabled) {
        // 开启YOLO
        if (!yoloModel) {
            const model = await loadYoloModel();
            if (!model) return;
        }
        
        yoloEnabled = true;
        yoloBtn.style.backgroundColor = '#FF6B6B'; // 红色表示开启
        yoloStatusDot.className = 'w-2 h-2 rounded-full bg-green-500 animate-pulse';
        yoloStatusText.textContent = 'YOLO On';
        
        // 开始定时检测（每500ms一次）
        yoloDetectionInterval = setInterval(performDetection, 500);
        
        updateStatus('🎯 YOLO ON');
        console.log('YOLO detection enabled');
    } else {
        // 关闭YOLO
        yoloEnabled = false;
        yoloBtn.style.backgroundColor = ''; // 恢复绿色
        yoloStatusDot.className = 'w-2 h-2 rounded-full bg-gray-500';
        yoloStatusText.textContent = 'YOLO Off';
        
        // 停止定时检测
        if (yoloDetectionInterval) {
            clearInterval(yoloDetectionInterval);
            yoloDetectionInterval = null;
        }
        
        // 清除Canvas
        ctx.clearRect(0, 0, detectionCanvas.width, detectionCanvas.height);
        
        updateStatus('🎯 YOLO OFF');
        console.log('YOLO detection disabled');
    }
}

// YOLO按钮事件
yoloBtn.addEventListener('mousedown', () => {
    yoloBtn.classList.add('button-press');
    toggleYolo();
});
yoloBtn.addEventListener('mouseup', () => {
    yoloBtn.classList.remove('button-press');
});
yoloBtn.addEventListener('mouseleave', () => {
    yoloBtn.classList.remove('button-press');
});

// 触摸事件
yoloBtn.addEventListener('touchstart', async (e) => {
    e.preventDefault();
    yoloBtn.classList.add('button-press');
    toggleYolo();
});
yoloBtn.addEventListener('touchend', (e) => {
    e.preventDefault();
    yoloBtn.classList.remove('button-press');
});

// 当摄像头流停止时，也停止YOLO检测
const originalStopCamera = stopCameraStream;
stopCameraStream = function() {
    originalStopCamera();
    if (yoloEnabled) {
        yoloEnabled = false;
        yoloBtn.style.backgroundColor = '';
        yoloStatusDot.className = 'w-2 h-2 rounded-full bg-gray-500';
        yoloStatusText.textContent = 'YOLO Off';
        if (yoloDetectionInterval) {
            clearInterval(yoloDetectionInterval);
            yoloDetectionInterval = null;
        }
        ctx.clearRect(0, 0, detectionCanvas.width, detectionCanvas.height);
    }
};

       