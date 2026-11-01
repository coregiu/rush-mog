# 自定义音频消息。
# 原设计建议使用 audio_common_msgs/AudioData，但该 ROS 包不是 Humble 基础安装的一部分，
# 为保证本工作空间可直接 colcon build / ros2 run，这里用等价的自定义消息替代，
# 同时保留标准 Header 以便 message_filters 做时间同步。
std_msgs/Header header
string format            # 编码格式，默认 "pcm_s16le"
uint32 sample_rate       # 采样率 Hz，默认 16000
uint8[] data             # 原始音频字节
