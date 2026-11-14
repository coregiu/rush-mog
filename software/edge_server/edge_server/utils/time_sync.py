from message_filters import ApproximateTimeSynchronizer


def make_approximate_synchronizer(subscribers, queue_size: int = 10, slop: float = 0.05):
    """为多路传感器话题创建近似时间同步器。

    DESIGN.md 5.2 建议使用 message_filters；图像、雷达、音频、里程计、
    关节状态来自不同传感器，无法保证时间戳严格相等，因此使用近似同步。
    """
    subscribers = list(subscribers)
    if not subscribers:
        raise ValueError("at least one subscriber is required")
    queue_size = max(int(queue_size), len(subscribers))
    slop = max(float(slop), 1e-3)
    return ApproximateTimeSynchronizer(subscribers, queue_size=queue_size, slop=slop)
