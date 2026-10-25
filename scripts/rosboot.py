"""让脚本可以脱离事先 source 的环境直接运行。

用法（放在其它 import 之前）：
    from rosboot import ensure_ros
    ensure_ros()

原理：如果 import px4_msgs 失败，就用 bash 重新 source ROS 2 和工作区环境，
再 exec 回当前脚本。加一层判断是为了避免无限自我重启。
"""
import os
import shlex
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def ensure_ros() -> None:
    try:
        import px4_msgs  # noqa: F401
        return
    except ImportError:
        pass

    if os.environ.get('_ROSBOOT_DONE') == '1':
        sys.exit('错误：source 之后仍然找不到 px4_msgs，请检查 install/setup.bash')

    distro = os.environ.get('ROS_DISTRO', 'humble')
    args = ' '.join(shlex.quote(a) for a in [sys.executable, *sys.argv])
    script = (
        f'source /opt/ros/{distro}/setup.bash && '
        f'source {shlex.quote(ROOT)}/install/setup.bash && '
        f'_ROSBOOT_DONE=1 exec {args}'
    )
    os.execvp('bash', ['bash', '-c', script])
