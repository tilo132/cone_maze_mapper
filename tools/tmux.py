import sys
import os

def generate_env_args():
    args = [
        "ROS_DOMAIN_ID",
        "ROS_DISCOVERY_SERVER",
        "FASTRTPS_DEFAULT_PROFILES_FILE"
    ]
    msg = ""
    for arg in args:
        _arg = os.getenv(arg)
        if _arg is None:
            continue
        msg += f"\n  send-keys 'export {arg}={_arg}' C-m \; \\"
    return msg

bash = f"""
#!/bin/sh
tmux new-session -s tmux \; \{generate_env_args()}
  split-window -v \; \
  split-window -h \; \
  select-pane -t 1 \; \
  select-pane -t 0 \; \
  split-window -h \;
tmux attach-session -t tmux
"""

os.system(bash)
exit()
