import json
from pathlib import Path

def load_pid_config():
    config_path = Path("config/pid_config.json")
    with open(config_path, "r") as f:
        return json.load(f)