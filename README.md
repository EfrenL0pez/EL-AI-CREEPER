# El AI Creeper

Control your robot with a keyboard over SSH.

## Quick Start

Run this on your Raspberry Pi:

```
curl -fsSL https://raw.githubusercontent.com/EfrenL0pez/EL-AI-CREEPER/main/install.sh | bash
```

First time? Log out and back in when it asks, then run the command again.

## Controls

- W - Forward
- S - Reverse
- A - Left (steering + head)
- D - Right (steering + head)
- Space - Stop
- X - Quit

## Requirements

- Raspberry Pi 4 or 5
- Pimoroni Yukon (USB connection)
- Motors/servos on Yukon

## Manual Setup

If you want to do it step by step:

```
# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in

# Clone and run
git clone https://github.com/EfrenL0pez/EL-AI-CREEPER.git
cd EL-AI-CREEPER/ros2_ws
docker compose up --build
```

## Configuration

If the robot doesn't drive straight, edit:
`src/charged_creeper/nodes/communication_node.py`

Change `STEERING_TRIM` value (positive = right, negative = left)

## License

MIT
