#!/bin/bash
# El AI Creeper - Install Script
# Run: curl -fsSL https://raw.githubusercontent.com/EfrenL0pez/EL-AI-CREEPER/main/install.sh | bash

echo "El AI Creeper - Setup"
echo "====================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
    echo ""
    echo "Docker installed."
    echo "Log out and log back in, then run this again:"
    echo ""
    echo "  curl -fsSL https://raw.githubusercontent.com/EfrenL0pez/EL-AI-CREEPER/main/install.sh | bash"
    echo ""
    exit 0
fi

# Clone the repo
echo "Downloading..."
cd ~
if [ -d "EL-AI-CREEPER" ]; then
    echo "Updating existing files..."
    cd EL-AI-CREEPER
    git pull
else
    git clone https://github.com/EfrenL0pez/EL-AI-CREEPER.git
    cd EL-AI-CREEPER
fi

# Start it up
echo "Starting..."
cd ros2_ws
docker compose up --build
