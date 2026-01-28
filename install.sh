#!/bin/bash
# El AI Creeper - Setup Script

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m'

print_logo() {
    clear
    echo -e "${GREEN}"
    echo " ███████╗██╗      █████╗ ██╗ ██████╗██████╗ ███████╗███████╗██████╗ ███████╗██████╗ ██╗"
    echo " ██╔════╝██║     ██╔══██╗██║██╔════╝██╔══██╗██╔════╝██╔════╝██╔══██╗██╔════╝██╔══██╗██║"
    echo " █████╗  ██║     ███████║██║██║     ██████╔╝█████╗  █████╗  ██████╔╝█████╗  ██████╔╝██║"
    echo " ██╔══╝  ██║     ██╔══██║██║██║     ██╔══██╗██╔══╝  ██╔══╝  ██╔═══╝ ██╔══╝  ██╔══██╗╚═╝"
    echo " ███████╗███████╗██║  ██║██║╚██████╗██║  ██║███████╗███████╗██║     ███████╗██║  ██║██╗"
    echo " ╚══════╝╚══════╝╚═╝  ╚═╝╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝"
    echo -e "${NC}"
    echo ""
}

# =============================================================================
# WELCOME
# =============================================================================
print_logo
echo -e "${CYAN}  Setup Guide${NC}"
echo ""
echo -e "${WHITE}    1. Yukon Setup${NC}"
echo -e "${WHITE}    2. Docker Install${NC}"
echo -e "${WHITE}    3. Start Robot${NC}"
echo ""
echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} to begin...${NC}"
read </dev/tty

# =============================================================================
# STEP 1: YUKON SETUP
# =============================================================================
print_logo
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${WHITE}  STEP 1: YUKON SETUP${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Download code
echo -e "${WHITE}  Downloading code...${NC}"
cd ~
if [ -d "EL-AI-CREEPER" ]; then
    cd EL-AI-CREEPER && git pull --quiet
else
    git clone --quiet https://github.com/EfrenL0pez/EL-AI-CREEPER.git
    cd EL-AI-CREEPER
fi
echo -e "${GREEN}  ✓ Code downloaded${NC}"
echo ""

echo -e "${CYAN}  Flash firmware:${NC}"
echo -e "${WHITE}    1. Go to: ${GREEN}github.com/pimoroni/yukon/releases/latest${NC}"
echo -e "${WHITE}    2. Download ${YELLOW}.uf2${WHITE} file (with-filesystem)${NC}"
echo -e "${WHITE}    3. Hold BOOT + press PWR on Yukon${NC}"
echo -e "${WHITE}    4. Drag .uf2 to RPI-RP2 drive${NC}"
echo ""
echo -e "${CYAN}  Upload code:${NC}"
echo -e "${WHITE}    1. Open Thonny${NC}"
echo -e "${WHITE}    2. Open ${GREEN}~/EL-AI-CREEPER/yukon/main.py${NC}"
echo -e "${WHITE}    3. Save to Yukon as ${GREEN}main.py${NC}"
echo ""

echo -ne "${WHITE}  Yukon ready? ${GREEN}[y/n]${WHITE}: ${NC}"
read -n 1 answer </dev/tty
echo ""
if [ "$answer" != "y" ]; then
    echo -e "${YELLOW}  Complete Yukon setup, then run this script again.${NC}"
    exit 0
fi

echo ""
echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} to continue...${NC}"
read </dev/tty

# =============================================================================
# STEP 2: DOCKER
# =============================================================================
print_logo
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${WHITE}  STEP 2: DOCKER INSTALL${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if ! command -v docker &> /dev/null; then
    echo -e "${WHITE}  Installing Docker...${NC}"
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
    echo ""
    echo -e "${GREEN}  ✓ Docker installed${NC}"
    echo ""
    echo -e "${YELLOW}  Reboot required. Run this script again after reboot.${NC}"
    echo ""
    echo -ne "${WHITE}  Reboot now? ${GREEN}[y/n]${WHITE}: ${NC}"
    read -n 1 answer </dev/tty
    echo ""
    if [ "$answer" = "y" ]; then
        sudo reboot
    fi
    exit 0
fi

if ! groups | grep -q docker; then
    echo -e "${YELLOW}  Reboot required for Docker permissions.${NC}"
    echo ""
    echo -ne "${WHITE}  Reboot now? ${GREEN}[y/n]${WHITE}: ${NC}"
    read -n 1 answer </dev/tty
    echo ""
    if [ "$answer" = "y" ]; then
        sudo reboot
    fi
    exit 0
fi

echo -e "${GREEN}  ✓ Docker ready${NC}"
echo ""
echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} to continue...${NC}"
read </dev/tty

# =============================================================================
# STEP 3: START ROBOT
# =============================================================================
print_logo
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${WHITE}  STEP 3: START ROBOT${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo -e "${CYAN}  Controls:${NC}"
echo -e "${WHITE}    W/S  = Forward/Back${NC}"
echo -e "${WHITE}    A/D  = Steer${NC}"
echo -e "${WHITE}    Q/E  = Head${NC}"
echo -e "${WHITE}    SPACE = Stop${NC}"
echo ""

echo -ne "${WHITE}  Start? ${GREEN}[y/n]${WHITE}: ${NC}"
read -n 1 answer </dev/tty
echo ""

if [ "$answer" = "y" ]; then
    echo ""
    echo -e "${GREEN}  Starting...${NC}"
    echo ""
    cd ~/EL-AI-CREEPER
    docker compose up --build
fi