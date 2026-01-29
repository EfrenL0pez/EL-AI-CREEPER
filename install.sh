#!/bin/bash
# El AI Creeper - Setup & Run Script

VERSION="1.0.0"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
RED='\033[0;31m'
NC='\033[0m'

# Clone repo if not exists
if [ ! -d ~/EL-AI-CREEPER ]; then
    echo -e "Downloading El AI Creeper..."
    git clone --quiet https://github.com/EfrenL0pez/EL-AI-CREEPER.git ~/EL-AI-CREEPER
fi
cd ~/EL-AI-CREEPER

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
    echo -e "${WHITE}                              Version ${GREEN}${VERSION}${NC}"
    echo ""
}

check_docker() {
    command -v docker &> /dev/null
}

check_docker_group() {
    groups | grep -q docker
}

check_installed() {
    docker images 2>/dev/null | grep -q "el-ai-creeper"
}

first_time_setup() {
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  YUKON SETUP${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${CYAN}  Flash firmware (one time only):${NC}"
    echo -e "${WHITE}    1. Download: ${GREEN}github.com/pimoroni/yukon/releases/latest${NC}"
    echo -e "${WHITE}    2. Hold ${YELLOW}BOOT${WHITE} + press ${YELLOW}PWR${WHITE} on Yukon${NC}"
    echo -e "${WHITE}    3. Drag .uf2 to RPI-RP2 drive${NC}"
    echo ""
    echo -e "${CYAN}  Upload code:${NC}"
    echo -e "${WHITE}    1. Open ${GREEN}Thonny${NC}"
    echo -e "${WHITE}    2. Open ${GREEN}~/EL-AI-CREEPER/yukon/main.py${NC}"
    echo -e "${WHITE}    3. Save to Yukon as ${GREEN}main.py${NC}"
    echo ""
    
    while true; do
        echo -ne "${WHITE}  Yukon ready? ${GREEN}[y/n]${WHITE}: ${NC}"
        read -n 1 answer </dev/tty
        echo ""
        case $answer in
            y|Y) break ;;
            n|N) echo -e "${YELLOW}  Run this script again when ready.${NC}"; return 1 ;;
            *) echo -e "${RED}  Enter y or n${NC}" ;;
        esac
    done

    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  DOCKER${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""

    if ! check_docker; then
        echo -e "${WHITE}  Installing Docker...${NC}"
        curl -fsSL https://get.docker.com | sh
        sudo usermod -aG docker $USER
        echo -e "${GREEN}  ✓ Docker installed${NC}"
        echo -e "${YELLOW}  Reboot required. Run script again after.${NC}"
        echo ""
        while true; do
            echo -ne "${WHITE}  Reboot now? ${GREEN}[y/n]${WHITE}: ${NC}"
            read -n 1 answer </dev/tty
            echo ""
            case $answer in
                y|Y) sudo reboot ;;
                n|N) exit 0 ;;
            esac
        done
    fi

    if ! check_docker_group; then
        echo -e "${YELLOW}  Reboot required for Docker permissions.${NC}"
        while true; do
            echo -ne "${WHITE}  Reboot now? ${GREEN}[y/n]${WHITE}: ${NC}"
            read -n 1 answer </dev/tty
            echo ""
            case $answer in
                y|Y) sudo reboot ;;
                n|N) exit 0 ;;
            esac
        done
    fi

    echo -e "${GREEN}  ✓ Docker ready${NC}"
    do_update
}

do_update() {
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  UPDATING${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    
    echo -e "${WHITE}  Downloading latest code...${NC}"
    cd ~/EL-AI-CREEPER
    git stash --quiet 2>/dev/null
    git pull --quiet
    echo -e "${GREEN}  ✓ Code updated${NC}"
    echo ""
    
    echo -e "${WHITE}  Building container (takes a few minutes)...${NC}"
    echo ""
    
    if ! docker compose build; then
        echo ""
        echo -e "${RED}  Build failed!${NC}"
        echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE}...${NC}"
        read </dev/tty
        return 1
    fi
    
    echo ""
    echo -e "${GREEN}  ✓ Update complete!${NC}"
    echo ""
    echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE}...${NC}"
    read </dev/tty
    return 0
}

do_start() {
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  STARTING ROBOT${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${CYAN}  Controls:${NC}"
    echo -e "${WHITE}    W = Forward    S = Backward${NC}"
    echo -e "${WHITE}    A = Left       D = Right${NC}"
    echo -e "${WHITE}    SPACE = Stop   Q = Quit${NC}"
    echo ""
    
    cd ~/EL-AI-CREEPER
    docker compose down 2>/dev/null
    docker compose up -d
    sleep 3
    
    echo -e "${GREEN}  ✓ Ready!${NC}"
    echo ""
    
    docker exec -it ros2_container /bin/bash -c '
        source /opt/ros/kilted/setup.bash
        source /root/ros2_ws/install/setup.bash
        ros2 run charged_creeper keyboard_node
    '
    
    echo ""
    echo -e "${WHITE}  Stopping...${NC}"
    docker compose down 2>/dev/null
    echo -e "${GREEN}  ✓ Stopped${NC}"
    sleep 1
}

do_exit() {
    cd ~/EL-AI-CREEPER 2>/dev/null
    docker compose down 2>/dev/null
    echo -e "${GREEN}  Bye!${NC}"
    exit 0
}

while true; do
    print_logo
    
    if ! check_installed; then
        echo -e "${WHITE}  First time? Run Install first.${NC}"
        echo ""
        echo -e "    ${GREEN}1${NC}) Install${NC}"
        echo -e "    ${GREEN}2${NC}) Exit${NC}"
        echo ""
        echo -ne "${WHITE}  Choice ${GREEN}[1-2]${WHITE}: ${NC}"
        read -n 1 choice </dev/tty
        echo ""
        
        case $choice in
            1) first_time_setup ;;
            2) do_exit ;;
            *) echo -e "${RED}  Invalid.${NC}"; sleep 1 ;;
        esac
    else
        echo -e "${WHITE}  What would you like to do?${NC}"
        echo ""
        echo -e "    ${GREEN}1${NC}) Start    ${WHITE}(run the robot)${NC}"
        echo -e "    ${GREEN}2${NC}) Update   ${WHITE}(download latest code)${NC}"
        echo -e "    ${GREEN}3${NC}) Exit${NC}"
        echo ""
        echo -ne "${WHITE}  Choice ${GREEN}[1-3]${WHITE}: ${NC}"
        read -n 1 choice </dev/tty
        echo ""
        
        case $choice in
            1) do_start ;;
            2) do_update ;;
            3) do_exit ;;
            *) echo -e "${RED}  Invalid.${NC}"; sleep 1 ;;
        esac
    fi
done
