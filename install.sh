#!/bin/bash
# El AI Creeper - Setup & Run Script

VERSION="1.0.0"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
RED='\033[0;31m'
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
    echo -e "${WHITE}                              Version ${GREEN}${VERSION}${NC}"
    echo ""
}

check_docker() {
    command -v docker &> /dev/null
}

check_docker_group() {
    groups | grep -q docker
}

check_container() {
    docker images 2>/dev/null | grep -q "el-ai-creeper"
}

# =============================================================================
# INSTALL
# =============================================================================
do_install() {

    # Download/update code
    print_logo
    echo -e "${WHITE}  Updating code from GitHub...${NC}"
    cd ~
    if [ -d "EL-AI-CREEPER" ]; then
        cd EL-AI-CREEPER
        git stash --quiet 2>/dev/null
        git pull --quiet
    else
        git clone https://github.com/EfrenL0pez/EL-AI-CREEPER.git
        cd EL-AI-CREEPER
    fi
    echo -e "${GREEN}  ✓ Code updated${NC}"
    sleep 1

    # Step 1: Yukon
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  STEP 1: YUKON SETUP${NC}"
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

    # Step 2: Docker
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  STEP 2: DOCKER${NC}"
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
                *) echo -e "${RED}  Enter y or n${NC}" ;;
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
                *) echo -e "${RED}  Enter y or n${NC}" ;;
            esac
        done
    fi

    echo -e "${GREEN}  ✓ Docker ready${NC}"
    echo ""

    # Step 3: Build
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  STEP 3: BUILD CONTAINER${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${WHITE}  Building ROS2 container (takes a few minutes)...${NC}"
    echo ""
    
    cd ~/EL-AI-CREEPER
    if ! docker compose build; then
        echo ""
        echo -e "${RED}  Build failed! Try:${NC}"
        echo -e "${WHITE}    docker builder prune -f${NC}"
        echo -e "${WHITE}    bash install.sh${NC}"
        return 1
    fi
    
    echo ""
    echo -e "${GREEN}  ✓ Build complete!${NC}"
    echo ""
    echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} to continue...${NC}"
    read </dev/tty
    return 0
}

# =============================================================================
# RUN
# =============================================================================
do_run() {
    print_logo
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}  START ROBOT${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${CYAN}  Controls:${NC}"
    echo -e "${WHITE}    W = Forward    S = Backward${NC}"
    echo -e "${WHITE}    A = Left       D = Right${NC}"
    echo -e "${WHITE}    SPACE = Stop   Q = Quit${NC}"
    echo ""
    echo -e "${WHITE}  Starting...${NC}"
    
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
}

# =============================================================================
# MAIN MENU
# =============================================================================
while true; do
    print_logo
    echo -e "${WHITE}  What would you like to do?${NC}"
    echo ""
    echo -e "    ${GREEN}1${NC}) Install  ${WHITE}(first time setup)${NC}"
    echo -e "    ${GREEN}2${NC}) Run      ${WHITE}(start robot)${NC}"
    echo -e "    ${GREEN}3${NC}) Rebuild  ${WHITE}(rebuild container)${NC}"
    echo -e "    ${GREEN}4${NC}) Exit${NC}"
    echo ""
    echo -ne "${WHITE}  Choice ${GREEN}[1-4]${WHITE}: ${NC}"
    read -n 1 choice </dev/tty
    echo ""
    
    case $choice in
        1) do_install ;;
        2) 
            if ! check_container; then
                echo -e "${RED}  Not installed. Select 1 first.${NC}"
                sleep 2
            else
                do_run
            fi
            ;;
        3)
            cd ~/EL-AI-CREEPER
            docker compose build --no-cache
            echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE}...${NC}"
            read </dev/tty
            ;;
        4)
            echo -e "${GREEN}  Bye!${NC}"
            exit 0
            ;;
        *) 
            echo -e "${RED}  Invalid. Enter 1-4.${NC}"
            sleep 1
            ;;
    esac
done
