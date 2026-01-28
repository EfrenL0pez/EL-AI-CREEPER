#!/bin/bash
# El AI Creeper - Interactive Setup
# Run: curl -fsSL https://raw.githubusercontent.com/EfrenL0pez/EL-AI-CREEPER/main/install.sh | bash

# =============================================================================
# COLORS AND STYLING
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
print_header() {
    clear
    echo -e "${GREEN}"
    echo "  ███████╗██╗          █████╗ ██╗     ██████╗██████╗ ███████╗███████╗██████╗ ███████╗██████╗ "
    echo "  ██╔════╝██║         ██╔══██╗██║    ██╔════╝██╔══██╗██╔════╝██╔════╝██╔══██╗██╔════╝██╔══██╗"
    echo "  █████╗  ██║         ███████║██║    ██║     ██████╔╝█████╗  █████╗  ██████╔╝█████╗  ██████╔╝"
    echo "  ██╔══╝  ██║         ██╔══██║██║    ██║     ██╔══██╗██╔══╝  ██╔══╝  ██╔═══╝ ██╔══╝  ██╔══██╗"
    echo "  ███████╗███████╗    ██║  ██║██║    ╚██████╗██║  ██║███████╗███████╗██║     ███████╗██║  ██║"
    echo "  ╚══════╝╚══════╝    ╚═╝  ╚═╝╚═╝     ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝"
    echo -e "${NC}"
    echo -e "${CYAN}  Interactive Setup Guide${NC}"
    echo ""
}

print_step() {
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${WHITE}${BOLD}  STEP $1 OF 6: $2${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}  ✓ $1${NC}"
}

print_error() {
    echo -e "${RED}  ✗ $1${NC}"
}

print_info() {
    echo -e "${CYAN}  ℹ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}  ⚠ $1${NC}"
}

print_tip() {
    echo -e "${PURPLE}  💡 TIP: $1${NC}"
}

wait_for_enter() {
    echo ""
    echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} to continue...${NC}"
    read
}

ask_yes_no() {
    while true; do
        echo -ne "${WHITE}  $1 ${GREEN}[y/n]${WHITE}: ${NC}"
        read -n 1 answer
        echo ""
        case $answer in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo -e "${RED}  Please answer y or n${NC}";;
        esac
    done
}

show_progress() {
    local current=$1
    local total=6
    local width=50
    local percentage=$((current * 100 / total))
    local filled=$((current * width / total))
    local empty=$((width - filled))
    
    printf "${CYAN}  Progress: [${GREEN}"
    for ((i=0; i<filled; i++)); do printf "█"; done
    printf "${WHITE}"
    for ((i=0; i<empty; i++)); do printf "░"; done
    printf "${CYAN}] ${percentage}%%${NC}\n"
    echo ""
}

# =============================================================================
# WELCOME SCREEN
# =============================================================================
welcome_screen() {
    print_header
    echo -e "${WHITE}  Welcome to the El AI Creeper setup!${NC}"
    echo ""
    echo -e "${CYAN}  This interactive guide will walk you through:${NC}"
    echo ""
    echo -e "${WHITE}    1. ${NC}Downloading the code"
    echo -e "${WHITE}    2. ${NC}Flashing Yukon firmware"
    echo -e "${WHITE}    3. ${NC}Uploading Yukon code"
    echo -e "${WHITE}    4. ${NC}Verifying connections"
    echo -e "${WHITE}    5. ${NC}Installing Docker"
    echo -e "${WHITE}    6. ${NC}Starting the robot!"
    echo ""
    echo -e "${YELLOW}  Requirements:${NC}"
    echo -e "${WHITE}    • Raspberry Pi (with internet)${NC}"
    echo -e "${WHITE}    • Pimoroni Yukon${NC}"
    echo -e "${WHITE}    • USB cable (Pi to Yukon)${NC}"
    echo ""
    print_tip "Take your time - each step will wait for you!"
    wait_for_enter
}

# =============================================================================
# STEP 1: DOWNLOAD CODE
# =============================================================================
step_download() {
    print_header
    print_step "1" "DOWNLOAD CODE"
    show_progress 1
    
    echo -e "${WHITE}  Downloading El AI Creeper files from GitHub...${NC}"
    echo ""
    
    cd ~
    if [ -d "EL-AI-CREEPER" ]; then
        print_info "Found existing installation, updating..."
        cd EL-AI-CREEPER
        git pull --quiet
        if [ $? -eq 0 ]; then
            print_success "Files updated successfully!"
        else
            print_error "Update failed, trying fresh download..."
            cd ~
            rm -rf EL-AI-CREEPER
            git clone --quiet https://github.com/EfrenL0pez/EL-AI-CREEPER.git
        fi
    else
        git clone --quiet https://github.com/EfrenL0pez/EL-AI-CREEPER.git
        if [ $? -eq 0 ]; then
            print_success "Download complete!"
        else
            print_error "Download failed. Check your internet connection."
            exit 1
        fi
    fi
    
    cd ~/EL-AI-CREEPER
    echo ""
    print_success "Files downloaded to: ~/EL-AI-CREEPER"
    print_success "Yukon code is at: ~/EL-AI-CREEPER/yukon/main.py"
    echo ""
    print_tip "You can edit the config in yukon/main.py to match your slot setup"
    wait_for_enter
}

# =============================================================================
# STEP 2: FLASH YUKON FIRMWARE
# =============================================================================
step_flash_firmware() {
    print_header
    print_step "2" "FLASH YUKON FIRMWARE"
    show_progress 2
    
    echo -e "${WHITE}  Time to flash the Yukon firmware!${NC}"
    echo ""
    echo -e "${CYAN}  Follow these steps:${NC}"
    echo ""
    echo -e "${WHITE}    1. Open a web browser on your Pi${NC}"
    echo ""
    echo -e "${WHITE}    2. Go to:${NC}"
    echo -e "${GREEN}       https://github.com/pimoroni/yukon/releases/latest${NC}"
    echo ""
    echo -e "${WHITE}    3. Download the file that says:${NC}"
    echo -e "${YELLOW}       pimoroni-yukon-vX.X.X-micropython-with-filesystem.uf2${NC}"
    echo ""
    echo -e "${WHITE}    4. Connect Yukon to Pi with USB cable${NC}"
    echo ""
    echo -e "${WHITE}    5. Put Yukon in bootloader mode:${NC}"
    echo -e "${CYAN}       • Press PWR button (turn OFF - green light off)${NC}"
    echo -e "${CYAN}       • Hold BOOT button${NC}"
    echo -e "${CYAN}       • While holding BOOT, press PWR${NC}"
    echo -e "${CYAN}       • Release both buttons${NC}"
    echo ""
    echo -e "${WHITE}    6. A drive called ${YELLOW}RPI-RP2${WHITE} should appear${NC}"
    echo ""
    echo -e "${WHITE}    7. Drag the .uf2 file onto the RPI-RP2 drive${NC}"
    echo ""
    echo -e "${WHITE}    8. Wait for it to finish (LEDs will flash)${NC}"
    echo ""
    print_tip "The 'with-filesystem' version is larger but includes everything you need"
    echo ""
    
    if ask_yes_no "Did you successfully flash the firmware?"; then
        print_success "Firmware flashed!"
    else
        echo ""
        print_warning "Troubleshooting:"
        echo -e "${WHITE}    • Make sure Yukon is in bootloader mode (dim A/B LEDs)${NC}"
        echo -e "${WHITE}    • Try a different USB cable${NC}"
        echo -e "${WHITE}    • Try a different USB port${NC}"
        echo ""
        echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} when ready to continue anyway...${NC}"
        read
    fi
    wait_for_enter
}

# =============================================================================
# STEP 3: UPLOAD YUKON CODE
# =============================================================================
step_upload_code() {
    print_header
    print_step "3" "UPLOAD YUKON CODE"
    show_progress 3
    
    echo -e "${WHITE}  Now let's upload the control code to Yukon!${NC}"
    echo ""
    echo -e "${CYAN}  Follow these steps:${NC}"
    echo ""
    echo -e "${WHITE}    1. Open ${YELLOW}Thonny${WHITE} on your Pi${NC}"
    echo -e "${CYAN}       (Menu > Programming > Thonny)${NC}"
    echo ""
    echo -e "${WHITE}    2. In Thonny, look at the ${YELLOW}bottom right corner${NC}"
    echo -e "${WHITE}       Click it and select:${NC}"
    echo -e "${GREEN}       MicroPython (Raspberry Pi Pico)${NC}"
    echo -e "${CYAN}       or${NC}"
    echo -e "${GREEN}       MicroPython (USB)${NC}"
    echo ""
    echo -e "${WHITE}    3. Go to ${YELLOW}File > Open${NC}"
    echo ""
    echo -e "${WHITE}    4. Navigate to:${NC}"
    echo -e "${GREEN}       ~/EL-AI-CREEPER/yukon/main.py${NC}"
    echo ""
    echo -e "${WHITE}    5. Click ${YELLOW}File > Save as${NC}"
    echo ""
    echo -e "${WHITE}    6. Choose ${YELLOW}MicroPython device${NC}"
    echo ""
    echo -e "${WHITE}    7. Save as: ${GREEN}main.py${NC}"
    echo ""
    print_tip "Make sure to save TO the Yukon, not to your Pi!"
    echo ""
    print_warning "Want to change motor/servo slots? Edit the CONFIG section at the top!"
    echo ""
    
    if ask_yes_no "Did you successfully upload the code to Yukon?"; then
        print_success "Yukon code uploaded!"
    else
        echo ""
        print_warning "Troubleshooting:"
        echo -e "${WHITE}    • Make sure USB cable is connected${NC}"
        echo -e "${WHITE}    • Try unplugging and replugging Yukon${NC}"
        echo -e "${WHITE}    • Make sure you selected MicroPython interpreter${NC}"
        echo ""
        echo -e "${WHITE}  Press ${GREEN}[ENTER]${WHITE} when ready to continue anyway...${NC}"
        read
    fi
    wait_for_enter
}

# =============================================================================
# STEP 4: VERIFY CONNECTION
# =============================================================================
step_verify() {
    print_header
    print_step "4" "VERIFY CONNECTION"
    show_progress 4
    
    echo -e "${WHITE}  Checking if Yukon is connected...${NC}"
    echo ""
    
    sleep 1
    
    if [ -e /dev/ttyACM0 ]; then
        print_success "Yukon detected on /dev/ttyACM0"
        YUKON_PORT="/dev/ttyACM0"
    elif [ -e /dev/ttyACM1 ]; then
        print_success "Yukon detected on /dev/ttyACM1"
        YUKON_PORT="/dev/ttyACM1"
        print_warning "Note: Your Yukon is on ttyACM1, not ttyACM0"
        print_info "You may need to edit ~/EL-AI-CREEPER/src/charged_creeper/nodes/communication_node.py"
        print_info "Change SERIAL_PORT = '/dev/ttyACM0' to SERIAL_PORT = '/dev/ttyACM1'"
    elif [ -e /dev/ttyACM2 ]; then
        print_success "Yukon detected on /dev/ttyACM2"
        YUKON_PORT="/dev/ttyACM2"
        print_warning "Note: Your Yukon is on ttyACM2"
        print_info "Edit the SERIAL_PORT in communication_node.py"
    else
        print_error "No Yukon detected!"
        echo ""
        echo -e "${WHITE}  Available serial ports:${NC}"
        ls /dev/ttyACM* 2>/dev/null || echo -e "${RED}    None found${NC}"
        ls /dev/ttyUSB* 2>/dev/null
        echo ""
        print_warning "Troubleshooting:"
        echo -e "${WHITE}    • Make sure Yukon is connected via USB${NC}"
        echo -e "${WHITE}    • Make sure Yukon has power (green LED on)${NC}"
        echo -e "${WHITE}    • Try unplugging and replugging${NC}"
        echo ""
    fi
    
    wait_for_enter
}

# =============================================================================
# STEP 5: INSTALL DOCKER
# =============================================================================
step_docker() {
    print_header
    print_step "5" "INSTALL DOCKER"
    show_progress 5
    
    echo -e "${WHITE}  Checking Docker installation...${NC}"
    echo ""
    
    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        print_warning "Docker is not installed"
        echo ""
        echo -e "${WHITE}  Installing Docker now...${NC}"
        echo -e "${CYAN}  (This may take a few minutes)${NC}"
        echo ""
        
        curl -fsSL https://get.docker.com | sh
        sudo usermod -aG docker $USER
        
        if [ $? -eq 0 ]; then
            print_success "Docker installed successfully!"
            echo ""
            print_warning "IMPORTANT: You need to reboot for Docker to work!"
            echo ""
            echo -e "${WHITE}  After rebooting, run this command again:${NC}"
            echo -e "${GREEN}  curl -fsSL https://raw.githubusercontent.com/EfrenL0pez/EL-AI-CREEPER/main/install.sh | bash${NC}"
            echo ""
            
            if ask_yes_no "Reboot now?"; then
                echo -e "${WHITE}  Rebooting in 3 seconds...${NC}"
                sleep 3
                sudo reboot
            else
                echo ""
                print_info "Run 'sudo reboot' when you're ready, then run this script again"
                exit 0
            fi
        else
            print_error "Docker installation failed"
            exit 1
        fi
    fi
    
    # Check if user is in docker group
    if ! groups | grep -q docker; then
        print_warning "Docker is installed but you need to reboot"
        echo ""
        
        if ask_yes_no "Reboot now?"; then
            echo -e "${WHITE}  Rebooting in 3 seconds...${NC}"
            sleep 3
            sudo reboot
        else
            echo ""
            print_info "Run 'sudo reboot' when you're ready, then run this script again"
            exit 0
        fi
    fi
    
    print_success "Docker is installed and ready!"
    wait_for_enter
}

# =============================================================================
# STEP 6: START THE ROBOT
# =============================================================================
step_start() {
    print_header
    print_step "6" "START THE ROBOT"
    show_progress 6
    
    echo -e "${WHITE}  Almost there! Let's start El AI Creeper!${NC}"
    echo ""
    echo -e "${CYAN}  ╔════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}  ║           ${WHITE}KEYBOARD CONTROLS${CYAN}            ║${NC}"
    echo -e "${CYAN}  ╠════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}  ║                                        ║${NC}"
    echo -e "${CYAN}  ║      ${YELLOW}W${CYAN}          Forward               ║${NC}"
    echo -e "${CYAN}  ║    ${YELLOW}A   D${CYAN}        Steer Left / Right     ║${NC}"
    echo -e "${CYAN}  ║      ${YELLOW}S${CYAN}          Backward              ║${NC}"
    echo -e "${CYAN}  ║                                        ║${NC}"
    echo -e "${CYAN}  ║    ${YELLOW}Q   E${CYAN}        Head Left / Right      ║${NC}"
    echo -e "${CYAN}  ║                                        ║${NC}"
    echo -e "${CYAN}  ║    ${YELLOW}SPACE${CYAN}        Stop All               ║${NC}"
    echo -e "${CYAN}  ║    ${YELLOW}Ctrl+C${CYAN}       Quit                   ║${NC}"
    echo -e "${CYAN}  ║                                        ║${NC}"
    echo -e "${CYAN}  ╚════════════════════════════════════════╝${NC}"
    echo ""
    print_tip "The first build takes a few minutes. Be patient!"
    echo ""
    
    if ask_yes_no "Ready to start?"; then
        echo ""
        print_success "Starting El AI Creeper..."
        echo ""
        echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo ""
        
        cd ~/EL-AI-CREEPER
        docker compose up --build
    else
        echo ""
        print_info "To start later, run:"
        echo -e "${GREEN}  cd ~/EL-AI-CREEPER && docker compose up --build${NC}"
        echo ""
    fi
}

# =============================================================================
# MAIN SCRIPT
# =============================================================================

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ] || ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo ""
    echo -e "${YELLOW}  Warning: This doesn't look like a Raspberry Pi${NC}"
    echo -e "${WHITE}  This script is designed for Raspberry Pi.${NC}"
    echo ""
    echo -ne "  Continue anyway? [y/n]: "
    read -n 1 answer
    echo ""
    if [ "$answer" != "y" ]; then
        exit 0
    fi
fi

# Run all steps
welcome_screen
step_download
step_flash_firmware
step_upload_code
step_verify
step_docker
step_start

# Done!
echo ""
print_header
echo -e "${GREEN}  ╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}  ║                                        ║${NC}"
echo -e "${GREEN}  ║      ${WHITE}SETUP COMPLETE!${GREEN}                  ║${NC}"
echo -e "${GREEN}  ║                                        ║${NC}"
echo -e "${GREEN}  ║   ${WHITE}Thanks for using El AI Creeper!${GREEN}      ║${NC}"
echo -e "${GREEN}  ║                                        ║${NC}"
echo -e "${GREEN}  ║   ${CYAN}GitHub: EfrenL0pez/EL-AI-CREEPER${GREEN}     ║${NC}"
echo -e "${GREEN}  ║                                        ║${NC}"
echo -e "${GREEN}  ╚════════════════════════════════════════╝${NC}"
echo ""