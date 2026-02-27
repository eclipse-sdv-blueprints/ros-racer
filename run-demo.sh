#!/bin/bash
# =============================================================================
# Eclipse Muto OTA Demo - Interactive Script
# =============================================================================
# This script walks through the complete Muto OTA demonstration step by step.
# Each step shows what's happening and waits for user confirmation.
#
# Demonstrates TWO deployment methods:
#   1. Direct ROS topic deployment (Muto native)
#   2. Eclipse Symphony orchestration (cloud-native)
#
# Usage:
#   ./run-demo.sh                                    # Default: 3 agents
#   NUM_AGENTS=10 ./run-demo.sh                      # 10 agents
#   NUM_AGENTS=20 ./run-demo.sh                      # 20 agents
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color
BOLD='\033[1m'
DIM='\033[2m'

# Script directory (for relative paths)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACKS_DIR="$SCRIPT_DIR/demo/stacks"
DEPLOY_SCRIPT="$SCRIPT_DIR/demo/scripts/deploy-stack.py"
SYMPHONY_DIR="$SCRIPT_DIR/demo/symphony"
SYMPHONY_INFRA_DIR="$SCRIPT_DIR/demo/symphony"

# Port configuration (using 10000 offset to avoid common port conflicts)
PORT_NOVNC=18080
PORT_SYMPHONY_API=8082
PORT_SYMPHONY_PORTAL=13000
PORT_MQTT=1883

# Fleet configuration (can be overridden via environment)
NUM_AGENTS="${NUM_AGENTS:-3}"
DEPLOY_STAGGER="${DEPLOY_STAGGER:-0.5}"
DEPLOY_DISCOVERY_WAIT="${DEPLOY_DISCOVERY_WAIT:-3.0}"

# Symphony API URL
SYMPHONY_API_URL="http://localhost:${PORT_SYMPHONY_API}/v1alpha2/"

# Container runtime (docker or podman)
CONTAINER_RUNTIME=""

# Detect container runtime - if both available, let user choose
detect_container_runtime() {
    local has_docker=false
    local has_podman=false
    
    if command -v docker &> /dev/null; then
        has_docker=true
    fi
    
    if command -v podman &> /dev/null; then
        has_podman=true
    fi
    
    # Both available - let user choose
    if $has_docker && $has_podman; then
        echo ""
        echo -e "${YELLOW}Both Docker and Podman are available.${NC}"
        echo -e "${WHITE}Which container runtime would you like to use?${NC}"
        echo ""
        echo -e "  ${GREEN}1)${NC} Docker"
        echo -e "  ${GREEN}2)${NC} Podman"
        echo ""
        echo -ne "${WHITE}Enter your choice [1-2] (default: 1): ${NC}"
        read -r choice
        
        case "$choice" in
            2)
                CONTAINER_RUNTIME="podman"
                ;;
            *)
                CONTAINER_RUNTIME="docker"
                ;;
        esac
        return 0
    # Only docker available
    elif $has_docker; then
        CONTAINER_RUNTIME="docker"
        return 0
    # Only podman available
    elif $has_podman; then
        CONTAINER_RUNTIME="podman"
        return 0
    # Neither available
    else
        return 1
    fi
}

# Portable sed in-place edit (works on both macOS and Linux)
sed_inplace() {
    local pattern="$1"
    local file="$2"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        sed -i '' "$pattern" "$file"
    else
        sed -i "$pattern" "$file"
    fi
}

# Track current step
STEP=0

# =============================================================================
# Helper Functions
# =============================================================================

print_header() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
}

print_subheader() {
    echo ""
    echo -e "${MAGENTA}───────────────────────────────────────────────────────────────────────────────${NC}"
    echo -e "${MAGENTA}  $1${NC}"
    echo -e "${MAGENTA}───────────────────────────────────────────────────────────────────────────────${NC}"
    echo ""
}

print_step() {
    STEP=$((STEP + 1))
    echo ""
    echo -e "${YELLOW}┌─────────────────────────────────────────────────────────────────────────────┐${NC}"
    echo -e "${YELLOW}│${NC} ${BOLD}STEP $STEP:${NC} $1"
    echo -e "${YELLOW}└─────────────────────────────────────────────────────────────────────────────┘${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ${NC}  $1"
}

print_success() {
    echo -e "${GREEN}✓${NC}  $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC}  $1"
}

print_error() {
    echo -e "${RED}✗${NC}  $1"
}

print_action() {
    echo -e "${MAGENTA}▶${NC}  ${BOLD}$1${NC}"
}

print_narration() {
    echo ""
    echo -e "${DIM}$1${NC}"
    echo ""
}

wait_for_enter() {
    echo ""
    echo -e "${WHITE}Press ${GREEN}[ENTER]${WHITE} to continue...${NC}"
    read -r
}

wait_with_message() {
    echo ""
    echo -e "${WHITE}$1 Press ${GREEN}[ENTER]${WHITE} when ready...${NC}"
    read -r
}

countdown() {
    local seconds=$1
    local message=$2
    echo ""
    for ((i=seconds; i>0; i--)); do
        echo -ne "\r${CYAN}$message ($i seconds remaining)...${NC}    "
        sleep 1
    done
    echo -ne "\r${GREEN}$message - Done!${NC}                          \n"
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        print_error "Required command not found: $1"
        exit 1
    fi
}

# Generate artifact tar.gz files and update checksums
generate_artifacts() {
    print_info "Generating artifact archives..."

    local VARIANTS_DIR="$SCRIPT_DIR/demo/variants"
    local ARTIFACTS_DIR="$SCRIPT_DIR/demo/artifacts"

    mkdir -p "$ARTIFACTS_DIR"

    for variant in conservative balanced aggressive broken; do
        local tar_name="gap_follower_${variant}.tar.gz"

        # Create tar.gz from variant directory
        tar -czf "$ARTIFACTS_DIR/$tar_name" -C "$VARIANTS_DIR/$variant" .

        # Calculate checksum
        local checksum=$(sha256sum "$ARTIFACTS_DIR/$tar_name" | cut -d' ' -f1)

        # Update demo/stacks JSON
        local stacks_json="$STACKS_DIR/gap_follower_${variant}.json"
        if [ -f "$stacks_json" ]; then
            sed_inplace "s/\"checksum\": \"[a-f0-9]*\"/\"checksum\": \"$checksum\"/" "$stacks_json"
        fi

        # Update symphony JSON (convert underscore to hyphen for filename)
        local symphony_json="$SYMPHONY_DIR/gap-follower-${variant}.json"
        if [ -f "$symphony_json" ]; then
            sed_inplace "s/\"checksum\": \"[a-f0-9]*\"/\"checksum\": \"$checksum\"/" "$symphony_json"
        fi

        print_success "Created $tar_name (${checksum:0:12}...)"
    done
}

# Show Muto logs from inside a container
show_muto_logs() {
    local container="${1:-ros-racer-edge-1}"
    local lines="${2:-20}"
    local title="${3:-Muto Logs}"

    echo ""
    echo -e "${CYAN}┌─────────────────────────────────────────────────────────────────────────────┐${NC}"
    echo -e "${CYAN}│${NC}  ${BOLD}$title${NC} (last $lines lines from $container)"
    echo -e "${CYAN}└─────────────────────────────────────────────────────────────────────────────┘${NC}"

    $CONTAINER_RUNTIME logs "$container" 2>&1 | tail -n "$lines" | while IFS= read -r line; do
        # Colorize log levels
        if [[ "$line" == *"[INFO]"* ]]; then
            echo -e "  ${GREEN}$line${NC}"
        elif [[ "$line" == *"[WARN]"* ]]; then
            echo -e "  ${YELLOW}$line${NC}"
        elif [[ "$line" == *"[ERROR]"* ]]; then
            echo -e "  ${RED}$line${NC}"
        else
            echo -e "  ${DIM}$line${NC}"
        fi
    done
}

# Show Muto state from inside a container
show_muto_state() {
    local container="${1:-ros-racer-edge-1}"
    local title="${2:-Muto State}"

    echo ""
    echo -e "${CYAN}┌─────────────────────────────────────────────────────────────────────────────┐${NC}"
    echo -e "${CYAN}│${NC}  ${BOLD}$title${NC} (from $container)"
    echo -e "${CYAN}└─────────────────────────────────────────────────────────────────────────────┘${NC}"

    # Try to read state from inside the container - check workspaces directory
    local state_output=$($CONTAINER_RUNTIME exec "$container" bash -c '
        WORKSPACES_DIR="$HOME/.muto/workspaces"
        if [ -d "$WORKSPACES_DIR" ]; then
            echo "Deployed workspaces:"
            for ws in "$WORKSPACES_DIR"/*/; do
                if [ -d "$ws" ]; then
                    ws_name=$(basename "$ws")
                    echo "  - $ws_name"
                fi
            done
            echo ""
            # Check for running gap_follower process
            if pgrep -f "gap_follower" > /dev/null 2>&1; then
                echo "Running: gap_follower node is active"
            else
                echo "Status: No gap_follower process running"
            fi
        else
            echo "  No workspaces deployed yet"
        fi
    ' 2>/dev/null)

    if [ -n "$state_output" ]; then
        echo "$state_output"
    else
        echo -e "  ${DIM}(State not available yet or container not running)${NC}"
    fi
}

# Show Muto state from all edge containers
show_all_muto_states() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  Muto State Across Fleet (${NUM_AGENTS} vehicles)${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════════════${NC}"

    for i in $(seq 1 "$NUM_AGENTS"); do
        local container
        if [ "$i" -eq 1 ]; then
            container="ros-racer-edge-1"
        else
            container="ros-racer-edge${i}-1"
        fi
        show_muto_state "$container" "racecar${i}"
    done
}

# Deploy a stack by running the deploy script inside a container
# This ensures DDS discovery works regardless of local RMW implementation
deploy_stack() {
    local stack_json="$1"
    local vehicle="${2:-}"  # Optional: target specific vehicle
    local container="ros-racer-edge-1"

    # Get just the filename from the path
    local stack_filename=$(basename "$stack_json")

    # Build the command - use the mounted /edge/demo directory
    # Pass NUM_AGENTS and DEPLOY_DISCOVERY_WAIT so deploy-stack.py knows fleet size
    local cmd="export NUM_AGENTS=${NUM_AGENTS} DEPLOY_DISCOVERY_WAIT=${DEPLOY_DISCOVERY_WAIT} && source /opt/ros/humble/setup.bash && source /edge/install/setup.bash && cd /edge && python3 demo/scripts/deploy-stack.py demo/stacks/${stack_filename}"

    if [ -n "$vehicle" ]; then
        cmd="$cmd --vehicle $vehicle"
    fi

    # Execute inside container
    $CONTAINER_RUNTIME exec "$container" bash -c "$cmd" 2>&1
}

# Get Symphony auth token
get_symphony_token() {
    curl -s -X POST -H "Content-Type: application/json" \
        -d '{"username":"admin","password":""}' \
        "${SYMPHONY_API_URL}users/auth" 2>/dev/null | jq -r '.accessToken' 2>/dev/null
}

# Create Symphony solution from stack JSON
create_symphony_solution() {
    local json_file="$1"
    local root_name=$(basename "$json_file" .json)
    local solution_name="${root_name}-v-1"

    local token=$(get_symphony_token)
    if [ -z "$token" ] || [ "$token" = "null" ]; then
        print_error "Failed to authenticate with Symphony"
        return 1
    fi

    # Base64 encode the stack
    local stack_base64=$(base64 -w 0 "$json_file" 2>/dev/null || base64 < "$json_file" | tr -d '\n')

    # Create solution JSON
    local solution_data=$(cat << EOF
{
    "metadata": {
        "namespace": "default",
        "name": "$solution_name"
    },
    "spec": {
        "displayName": "$solution_name",
        "rootResource": "$root_name",
        "version": "1",
        "components": [
            {
                "name": "$solution_name",
                "type": "muto-agent",
                "properties": {
                    "type": "stack",
                    "content-type": "application/json",
                    "data": "$stack_base64"
                }
            }
        ]
    }
}
EOF
)

    # Delete existing if present
    curl -s -X DELETE -H "Authorization: Bearer $token" \
        "${SYMPHONY_API_URL}solutions/$solution_name" > /dev/null 2>&1

    # Create new solution
    local response=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $token" \
        -d "$solution_data" \
        "${SYMPHONY_API_URL}solutions/$solution_name" 2>/dev/null)

    local http_status=$(echo "$response" | tail -n1)

    if [ "$http_status" -eq 200 ] || [ "$http_status" -eq 201 ]; then
        print_success "Solution '$solution_name' created"
        return 0
    else
        print_error "Failed to create solution (HTTP $http_status)"
        return 1
    fi
}

# Create Symphony target
create_symphony_target() {
    local json_file="$1"
    local target_name=$(jq -r '.metadata.name' "$json_file")

    local token=$(get_symphony_token)
    if [ -z "$token" ] || [ "$token" = "null" ]; then
        print_error "Failed to authenticate with Symphony"
        return 1
    fi

    # Delete existing if present
    curl -s -X DELETE -H "Authorization: Bearer $token" \
        "${SYMPHONY_API_URL}targets/registry/$target_name" > /dev/null 2>&1

    # Create new target
    local response=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $token" \
        -d @"$json_file" \
        "${SYMPHONY_API_URL}targets/registry/$target_name" 2>/dev/null)

    local http_status=$(echo "$response" | tail -n1)

    if [ "$http_status" -eq 200 ] || [ "$http_status" -eq 201 ]; then
        print_success "Target '$target_name' created"
        return 0
    else
        print_error "Failed to create target (HTTP $http_status)"
        return 1
    fi
}

# Create Symphony instance (triggers deployment)
create_symphony_instance() {
    local solution_name="$1"
    local target_name="$2"
    local instance_name="${solution_name}-instance"

    local token=$(get_symphony_token)
    if [ -z "$token" ] || [ "$token" = "null" ]; then
        print_error "Failed to authenticate with Symphony"
        return 1
    fi

    # Create instance JSON
    local instance_data=$(cat << EOF
{
    "metadata": {
        "name": "$instance_name",
        "labels": {
            "muto": "demo"
        }
    },
    "spec": {
        "displayName": "$instance_name",
        "solution": "${solution_name}:1",
        "target": {
            "name": "$target_name"
        }
    }
}
EOF
)

    # Delete existing if present
    curl -s -X DELETE -H "Authorization: Bearer $token" \
        "${SYMPHONY_API_URL}instances/$instance_name" > /dev/null 2>&1

    sleep 1

    # Create new instance
    local response=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -H "Authorization: Bearer $token" \
        -d "$instance_data" \
        "${SYMPHONY_API_URL}instances/$instance_name" 2>/dev/null)

    local http_status=$(echo "$response" | tail -n1)

    if [ "$http_status" -eq 200 ] || [ "$http_status" -eq 201 ] || [ "$http_status" -eq 204 ]; then
        print_success "Instance '$instance_name' created - deployment triggered!"
        return 0
    else
        print_error "Failed to create instance (HTTP $http_status)"
        return 1
    fi
}

# =============================================================================
# Demo Phases
# =============================================================================

phase_intro() {
    clear
    print_header "Eclipse Muto OTA Demo"

    echo -e "${WHITE}Welcome to the Eclipse Muto Over-The-Air (OTA) Update Demonstration!${NC}"
    echo ""
    echo -e "${CYAN}Fleet Configuration:${NC}"
    echo -e "  ${CYAN}•${NC} Agents:     ${BOLD}${NUM_AGENTS}${NC} racecars"
    echo ""
    echo "This interactive demo will walk you through:"
    echo ""
    echo -e "  ${CYAN}PART 1: Infrastructure Setup${NC}"
    echo -e "  ${GREEN}1.${NC} Starting Eclipse Symphony (cloud orchestration)"
    echo -e "  ${GREEN}2.${NC} Starting F1TENTH simulation with ${NUM_AGENTS} Muto edge containers"
    echo ""
    echo -e "  ${CYAN}PART 2: Direct Deployment (Muto Native)${NC}"
    echo -e "  ${GREEN}3.${NC} Deploy conservative algorithm via ROS topic"
    echo -e "  ${GREEN}4.${NC} OTA update to balanced algorithm"
    echo -e "  ${GREEN}5.${NC} OTA update to aggressive algorithm"
    echo -e "  ${GREEN}6.${NC} Deploy broken algorithm (automatic rollback)"
    echo ""
    echo -e "  ${CYAN}PART 3: Symphony Orchestration (Cloud-Native)${NC}"
    echo -e "  ${GREEN}7.${NC} Register solutions and targets in Symphony"
    echo -e "  ${GREEN}8.${NC} Deploy via Symphony instance creation"
    echo ""
    echo -e "  ${CYAN}PART 4: Fleet Management${NC}"
    echo -e "  ${GREEN}9.${NC} Deploy different algorithms to ${NUM_AGENTS} vehicles"
    echo ""
    echo -e "${DIM}Each step will explain what's happening and wait for your confirmation.${NC}"

    wait_for_enter
}

phase_prerequisites() {
    print_step "Checking Prerequisites"

    print_info "Verifying required tools are installed..."

    if detect_container_runtime; then
        print_success "Container runtime found: $CONTAINER_RUNTIME"
    else
        print_error "No container runtime found. Please install Docker or Podman."
        exit 1
    fi

    check_command curl
    print_success "curl found"

    check_command jq
    print_success "jq found"

    check_command base64
    print_success "base64 found"

    echo ""
    # Generate artifact archives and update checksums
    generate_artifacts

    # Create shared Docker network (used by both symphony and simulation compose files)
    if ! $CONTAINER_RUNTIME network inspect ros-racer_x11 &>/dev/null; then
        $CONTAINER_RUNTIME network create ros-racer_x11
        print_success "Created shared Docker network: ros-racer_x11"
    else
        print_success "Shared Docker network ros-racer_x11 already exists"
    fi

    wait_for_enter
}

phase_start_symphony() {
    print_step "Starting Eclipse Symphony Infrastructure"

    print_narration "Eclipse Symphony is a cloud-native orchestration platform that can manage
deployments across edge devices. It communicates with Muto agents via MQTT."

    echo "Services to be started:"
    echo -e "  ${CYAN}•${NC} symphony-api    - REST API for orchestration (port $PORT_SYMPHONY_API)"
    echo -e "  ${CYAN}•${NC} symphony-portal - Web dashboard (port $PORT_SYMPHONY_PORTAL)"
    echo -e "  ${CYAN}•${NC} mosquitto       - MQTT broker (port $PORT_MQTT)"
    echo ""

    print_action "Running: $CONTAINER_RUNTIME compose up -d --build (in symphony directory)"

    wait_for_enter

    cd "$SYMPHONY_INFRA_DIR"
    $CONTAINER_RUNTIME compose up -d --build

    print_success "Symphony containers started"

    countdown 8 "Waiting for Symphony API to initialize"

    # Verify Symphony is responding
    print_info "Verifying Symphony API..."
    local token=$(get_symphony_token)
    if [ -n "$token" ] && [ "$token" != "null" ]; then
        print_success "Symphony API is responding!"
    else
        print_warning "Symphony API may not be fully ready yet"
    fi

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  Symphony Portal: ${BOLD}http://localhost:${PORT_SYMPHONY_PORTAL}${NC}                    ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  Symphony API:    ${BOLD}http://localhost:${PORT_SYMPHONY_API}${NC}                    ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  MQTT Broker:     ${BOLD}localhost:${PORT_MQTT}${NC}                           ${GREEN}║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"

    wait_for_enter
}

phase_start_simulation() {
    print_step "Starting F1TENTH Simulation Infrastructure"

    print_narration "Now we'll start the F1TENTH racing simulation with ${NUM_AGENTS} Muto edge containers.
Each edge container represents an autonomous racecar with the Muto agent and composer."

    echo "Configuration:"
    echo -e "  ${CYAN}NUM_AGENTS:${NC}          $NUM_AGENTS"
    echo ""
    echo "Services to be started:"
    echo -e "  ${CYAN}•${NC} novnc           - VNC server for visualization (port $PORT_NOVNC)"
    echo -e "  ${CYAN}•${NC} sim             - F1TENTH Gym simulator with RViz"
    echo -e "  ${CYAN}•${NC} artifact-server - Nginx for stack downloads"
    for i in $(seq 1 "$NUM_AGENTS"); do
        if [ "$i" -eq 1 ]; then
            echo -e "  ${CYAN}•${NC} edge            - Muto edge container (racecar1)"
        else
            echo -e "  ${CYAN}•${NC} edge${i}           - Muto edge container (racecar${i})"
        fi
    done
    echo ""

    print_info "Generating docker-compose.yml for ${NUM_AGENTS} agents..."
    cd "$SCRIPT_DIR"
    NUM_AGENTS="$NUM_AGENTS" python3 scripts/generate-compose.py > docker-compose.yml
    print_success "docker-compose.yml generated"

    print_action "Running: $CONTAINER_RUNTIME compose up -d --build"

    wait_for_enter

    cd "$SCRIPT_DIR"
    $CONTAINER_RUNTIME compose up -d --build

    print_success "Simulation containers started"

    countdown 15 "Waiting for simulation to initialize"

    echo ""
    print_info "Checking container status..."
    $CONTAINER_RUNTIME compose ps

    echo ""
    print_success "Simulation is running!"
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  Open your browser to: ${BOLD}http://localhost:${PORT_NOVNC}/vnc.html${NC}       ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  You should see RViz with the F1TENTH track                  ${GREEN}║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"

    wait_with_message "Open the VNC viewer in your browser."
}

# =============================================================================
# PART 2: Direct Deployment (Muto Native)
# =============================================================================

phase_direct_header() {
    print_header "PART 2: Direct Deployment via ROS Topics"

    print_narration "In this section, we demonstrate Muto's native deployment capability.
Stack definitions are published directly to ROS topics, and each Muto agent
receives and processes them independently."

    wait_for_enter
}

phase_deploy_conservative() {
    print_step "Deploy Conservative Algorithm (Direct)"

    print_narration "We'll deploy our first driving algorithm - a conservative gap follower.
This algorithm prioritizes safety with moderate speeds and wide safety margins."

    echo "Stack details:"
    echo -e "  ${CYAN}Name:${NC}    gap_follower_conservative"
    echo -e "  ${CYAN}Version:${NC} 1.0.0"
    echo -e "  ${CYAN}Speed:${NC}   1.5 m/s (moderate)"
    echo -e "  ${CYAN}Gap:${NC}     2.5 m (wide safety margin)"
    echo -e "  ${CYAN}Method:${NC}  Direct ROS topic publish"
    echo ""

    print_info "What happens behind the scenes:"
    echo "  1. Stack definition published to ROS topic /muto/stack"
    echo "  2. Each Muto agent receives the message"
    echo "  3. Composer downloads algorithm package from artifact server"
    echo "  4. Workspace is built and gap_follower node is launched"
    echo ""

    print_action "Deploying: gap_follower_conservative.json"

    wait_for_enter

    deploy_stack "$STACKS_DIR/gap_follower_conservative.json"

    print_success "Deployment command sent!"

    countdown 10 "Waiting for provisioning and launch"

    echo ""
    print_success "Conservative algorithm deployed!"
    echo ""
    echo -e "${DIM}Watch the simulation - the cars should be moving slowly and cautiously.${NC}"

    # Show Muto logs and state after first deployment
    show_muto_logs "ros-racer-edge-1" 45 "racecar1 Muto Logs"
    show_muto_state "ros-racer-edge-1" "racecar1 State"

    wait_with_message "Observe the cars moving slowly."
}

phase_deploy_balanced() {
    print_step "OTA Update to Balanced Algorithm (Direct)"

    print_narration "The engineering team validated the conservative algorithm. Now let's push
an Over-The-Air update with improved performance."

    echo "Stack details:"
    echo -e "  ${CYAN}Name:${NC}    gap_follower_balanced"
    echo -e "  ${CYAN}Version:${NC} 1.1.0"
    echo -e "  ${CYAN}Speed:${NC}   2.5 m/s (fast)"
    echo -e "  ${CYAN}Gap:${NC}     1.8 m (standard safety)"
    echo -e "  ${CYAN}Method:${NC}  Direct ROS topic publish"
    echo ""

    print_warning "This is a ZERO-DOWNTIME update!"
    print_info "The old algorithm will be stopped and the new one takes over seamlessly."
    echo ""

    print_action "Deploying: gap_follower_balanced.json"

    wait_for_enter

    deploy_stack "$STACKS_DIR/gap_follower_balanced.json"

    print_success "OTA update sent!"

    countdown 8 "Transitioning to new algorithm"

    echo ""
    print_success "Balanced algorithm deployed!"
    echo ""
    echo -e "${DIM}Watch the transition - the cars should immediately drive faster.${NC}"

    # Show logs and state
    show_muto_logs "ros-racer-edge-1" 45 "racecar1 Muto Logs"
    show_muto_state "ros-racer-edge-1" "racecar1 State"

    wait_with_message "Observe the speed increase."
}

phase_deploy_aggressive() {
    print_step "OTA Update to Aggressive Algorithm (Direct)"

    print_narration "For maximum performance, let's push the aggressive algorithm.
This pushes the cars to their limits with high speeds and tight safety margins."

    echo "Stack details:"
    echo -e "  ${CYAN}Name:${NC}    gap_follower_aggressive"
    echo -e "  ${CYAN}Version:${NC} 1.2.0"
    echo -e "  ${CYAN}Speed:${NC}   4.0 m/s (very fast)"
    echo -e "  ${CYAN}Gap:${NC}     1.0 m (tight safety margin)"
    echo -e "  ${CYAN}Method:${NC}  Direct ROS topic publish"
    echo ""

    print_action "Deploying: gap_follower_aggressive.json"

    wait_for_enter

    deploy_stack "$STACKS_DIR/gap_follower_aggressive.json"

    print_success "Aggressive algorithm deployed!"

    countdown 8 "Transitioning to aggressive mode"

    echo ""
    print_success "Aggressive algorithm active!"
    echo ""
    echo -e "${DIM}Watch the cars - they should be driving much faster now!${NC}"

    # Show logs
    show_muto_logs "ros-racer-edge-1" 45 "racecar1 Muto Logs"

    wait_with_message "Observe the aggressive driving behavior."
}

phase_deploy_broken() {
    print_step "Rollback Demo - Deploy Broken Algorithm"

    print_narration "Now for the critical scenario - what happens when a BAD update gets deployed?

We're about to push version 1.3.0, which has a critical bug that causes it
to crash on startup. This simulates a real-world scenario where a faulty
build makes it to production."

    echo "Stack details:"
    echo -e "  ${CYAN}Name:${NC}    gap_follower_broken"
    echo -e "  ${CYAN}Version:${NC} 1.3.0"
    echo -e "  ${RED}Status:${NC}  INTENTIONALLY BROKEN"
    echo ""

    # Show state BEFORE the broken deployment
    print_info "Current state BEFORE deploying broken version:"
    show_muto_state "ros-racer-edge-1" "racecar1 State (Before)"

    print_warning "The cars will STOP briefly when the broken algorithm crashes!"
    print_info "Then watch for AUTOMATIC ROLLBACK to the previous working version."
    echo ""

    print_action "Deploying: gap_follower_broken.json"

    wait_for_enter

    deploy_stack "$STACKS_DIR/gap_follower_broken.json"

    print_warning "Broken algorithm deployed - watching for failure..."

    countdown 10 "Waiting for crash and automatic rollback"

    echo ""
    # Show logs to see the failure and rollback
    print_info "Logs showing the failure and rollback:"
    show_muto_logs "ros-racer-edge-1" 45 "racecar1 Muto Logs (Failure & Rollback)"

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  ${BOLD}AUTOMATIC ROLLBACK COMPLETE!${NC}                                ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                               ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  Muto detected the deployment failure and automatically       ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  rolled back to the previous working version.                 ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                               ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  ${DIM}No human intervention required - self-healing fleet!${NC}        ${GREEN}║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Show state AFTER rollback
    show_muto_state "ros-racer-edge-1" "racecar1 State (After Rollback)"

    echo -e "${DIM}The cars should be moving again with the previous algorithm.${NC}"

    wait_with_message "Observe the automatic recovery."
}

# =============================================================================
# PART 3: Symphony Orchestration (Cloud-Native)
# =============================================================================

phase_symphony_header() {
    print_header "PART 3: Eclipse Symphony Orchestration"

    print_narration "Now we'll demonstrate the same deployments using Eclipse Symphony.
Symphony provides cloud-native orchestration with:
  - Centralized management via REST API
  - Solution/Instance abstraction
  - Target-based deployment policies
  - Integration with digital twins"

    wait_for_enter
}

phase_symphony_up() {
    print_step "Starting Symphony Services"

    print_narration "If you haven't started the Symphony infrastructure yet, we'll do it now. This includes the Symphony API, Portal, and MQTT broker."
    echo "Services to be started:"
    echo -e "  ${CYAN}•${NC} symphony-api    - REST API for orchestration (port $PORT_SYMPHONY_API)"
    echo -e "  ${CYAN}•${NC} symphony-portal - Web dashboard (port $PORT_SYMPHONY_PORTAL)"
    echo -e "  ${CYAN}•${NC} mosquitto       - MQTT broker (port $PORT_MQTT)"
    echo ""
    print_action "Running: $CONTAINER_RUNTIME compose up -d --build (in symphony directory)"

    wait_for_enter

    cd "$SYMPHONY_INFRA_DIR"
    $CONTAINER_RUNTIME compose up -d --build

    print_success "Symphony containers started"
    echo ""

    countdown 8 "Waiting for Symphony API to initialize"

    # Verify Symphony is responding
    print_info "Verifying Symphony API..."
    local token=$(get_symphony_token)
    if [ -n "$token" ] && [ "$token" != "null" ]; then
        print_success "Symphony API is responding!"
    else
        print_warning "Symphony API may not be fully ready yet"
    fi

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  Symphony Portal: ${BOLD}http://localhost:${PORT_SYMPHONY_PORTAL}${NC}                    ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  Symphony API:    ${BOLD}http://localhost:${PORT_SYMPHONY_API}${NC}                    ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  MQTT Broker:     ${BOLD}localhost:${PORT_MQTT}${NC}                           ${GREEN}║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"

    wait_for_enter
}

phase_symphony_setup() {
    print_step "Register Solutions and Targets in Symphony"

    print_narration "Before deploying via Symphony, we need to:
  1. Create Solutions (the stack definitions wrapped for Symphony)
  2. Create a Target (represents our fleet of racecars)"

    echo "Creating Solutions..."
    echo ""

    print_action "Creating solution: gap-follower-conservative"
    create_symphony_solution "$STACKS_DIR/gap_follower_conservative.json"

    print_action "Creating solution: gap-follower-balanced"
    create_symphony_solution "$STACKS_DIR/gap_follower_balanced.json"

    print_action "Creating solution: gap-follower-aggressive"
    create_symphony_solution "$STACKS_DIR/gap_follower_aggressive.json"

    echo ""
    echo "Creating Target..."
    echo ""

    print_action "Creating target: racecar-fleet"
    create_symphony_target "$SYMPHONY_DIR/target.json"

    echo ""
    print_success "Symphony resources registered!"
    echo ""
    echo -e "${DIM}You can view these in the Symphony Portal at http://localhost:${PORT_SYMPHONY_PORTAL}${NC}"

    wait_for_enter
}

phase_symphony_deploy_conservative() {
    print_step "Deploy via Symphony - Conservative Algorithm"

    print_narration "Now we deploy by creating a Symphony Instance.
An Instance links a Solution to a Target, triggering deployment.

Symphony flow:
  1. Instance created via REST API
  2. Symphony resolves Solution -> Stack definition
  3. Symphony publishes to MQTT (via target binding)
  4. Muto agents receive and deploy"

    echo "Instance details:"
    echo -e "  ${CYAN}Solution:${NC} gap_follower_conservative:1"
    echo -e "  ${CYAN}Target:${NC}   racecar-fleet"
    echo -e "  ${CYAN}Method:${NC}   Symphony REST API -> MQTT -> Muto"
    echo ""

    print_action "Creating instance to trigger deployment..."

    wait_for_enter

    create_symphony_instance "gap_follower_conservative" "racecar-fleet"

    countdown 5 "Waiting for Symphony-orchestrated deployment"

    echo ""
    print_success "Deployed via Symphony!"
    echo ""
    echo -e "${DIM}The cars should now be running the conservative algorithm.${NC}"

    wait_with_message "Observe the Symphony-orchestrated deployment."
}

phase_symphony_deploy_balanced() {
    print_step "Symphony OTA Update - Balanced Algorithm"

    print_narration "Updating the fleet via Symphony is as simple as creating a new Instance
with a different Solution. Symphony handles the transition."

    echo "Instance details:"
    echo -e "  ${CYAN}Solution:${NC} gap_follower_balanced:1"
    echo -e "  ${CYAN}Target:${NC}   racecar-fleet"
    echo ""

    print_action "Creating new instance for balanced algorithm..."

    wait_for_enter

    create_symphony_instance "gap_follower_balanced" "racecar-fleet"

    countdown 3 "Transitioning via Symphony"

    echo ""
    print_success "Symphony OTA update complete!"
    echo ""
    echo -e "${DIM}Cars should now be running the balanced algorithm.${NC}"

    wait_with_message "Observe the Symphony-managed update."
}

# =============================================================================
# PART 4: Fleet Heterogeneity
# =============================================================================

phase_fleet_heterogeneity() {
    print_step "Fleet Heterogeneity - Different Algorithms Per Vehicle"

    print_narration "In real fleet deployments, you often need different configurations
for different vehicles. This demonstrates targeting individual vehicles with
cycling driving styles: aggressive -> balanced -> conservative."

    # Define styles array (cycles through these)
    local styles=("aggressive" "balanced" "conservative")
    local style_descs=("fast, tight" "medium" "slow, safe")

    echo "Deployment plan (${NUM_AGENTS} vehicles):"
    for i in $(seq 1 "$NUM_AGENTS"); do
        local style_idx=$(( (i - 1) % 3 ))
        local style="${styles[$style_idx]}"
        local desc="${style_descs[$style_idx]}"
        echo -e "  ${CYAN}racecar${i}:${NC} ${style} (${desc})"
    done
    echo ""

    print_info "Using direct ROS topic deployment to target individual vehicles."
    print_info "Stagger delay: ${DEPLOY_STAGGER}s between deployments"
    echo ""

    wait_for_enter

    for i in $(seq 1 "$NUM_AGENTS"); do
        local style_idx=$(( (i - 1) % 3 ))
        local style="${styles[$style_idx]}"
        local stack_file="$STACKS_DIR/gap_follower_${style}.json"

        print_action "Deploying ${style} to racecar${i}..."
        deploy_stack "$stack_file" "racecar${i}"
        print_success "racecar${i} -> ${style}"

        # Stagger between deployments (except after last)
        if [ "$i" -lt "$NUM_AGENTS" ] && [ "$DEPLOY_STAGGER" != "0" ] && [ "$DEPLOY_STAGGER" != "0.0" ]; then
            sleep "$DEPLOY_STAGGER"
        fi
    done

    echo ""
    print_success "Fleet configured with heterogeneous algorithms!"
    echo ""

    # Show state from all vehicles - demonstrates independent state management
    print_info "Each vehicle independently manages its own state:"
    show_all_muto_states

    echo -e "${DIM}Watch the cars - each one should be driving differently now!${NC}"

    wait_with_message "Observe the different driving behaviors."
}

# =============================================================================
# Summary and Cleanup
# =============================================================================

phase_summary() {
    print_header "Demo Complete!"

    echo -e "${WHITE}Summary of what we demonstrated with ${NUM_AGENTS} racecars:${NC}"
    echo ""
    echo -e "  ${GREEN}PART 1: Infrastructure${NC}"
    echo "     - Eclipse Symphony for cloud orchestration"
    echo "     - F1TENTH simulation with ${NUM_AGENTS} Muto edge containers"
    echo ""
    echo -e "  ${GREEN}PART 2: Direct Deployment (Muto Native)${NC}"
    echo "     - Push algorithms directly via ROS topics"
    echo "     - Zero-downtime OTA updates"
    echo "     - Automatic rollback on failure"
    echo ""
    echo -e "  ${GREEN}PART 3: Symphony Orchestration${NC}"
    echo "     - Solution/Instance abstraction"
    echo "     - REST API driven deployments"
    echo "     - MQTT bridge to edge devices"
    echo ""
    echo -e "  ${GREEN}PART 4: Fleet Management${NC}"
    echo "     - Individual vehicle targeting"
    echo "     - Heterogeneous fleet configuration (${NUM_AGENTS} vehicles)"
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Eclipse Muto brings cloud-native software delivery to the edge."
    echo ""
    echo -e "${DIM}The simulation is still running. You can:${NC}"
    echo -e "  ${CYAN}•${NC} Continue experimenting with deployments"
    echo -e "  ${CYAN}•${NC} View Symphony Portal: ${BOLD}http://localhost:${PORT_SYMPHONY_PORTAL}${NC}"
    echo -e "  ${CYAN}•${NC} View simulation: ${BOLD}http://localhost:${PORT_NOVNC}/vnc.html${NC}"
    echo -e "  ${CYAN}•${NC} Run: ${BOLD}$CONTAINER_RUNTIME compose logs -f${NC} to see container logs"
    echo ""
}

phase_cleanup_prompt() {
    echo ""
    echo -e "${YELLOW}Would you like to stop all containers? (y/N)${NC}"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        print_action "Stopping simulation containers..."
        cd "$SCRIPT_DIR"
        $CONTAINER_RUNTIME compose down
        print_success "Simulation stopped"

        print_action "Stopping Symphony containers..."
        cd "$SYMPHONY_INFRA_DIR"
        $CONTAINER_RUNTIME compose down
        print_success "Symphony stopped"
    else
        print_info "Containers left running"
    fi
}

# =============================================================================
# Main Execution
# =============================================================================

main() {
    # Introduction
    phase_intro
    phase_prerequisites

    # PART 1: Infrastructure
    phase_symphony_up
    phase_start_simulation

    # PART 2: Direct Deployment (simplified to 2 variants + rollback demo)
    phase_direct_header
    phase_deploy_conservative
    phase_deploy_aggressive
    phase_deploy_broken

    # PART 3: Symphony Orchestration
    phase_symphony_header
    phase_symphony_setup
    phase_symphony_deploy_conservative
    phase_symphony_deploy_balanced

    # PART 4: Fleet Heterogeneity
    # phase_fleet_heterogeneity

    # Summary and Cleanup
    phase_summary
    phase_cleanup_prompt

    echo ""
    echo -e "${GREEN}Thank you for watching the Eclipse Muto demo!${NC}"
    echo ""
}

# Handle Ctrl+C gracefully
trap 'echo ""; print_warning "Demo interrupted. Containers may still be running."; exit 1' INT

# Run main
main
