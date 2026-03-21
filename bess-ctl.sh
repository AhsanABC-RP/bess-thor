#!/bin/bash
# BESS Service Control Script
# Controls the BESS ROS2 containerized stack

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Service lists
PTP_SERVICES="ptp4l phc2sys"
SENSOR_CONTAINERS="bess-ouster bess-cameras bess-microstrain"
PROCESSING_CONTAINERS="bess-slam bess-recorder bess-odometry"
ALL_CONTAINERS="${SENSOR_CONTAINERS} ${PROCESSING_CONTAINERS}"

cmd_start() {
    local target="${1:-all}"

    case "$target" in
        ptp)
            log_info "Starting PTP services..."
            sudo systemctl start ptp4l
            sudo systemctl start phc2sys
            ;;
        sensors)
            log_info "Starting sensor containers..."
            sudo systemctl start bess-sensors-pod
            ;;
        processing)
            log_info "Starting processing containers..."
            sudo systemctl start bess-processing-pod
            ;;
        all)
            cmd_start ptp
            sleep 2
            cmd_start sensors
            sleep 5
            cmd_start processing
            ;;
        *)
            log_info "Starting $target..."
            sudo systemctl start "$target"
            ;;
    esac

    log_info "Started $target"
}

cmd_stop() {
    local target="${1:-all}"

    case "$target" in
        ptp)
            log_info "Stopping PTP services..."
            sudo systemctl stop phc2sys || true
            sudo systemctl stop ptp4l || true
            ;;
        sensors)
            log_info "Stopping sensor containers..."
            sudo systemctl stop bess-sensors-pod || true
            ;;
        processing)
            log_info "Stopping processing containers..."
            sudo systemctl stop bess-processing-pod || true
            ;;
        all)
            cmd_stop processing
            cmd_stop sensors
            cmd_stop ptp
            ;;
        *)
            log_info "Stopping $target..."
            sudo systemctl stop "$target" || true
            ;;
    esac

    log_info "Stopped $target"
}

cmd_restart() {
    cmd_stop "${1:-all}"
    sleep 2
    cmd_start "${1:-all}"
}

cmd_status() {
    echo "=== PTP Services ==="
    for svc in $PTP_SERVICES; do
        status=$(systemctl is-active $svc 2>/dev/null || echo "inactive")
        if [ "$status" = "active" ]; then
            echo -e "  $svc: ${GREEN}active${NC}"
        else
            echo -e "  $svc: ${RED}$status${NC}"
        fi
    done

    echo ""
    echo "=== Container Pods ==="
    podman pod ps --format "table {{.Name}}\t{{.Status}}\t{{.Created}}" 2>/dev/null | grep -E "^(NAME|bess)" || echo "  No pods running"

    echo ""
    echo "=== Containers ==="
    podman ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" 2>/dev/null | grep -E "^(NAMES|bess)" || echo "  No containers"

    echo ""
    echo "=== ROS2 Topics (if available) ==="
    if podman exec bess-ouster ros2 topic list 2>/dev/null; then
        :
    else
        echo "  (containers not running or ROS2 not responding)"
    fi
}

cmd_logs() {
    local container="${1:-bess-ouster}"
    local lines="${2:-100}"

    log_info "Showing last $lines lines from $container..."
    podman logs --tail "$lines" -f "$container"
}

cmd_shell() {
    local container="${1:-bess-ouster}"

    log_info "Opening shell in $container..."
    podman exec -it "$container" bash
}

cmd_topics() {
    log_info "Listing ROS2 topics..."
    podman exec bess-ouster ros2 topic list
}

cmd_record() {
    local action="${1:-start}"

    case "$action" in
        start)
            log_info "Starting bag recording..."
            sudo systemctl start bess-recorder
            ;;
        stop)
            log_info "Stopping bag recording..."
            sudo systemctl stop bess-recorder
            ;;
        *)
            log_error "Unknown action: $action (use start|stop)"
            ;;
    esac
}

cmd_ptp_status() {
    log_info "PTP Status:"
    if systemctl is-active ptp4l &>/dev/null; then
        echo "=== ptp4l (last 20 lines) ==="
        journalctl -u ptp4l -n 20 --no-pager
        echo ""
        echo "=== phc2sys (last 10 lines) ==="
        journalctl -u phc2sys -n 10 --no-pager
    else
        log_warn "ptp4l is not running"
    fi
}

cmd_help() {
    cat << EOF
BESS Service Control

Usage: bess-ctl <command> [args]

Commands:
  start [target]     Start services (ptp|sensors|processing|all)
  stop [target]      Stop services (ptp|sensors|processing|all)
  restart [target]   Restart services
  status             Show status of all services
  logs <container>   Follow container logs
  shell <container>  Open shell in container
  topics             List ROS2 topics
  record start|stop  Control bag recording
  ptp-status         Show PTP synchronization status
  help               Show this help

Containers:
  bess-ouster        Ouster LiDAR driver
  bess-cameras       FLIR camera driver
  bess-microstrain   MicroStrain IMU driver
  bess-slam          FAST-LIO SLAM
  bess-recorder      Rosbag2 recorder
  bess-odometry      Tesla CAN + EKF

Examples:
  bess-ctl start all
  bess-ctl logs bess-slam
  bess-ctl shell bess-ouster
  bess-ctl record start
EOF
}

# Main
case "${1:-help}" in
    start)      cmd_start "${2:-all}" ;;
    stop)       cmd_stop "${2:-all}" ;;
    restart)    cmd_restart "${2:-all}" ;;
    status)     cmd_status ;;
    logs)       cmd_logs "${2:-bess-ouster}" "${3:-100}" ;;
    shell)      cmd_shell "${2:-bess-ouster}" ;;
    topics)     cmd_topics ;;
    record)     cmd_record "${2:-start}" ;;
    ptp-status) cmd_ptp_status ;;
    help|--help|-h) cmd_help ;;
    *)          log_error "Unknown command: $1"; cmd_help; exit 1 ;;
esac
