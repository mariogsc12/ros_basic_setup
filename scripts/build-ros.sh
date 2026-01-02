#!/usr/bin/env bash

# Abort on error, unset vars, and pipeline errors
set -o errexit
set -o nounset
set -o pipefail

# =====================================================
# Defaults
# =====================================================
DEFAULT_PATH_TO_REPO="ros2_ws"
DEFAULT_CLEAN=false

# =====================================================
# Variables
# =====================================================
PATH_TO_REPO="$DEFAULT_PATH_TO_REPO"
DO_CLEAN="$DEFAULT_CLEAN"

# =====================================================
# Help
# =====================================================
print_help() {
    cat << EOF

Build ROS 2 workspace.

Usage:
  $0 [options]

Note:
  This script must be run from the root of the project.

Options:
  --help                 Show this help message
  --path <path>          Path to ROS 2 workspace (default: $DEFAULT_PATH_TO_REPO)
  --clean                Remove build/, install/ and log/ before building

Exit codes:
  0  Build completed successfully
  1  Build failed

Examples:
  $0
  $0 --clean
  $0 --path ros2_ws --clean

EOF
}

# =====================================================
# Parse command line arguments
# =====================================================
parse_command_line() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help)
                print_help
                exit 0
                ;;
            --path|--path_to_repo)
                PATH_TO_REPO="$2"
                shift 2
                ;;
            --clean)
                DO_CLEAN=true
                shift
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

# =====================================================
# Clean workspace
# =====================================================
clean_workspace() {
    echo "Cleaning workspace: $PATH_TO_REPO"
    rm -rf \
        "$PATH_TO_REPO/build" \
        "$PATH_TO_REPO/install" \
        "$PATH_TO_REPO/log"
}

# =====================================================
# Build ROS workspace
# =====================================================
build_ros() {
    echo "Building ROS workspace: $PATH_TO_REPO"

    set +u
    source /opt/ros/$ROS_DISTRO/setup.bash
    set -u
    cd "$PATH_TO_REPO"

    set +e
    colcon build
    local build_status=$?
    set -e

    if [[ $build_status -ne 0 ]]; then
        return 1
    fi

    return 0

    return 0
}

# =====================================================
# Main
# =====================================================
main() {
    parse_command_line "$@"

    echo "Workspace path: $PATH_TO_REPO"
    echo "Clean before build: $DO_CLEAN"
    echo ""

    if [[ "$DO_CLEAN" == true ]]; then
        clean_workspace
    fi

    if build_ros; then
        echo ""
        echo "Build finished successfully"
        exit 0
    else
        echo ""
        echo "Build failed"
        exit 1
    fi
}

main "$@"
