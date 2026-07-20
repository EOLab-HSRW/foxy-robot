#!/usr/bin/env bash
set -Eeuo pipefail
IFS=$'\n\t'

# Foxy robot simulation installer
# Revision: compact-spinner-v3 (single-line operations, full output on failure)
# Supported host combinations:
#   Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic
#   Ubuntu 24.04 + ROS 2 Jazzy  + Gazebo Harmonic
SCRIPT_VERSION="compact-spinner-v3"
WORKSPACE="${FOXY_WS:-$HOME/foxy_ws}"
REPO_URL="https://github.com/EOLab-HSRW/foxy-robot.git"
REPO_DIR="${WORKSPACE}/src/foxy-robot"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  C_RESET=$'\033[0m'
  C_BLUE=$'\033[1;34m'
  C_GREEN=$'\033[1;32m'
  C_YELLOW=$'\033[1;33m'
  C_RED=$'\033[1;31m'
else
  C_RESET=""
  C_BLUE=""
  C_GREEN=""
  C_YELLOW=""
  C_RED=""
fi

info()    { printf '%s[INFO]%s %s\n' "$C_BLUE" "$C_RESET" "$*"; }
success() { printf '%s[ OK ]%s %s\n' "$C_GREEN" "$C_RESET" "$*"; }
warn()    { printf '%s[WARN]%s %s\n' "$C_YELLOW" "$C_RESET" "$*" >&2; }
error()   { printf '%s[FAIL]%s %s\n' "$C_RED" "$C_RESET" "$*" >&2; }
die()     { error "$*"; exit 1; }

section() {
  printf '\n%s==> %s%s\n\n' "$C_BLUE" "$*" "$C_RESET"
}

ACTIVE_COMMAND_PID=""
ACTIVE_SPINNER_PID=""
ACTIVE_OPERATION_LOG=""

cleanup_active_operation() {
  if [[ -n "${ACTIVE_COMMAND_PID:-}" ]]; then
    kill "$ACTIVE_COMMAND_PID" 2>/dev/null || true
    wait "$ACTIVE_COMMAND_PID" 2>/dev/null || true
    ACTIVE_COMMAND_PID=""
  fi

  if [[ -n "${ACTIVE_SPINNER_PID:-}" ]]; then
    kill "$ACTIVE_SPINNER_PID" 2>/dev/null || true
    wait "$ACTIVE_SPINNER_PID" 2>/dev/null || true
    ACTIVE_SPINNER_PID=""
  fi

  if [[ -n "${ACTIVE_OPERATION_LOG:-}" ]]; then
    rm -f "$ACTIVE_OPERATION_LOG"
    ACTIVE_OPERATION_LOG=""
  fi
}

run_with_spinner() {
  local message=$1
  shift

  local log_file
  local exit_code
  local command_pid
  local spinner_pid

  # Preserve normal command output when animation is unavailable or disabled.
  if [[ ! -t 2 || -n "${NO_SPINNER:-}" ]]; then
    printf '%s[RUN ]%s %s\n' "$C_BLUE" "$C_RESET" "$message" >&2

    if "$@"; then
      success "$message"
      printf '\n'
      return 0
    else
      exit_code=$?
      error "$message"
      printf '\n'
      return "$exit_code"
    fi
  fi

  log_file="$(mktemp -t foxy-installer.XXXXXX.log)"
  ACTIVE_OPERATION_LOG="$log_file"

  (
    # The wrapper owns diagnostics and cleanup. Avoid duplicate trap messages
    # in the captured output if the wrapped command fails.
    trap - ERR EXIT INT TERM
    "$@"
  ) >"$log_file" 2>&1 &
  command_pid=$!
  ACTIVE_COMMAND_PID="$command_pid"

  (
    local -a frames=('|' '/' '-' $'\\')
    local frame_index=0

    while kill -0 "$command_pid" 2>/dev/null; do
      printf '\r%s[%s]%s %s' \
        "$C_BLUE" \
        "${frames[frame_index % ${#frames[@]}]}" \
        "$C_RESET" \
        "$message" >&2

      frame_index=$((frame_index + 1))
      sleep 0.12
    done
  ) &

  spinner_pid=$!
  ACTIVE_SPINNER_PID="$spinner_pid"

  if wait "$command_pid"; then
    exit_code=0
  else
    exit_code=$?
  fi
  ACTIVE_COMMAND_PID=""

  kill "$spinner_pid" 2>/dev/null || true
  wait "$spinner_pid" 2>/dev/null || true
  ACTIVE_SPINNER_PID=""

  # Replace the animated operation line with one final status line.
  printf '\r\033[K' >&2

  if (( exit_code == 0 )); then
    printf '%s[ OK ]%s %s\n\n' \
      "$C_GREEN" "$C_RESET" "$message" >&2

    rm -f "$log_file"

    ACTIVE_OPERATION_LOG=""
    return 0
  fi

  printf '%s[FAIL]%s %s\n' \
    "$C_RED" "$C_RESET" "$message" >&2

  printf '\n%s--- Output ---%s\n' \
    "$C_RED" "$C_RESET" >&2
  cat "$log_file" >&2
  printf '%s--- End output ---%s\n\n' \
    "$C_RED" "$C_RESET" >&2

  rm -f "$log_file"
  ACTIVE_OPERATION_LOG=""
  return "$exit_code"
}

on_error() {
  local exit_code=$?
  local line_no=$1
  error "Command failed at line ${line_no} (exit ${exit_code})."
  exit "$exit_code"
}
trap 'on_error "$LINENO"' ERR
trap cleanup_active_operation EXIT
trap 'cleanup_active_operation; exit 130' INT
trap 'cleanup_active_operation; exit 143' TERM


require_interactive_terminal() {
  [[ -r /dev/tty ]] || die "This installer is interactive and requires a terminal."
}

ask_yes_no() {
  local prompt=$1
  local answer
  require_interactive_terminal


  while true; do
    printf '%s%s [y/N]: %s' "$C_YELLOW" "$prompt" "$C_RESET" > /dev/tty
    IFS= read -r answer < /dev/tty || return 1
    case "${answer,,}" in
      y|yes) return 0 ;;
      n|no|"") return 1 ;;
      *) warn "Please answer yes or no." ;;
    esac
  done
}

confirm_phrase() {
  local prompt=$1
  local phrase=$2
  local answer
  require_interactive_terminal

  printf '%s%s%s\n' "$C_RED" "$prompt" "$C_RESET" > /dev/tty
  printf 'Type %s%s%s to continue: ' "$C_RED" "$phrase" "$C_RESET" > /dev/tty
  IFS= read -r answer < /dev/tty || return 1
  [[ "$answer" == "$phrase" ]]
}

if (( EUID == 0 )); then
  die "Run this installer as your normal user, not with sudo. It invokes sudo only for system changes."
fi

[[ -f /etc/os-release ]] || die "Cannot identify the operating system: /etc/os-release is missing."
# shellcheck disable=SC1091
source /etc/os-release

OS_ID="${ID:-unknown}"
OS_VERSION="${VERSION_ID:-unknown}"
OS_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"
ROS_DISTRO_SELECTED=""

detect_ros_distribution() {
  local has_humble=false
  local has_jazzy=false

  [[ -f /opt/ros/humble/setup.bash ]] && has_humble=true
  [[ -f /opt/ros/jazzy/setup.bash ]] && has_jazzy=true

  if [[ "$has_jazzy" == true ]]; then
    ROS_DISTRO_SELECTED="jazzy"
    if [[ "$has_humble" == true ]]; then
      info "ROS 2 Humble and Jazzy are installed; selecting Jazzy."
    else
      info "Detected ROS 2 Jazzy in /opt/ros/jazzy."
    fi
  elif [[ "$has_humble" == true ]]; then
    ROS_DISTRO_SELECTED="humble"
    info "Detected ROS 2 Humble in /opt/ros/humble."
  fi

}

expected_ros_for_host() {
  [[ "$OS_ID" == "ubuntu" ]] || return 1

  case "$OS_VERSION" in
    22.04) printf 'humble\n' ;;
    24.04) printf 'jazzy\n' ;;

    *) return 1 ;;
  esac

}

install_ros2_from_debs() {
  local distro=$1

  local ros_apt_source_version
  local ros_apt_source_deb="/tmp/ros2-apt-source.deb"

  section "ROS 2 ${distro^}"

  run_with_spinner \
    "Updating package lists before installing ROS 2" \
    sudo apt-get update

  run_with_spinner \
    "Installing ROS 2 repository prerequisites" \
    sudo apt-get install -y locales software-properties-common curl ca-certificates

  if ! locale charmap 2>/dev/null | grep -qi 'UTF-8'; then
    run_with_spinner \
      "Generating the UTF-8 locale" \
      sudo locale-gen en_US en_US.UTF-8

    run_with_spinner \
      "Selecting the UTF-8 locale" \
      sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
  fi

  run_with_spinner \
    "Enabling the Ubuntu Universe repository" \
    sudo add-apt-repository -y universe

  run_with_spinner \
    "Refreshing package lists for the ROS 2 repository" \
    sudo apt-get update

  ros_apt_source_version="$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
      sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' |
      head -n 1
  )"
  [[ -n "$ros_apt_source_version" ]] || die "Could not determine the latest ros2-apt-source release."

  run_with_spinner \
    "Downloading the ROS 2 repository configuration" \
    curl -fL \
      -o "$ros_apt_source_deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${OS_CODENAME}_all.deb"

  run_with_spinner \
    "Installing the ROS 2 repository configuration" \
    sudo dpkg -i "$ros_apt_source_deb"
  rm -f "$ros_apt_source_deb"

  run_with_spinner \
    "Refreshing package lists with the ROS 2 repository" \
    sudo apt-get update

  run_with_spinner \
    "Installing ROS 2 ${distro^} desktop and development tools" \
    sudo apt-get install -y "ros-${distro}-desktop" ros-dev-tools


  [[ -f "/opt/ros/${distro}/setup.bash" ]] ||
    die "ROS 2 installation finished, but /opt/ros/${distro}/setup.bash is missing."

}

install_common_tools() {
  section "Build tools"

  run_with_spinner \
    "Refreshing package lists for build tools" \

    sudo apt-get update

  run_with_spinner \
    "Installing build and repository tools" \
    sudo apt-get install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
}

initialize_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run_with_spinner \
      "Initializing rosdep" \
      sudo rosdep init
  else
    success "rosdep already initialized"
    printf '\n'
  fi
}

configure_gazebo_repository() {
  section "Gazebo repository"

  sudo install -d -m 0755 /usr/share/keyrings
  sudo curl -fsSL \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    https://packages.osrfoundation.org/gazebo.gpg

  printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' \
    "$(dpkg --print-architecture)" \
    "$OS_CODENAME" |
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

  # The project directly declares the rosdep key "gz-harmonic".
  sudo install -d -m 0755 /etc/ros/rosdep/sources.list.d
  sudo curl -fsSL \
    --output /etc/ros/rosdep/sources.list.d/00-gazebo.list \
    https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list

  run_with_spinner \
    "Refreshing package lists for Gazebo" \
    sudo apt-get update

  run_with_spinner \
    "Updating the rosdep database" \
    rosdep update
}

installed_packages() {
  dpkg-query -W -f='${binary:Package}\t${Status}\n' 2>/dev/null |

    awk -F '\t' '$2 == "install ok installed" { print $1 }'
}

find_humble_fortress_conflicts() {

  local package

  while IFS= read -r package; do
    case "$package" in
      # Keep an existing Harmonic installation.
      ros-humble-ros-gzharmonic*|gz-harmonic*)
        ;;
      ignition-fortress*|gz-fortress*|ignition-*|libignition-*|\
      libgz-sim6*|libgz-gui6*|libgz-transport11*|libgz-msgs8*|\
      libgz-rendering6*|libgz-sensors6*|libgz-physics5*|\
      libgz-launch5*|libgz-fuel-tools7*|libsdformat12*|sdformat12*|\
      ros-humble-ros-ign*|ros-humble-ros-gz*|\
      ros-humble-ign-ros2-control*|ros-humble-gz-ros2-control*)
        printf '%s\n' "$package"
        ;;
    esac
  done < <(installed_packages)
}

remove_humble_fortress_conflicts() {
  local -a conflicts=()
  local -a removal_plan=()

  mapfile -t conflicts < <(find_humble_fortress_conflicts | sort -u)
  (( ${#conflicts[@]} > 0 )) || {
    info "No installed Gazebo Fortress / Ignition conflicts were detected."
    return 0
  }

  warn "Gazebo Fortress or Fortress-oriented ROS packages are installed:"
  printf '  - %s\n' "${conflicts[@]}" >&2

  mapfile -t removal_plan < <(
    apt-get -s remove "${conflicts[@]}" 2>/dev/null |
      awk '/^Remv / { print $2 }' |
      sort -u
  )

  if (( ${#removal_plan[@]} > 0 )); then
    warn "APT reports that the following packages would be removed:"
    printf '  - %s\n' "${removal_plan[@]}" >&2
  fi

  cat >&2 <<EOF

${C_RED}Gazebo Harmonic is not compatible with this Humble/Fortress package set.${C_RESET}
Continuing requires removing the packages above. This can break software,
workspaces, or scripts on this computer that depend on Gazebo Fortress.
The installer will not run apt autoremove.
EOF


  if ! confirm_phrase \
    "Would you like to remove Fortress and install Gazebo Harmonic?" \
    "REMOVE FORTRESS"; then
    die "Fortress removal was not confirmed; no conflicting packages were removed."
  fi

  run_with_spinner \
    "Removing Fortress-oriented packages" \

    sudo apt-get remove -y "${conflicts[@]}"
}

install_harmonic_for_humble() {
  section "Gazebo Harmonic"

  remove_humble_fortress_conflicts
  run_with_spinner \
    "Installing Gazebo Harmonic and its ROS 2 Humble integration" \
    sudo apt-get install -y gz-harmonic ros-humble-ros-gzharmonic
}

prepare_workspace() {
  section "Workspace setup"

  mkdir -p "${WORKSPACE}/src"

  if [[ -d "${REPO_DIR}/.git" ]]; then
    info "Using the existing repository checkout at ${REPO_DIR}."
  elif [[ -e "$REPO_DIR" ]]; then
    die "${REPO_DIR} exists but is not a Git checkout. Move it aside or remove it first."
  else
    run_with_spinner \
      "Cloning the foxy-robot repository" \
      git clone "$REPO_URL" "$REPO_DIR" --depth 1
  fi

  if [[ "$ROS_DISTRO_SELECTED" == "humble" ]]; then

    local repos_file="${REPO_DIR}/sim.humble.repos"
    [[ -f "$repos_file" ]] || die "Missing repository manifest: ${repos_file}"

    run_with_spinner \
      "Importing Humble simulation source dependencies" \
      vcs import "${WORKSPACE}/src" --shallow --skip-existing < "$repos_file"
  fi
}

handle_workspace_distro_change() {
  local marker="${WORKSPACE}/.foxy_sim_ros_distro"
  local previous=""

  if [[ -f "$marker" ]]; then
    IFS= read -r previous < "$marker" || true
  fi

  if [[ -n "$previous" && "$previous" != "$ROS_DISTRO_SELECTED" ]]; then
    warn "This workspace was previously built for ROS 2 ${previous^}."
    if ask_yes_no "Remove its build, install, and log directories before continuing?"; then
      rm -rf "${WORKSPACE}/build" "${WORKSPACE}/install" "${WORKSPACE}/log"
    else
      die "A clean rebuild is required when changing ROS distributions."
    fi
  fi
}

build_simulation_workspace() {

  local -a sim_paths=()
  local marker="${WORKSPACE}/.foxy_sim_ros_distro"

  section "Simulation dependencies and build"

  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_SELECTED}/setup.bash"

  if [[ "$ROS_DISTRO_SELECTED" == "humble" ]]; then
    export GZ_VERSION=harmonic
  fi

  pushd "$WORKSPACE" > /dev/null

  colcon list --names-only | grep -qx 'foxy_bringup_sim' ||
    die "The package foxy_bringup_sim was not found under ${WORKSPACE}/src."

  mapfile -t sim_paths < <(

    colcon list --paths-only --packages-up-to foxy_bringup_sim
  )
  (( ${#sim_paths[@]} > 0 )) || die "Could not calculate the simulation package dependency closure."

  if [[ "$ROS_DISTRO_SELECTED" == "humble" ]]; then
    run_with_spinner \
      "Installing Humble simulation dependencies with rosdep" \
      rosdep install \
      -r \
      -y \
      --from-paths "${sim_paths[@]}" \
      --ignore-src \
      --rosdistro "$ROS_DISTRO_SELECTED" \

      --skip-keys="ros_gz_bridge ros_gz_sim"
  else
    run_with_spinner \
      "Installing Jazzy simulation dependencies with rosdep" \
      rosdep install \
      -r \
      -y \
      --from-paths "${sim_paths[@]}" \
      --ignore-src \
      --rosdistro "$ROS_DISTRO_SELECTED"
  fi


  run_with_spinner \
    "Building foxy_bringup_sim and its workspace dependencies" \
    colcon build \
    --symlink-install \
    --packages-up-to foxy_bringup_sim

  printf '%s\n' "$ROS_DISTRO_SELECTED" > "$marker"


  # This validates the result inside the installer process. The caller still
  # needs to source this file in its own shell after the script exits.
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  ros2 pkg prefix foxy_bringup_sim > /dev/null

  popd > /dev/null
}

main() {

  local expected_ros=""


  if [[ "${1:-}" == "--version" ]]; then
    printf 'Foxy robot simulation installer %s\n' "$SCRIPT_VERSION"
    return 0
  fi

  info "Foxy robot simulation installer (${SCRIPT_VERSION})"
  info "Workspace: ${WORKSPACE}"
  info "Host: ${PRETTY_NAME:-${OS_ID} ${OS_VERSION}}"


  detect_ros_distribution

  if [[ -z "$ROS_DISTRO_SELECTED" ]]; then
    expected_ros="$(expected_ros_for_host)" ||

      die "No supported ROS 2 installation was found. Automatic installation supports only Ubuntu 22.04 or Ubuntu 24.04."


    cat <<EOF


ROS 2 was not found in /opt/ros/humble or /opt/ros/jazzy.
At least, it is not installed using the expected Debian package layout.

Power-user note: if ROS 2 is installed from source, Conda, a container, or
another custom setup, stop here and open this installer in an editor. You will
probably be better served by adapting it to your environment manually.

Detected ${PRETTY_NAME:-Ubuntu ${OS_VERSION}}; this installer can install ROS 2 ${expected_ros^}.
EOF

    if ! ask_yes_no "Would you like to install ROS 2?"; then
      info "ROS 2 installation declined."
      exit 0
    fi

    sudo -v

    install_ros2_from_debs "$expected_ros"
    ROS_DISTRO_SELECTED="$expected_ros"
  else
    sudo -v
  fi

  expected_ros="$(expected_ros_for_host)" ||

    die "This installer supports Ubuntu 22.04 and Ubuntu 24.04 only."


  if [[ "$ROS_DISTRO_SELECTED" != "$expected_ros" ]]; then

    die "ROS 2 ${ROS_DISTRO_SELECTED^} was selected, but Ubuntu ${OS_VERSION} requires ROS 2 ${expected_ros^} for this installer."
  fi

  install_common_tools
  initialize_rosdep
  configure_gazebo_repository


  if [[ "$ROS_DISTRO_SELECTED" == "humble" ]]; then
    install_harmonic_for_humble
  fi

  prepare_workspace
  handle_workspace_distro_change
  build_simulation_workspace

  cat <<EOF

${C_GREEN}Installation complete.${C_RESET}

ROS distribution: ${ROS_DISTRO_SELECTED}
Workspace:        ${WORKSPACE}

Load the workspace in the current terminal with:


  source /opt/ros/${ROS_DISTRO_SELECTED}/setup.bash
  source ${WORKSPACE}/install/setup.bash
EOF
}

main "$@"
