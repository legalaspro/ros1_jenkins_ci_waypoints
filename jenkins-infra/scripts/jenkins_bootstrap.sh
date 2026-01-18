#!/bin/bash
# Combined script: Install Docker (if not present) + Run Jenkins via WAR file
# + Optional: install plugins (first run) + optional smee.io forwarder

# Exit on any error. If something fails, there's no use
# proceeding further because Jenkins might not be able 
# to run in that case
set -e 

# ========= Docker Installation (conditional) =========
if ! command -v docker &> /dev/null; then
  . /etc/os-release
  
  # WARNING:
  # This is a Construct-only workaround for Ubuntu 20.04 (focal). Do NOT treat this as a production install method.
  # Recommended solution: upgrade the machine to Ubuntu 22.04/24.04+ and use Docker's official install instructions.
  if [ "${VERSION_CODENAME:-}" = "focal" ]; then
    echo "Ubuntu focal detected (unsupported by get.docker.com). Installing docker.io from Ubuntu repo..."

    UPDATE_OUT="$(sudo apt-get update 2>&1)" || true

    if echo "$UPDATE_OUT" | grep -q "EXPKEYSIG F42ED6FBAB17C654"; then
        echo "ROS apt key expired; refreshing key and rewriting ROS repo entries to use signed-by..."

        CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
        ARCH="$(dpkg --print-architecture)"
        KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"

        # Ensure key exists
        sudo rm -f "$KEYRING"
        curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | sudo tee "$KEYRING" >/dev/null
        sudo chmod 644 "$KEYRING"

        ROS1_LINE="deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros/ubuntu ${CODENAME} main"
        ROS2_LINE="deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${CODENAME} main"

        # ---- ROS1: prefer updating existing ros-latest.list to avoid duplicates ----
        if [ -f /etc/apt/sources.list.d/ros-latest.list ]; then
            echo "$ROS1_LINE" | sudo tee /etc/apt/sources.list.d/ros-latest.list >/dev/null
            # If we previously created ros1.list, disable it to avoid duplicates
            [ -f /etc/apt/sources.list.d/ros1.list ] && sudo mv /etc/apt/sources.list.d/ros1.list /etc/apt/sources.list.d/ros1.list.disabled
        else
            echo "$ROS1_LINE" | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null
        fi

        # ---- ROS2: pick one canonical file name, and disable any older duplicates ----
        echo "$ROS2_LINE" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

        # Disable any *other* ROS1 duplicates (paranoid but safe)
        for f in /etc/apt/sources.list.d/*.list; do
            [ "$f" = "/etc/apt/sources.list.d/ros-latest.list" ] && continue
            [ "$f" = "/etc/apt/sources.list.d/ros1.list" ] && continue
            if grep -q "http://packages.ros.org/ros/ubuntu" "$f"; then
                echo "Disabling duplicate ROS1 source file: $f"
                sudo mv "$f" "${f}.disabled"
            fi
        done

        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get update
    fi
  
    sudo apt install -y docker.io=20.10.21-0ubuntu1~20.04.2
    sudo systemctl enable --now docker || true
    # Ensure docker storage dirs exist 
    sudo mkdir -p /var/lib/docker/tmp
    sudo chmod 0711 /var/lib/docker/tmp
    sudo systemctl restart docker || true

  else
    echo "Installing Docker Engine (get.docker.com convenience script)..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    rm -f get-docker.sh
  fi

  sudo usermod -aG docker "$USER"
  echo "Docker installed. NOTE: log out/in for 'docker' group to apply to new shells."
else
  echo "Docker already installed. Skipping."
fi

# ========= Jenkins WAR Setup =========
export JENKINS_HOME="$HOME/webpage_ws/jenkins"
mkdir -p "$JENKINS_HOME"

echo "Updating package list and installing Java 21..."
sudo apt-get update
sudo apt-get install -y openjdk-21-jre

cd "$HOME"
JENKINS_FILE="$HOME/jenkins.war"
JENKINS_VERSION="2.479.3"
JENKINS_URL_WAR="https://updates.jenkins.io/download/war/${JENKINS_VERSION}/jenkins.war"


# Download latest LTS WAR if not already present
if [ ! -f "$JENKINS_FILE" ]; then
  echo "Downloading Jenkins WAR ${JENKINS_VERSION}..."
  wget -O "$JENKINS_FILE" "$JENKINS_URL_WAR"
#   echo "Downloading latest Jenkins LTS WAR file..."
#   wget https://get.jenkins.io/war-stable/latest/jenkins.war -O jenkins.war
else
  echo "Existing jenkins.war found. Skipping download."
fi


# Check if Jenkins is already running
if pgrep -f "java .*jenkins\.war" >/dev/null 2>&1; then
  echo "Jenkins is already running. Exiting."
  exit 0
fi

# Start Jenkins (use prefix only if SLOT_PREFIX is defined)
echo "Starting Jenkins..."
LOG_FILE="$JENKINS_HOME/jenkins.log"
PID_FILE="$JENKINS_HOME/jenkins.pid"

# Start Jenkins under docker group so it can access /var/run/docker.sock immediately.
# This avoids needing newgrp or logout/login to run docker in pipelines.
if [ -n "${SLOT_PREFIX:-}" ]; then
  sg docker -c "nohup java -jar '$JENKINS_FILE' --prefix='/${SLOT_PREFIX}/jenkins/' >'$LOG_FILE' 2>&1 & echo \$! > '$PID_FILE'"
else
  sg docker -c "nohup java -jar '$JENKINS_FILE' >'$LOG_FILE' 2>&1 & echo \$! > '$PID_FILE'"
fi

JENKINS_PID="$(cat "$PID_FILE")"
sleep 5s

# Try to detect CONSTRUCT URL, fall back to local access
# ========= URLs =========
if [ -n "${SLOT_PREFIX:-}" ]; then
  LOCAL_URL="http://localhost:8080/${SLOT_PREFIX}/jenkins/"
  WEBHOOK_PATH="/${SLOT_PREFIX}/jenkins/github-webhook/"   # <-- note trailing slash
else
  LOCAL_URL="http://localhost:8080/"
  WEBHOOK_PATH="/github-webhook/"                          # <-- note trailing slash
fi

CONSTRUCT_URL=""
CONSTRUCT_WEBHOOK=""
if [ -n "${SLOT_PREFIX:-}" ] && curl -s --fail http://169.254.169.254/latest/meta-data/instance-id >/dev/null 2>&1; then
  INSTANCE_ID=$(curl -s http://169.254.169.254/latest/meta-data/instance-id)
  CONSTRUCT_URL="https://${INSTANCE_ID}.robotigniteacademy.com/${SLOT_PREFIX}/jenkins/"
  CONSTRUCT_WEBHOOK="https://${INSTANCE_ID}.robotigniteacademy.com/${SLOT_PREFIX}/jenkins/github-webhook/"
fi

echo ""
echo "1. Jenkins is running in the background (PID: $JENKINS_PID)."
echo "2. Local access: $LOCAL_URL"
if [ -n "$CONSTRUCT_URL" ]; then
  echo "   Construct access: $CONSTRUCT_URL"
else
  echo "   (No construct environment detected – use local or your server IP.)"
fi

# ========= Optional smee.io forwarder =========
# Usage:
#   SMEE_URL="https://smee.io/xxxxxx" ./your_script.sh
#
# In GitHub repo Webhook settings, set Payload URL to the SMEE_URL.
# This forwarder will deliver events to Jenkins at /github-webhook/
SMEE_PID=""
SMEE_LOG="$JENKINS_HOME/smee.log"

if [ -n "${SMEE_URL:-}" ]; then
  echo ""
  echo "SMEE_URL provided. Starting smee forwarder..."
  echo "Smee source:  $SMEE_URL"
  echo "Forwarding to: http://localhost:8080$WEBHOOK_PATH"

  # Use docker to run smee-client without needing node/npm installed.
  # Use sudo so it works even before docker group change applies.
  nohup sudo docker run --rm --network host node:20-alpine \
    sh -lc "npx -y smee-client@4.4.3 --url '$SMEE_URL' --path '$WEBHOOK_PATH' --port 8080" \
    >>"$SMEE_LOG" 2>&1 &

  SMEE_PID=$!
  echo "smee-client started (PID: $SMEE_PID). Log: $SMEE_LOG"
else
  echo ""
  echo "No SMEE_URL set. Skipping smee forwarder."
fi

# ========= Save info to state file =========
STATE_FILE="$HOME/jenkins_pid_url.txt"
{
  echo "To stop Jenkins, run: kill $JENKINS_PID"
  echo "Log file: $LOG_FILE"
  echo ""
  echo "Local Jenkins URL: $LOCAL_URL"
  echo "Local webhook URL: http://localhost:8080$WEBHOOK_PATH"
  if [ -n "$CONSTRUCT_URL" ]; then
    echo "Construct Jenkins URL: $CONSTRUCT_URL"
    echo "Construct webhook URL: $CONSTRUCT_WEBHOOK"
  fi
  echo ""
  if [ -n "$SMEE_PID" ]; then
    echo "To stop smee-client, run: kill $SMEE_PID"
    echo "Smee log: $SMEE_LOG"
  fi
  echo ""
  echo "Initial admin password:"
  echo "  cat $JENKINS_HOME/secrets/initialAdminPassword"
} > "$STATE_FILE"

echo ""
echo "3. Details saved to '$STATE_FILE'."
echo "Initial admin password: cat $JENKINS_HOME/secrets/initialAdminPassword"
echo "Done! Open the Jenkins URL in your browser to complete setup."