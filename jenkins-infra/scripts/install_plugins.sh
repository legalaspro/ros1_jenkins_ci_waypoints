#!/bin/bash
# Install Jenkins plugins into $JENKINS_HOME/plugins using the Plugin Installation Manager Tool.
# Runs once (marker file), supports FORCE_PLUGINS=1 to reinstall.

set -euo pipefail

# Inputs (override via env)
JENKINS_HOME="${JENKINS_HOME:-$HOME/webpage_ws/jenkins}"
JENKINS_FILE="${JENKINS_FILE:-$HOME/jenkins.war}"
JENKINS_VERSION="${JENKINS_VERSION:-2.479.3}"

# Where your plugins list lives (relative path assumes you run from repo root)
PLUGINS_FILE="${PLUGINS_FILE:-jenkins-infra/jenkins/plugins.txt}"

# Marker file so we only install once per Jenkins version
PLUGINS_MARKER="${PLUGINS_MARKER:-$JENKINS_HOME/.plugins_installed_${JENKINS_VERSION}}"
FORCE_PLUGINS="${FORCE_PLUGINS:-0}"

# Plugin Installation Manager Tool (pinned)
PIM_VERSION="${PIM_VERSION:-2.13.2}"
PIM_JAR="${PIM_JAR:-$HOME/jenkins-plugin-manager-${PIM_VERSION}.jar}"
PIM_URL="https://github.com/jenkinsci/plugin-installation-manager-tool/releases/download/${PIM_VERSION}/jenkins-plugin-manager-${PIM_VERSION}.jar"

echo "JENKINS_HOME:   $JENKINS_HOME"
echo "JENKINS_WAR:    $JENKINS_FILE"
echo "Jenkins ver:    $JENKINS_VERSION"
echo "Plugins file:   $PLUGINS_FILE"
echo "Marker:         $PLUGINS_MARKER"

# Basic checks
if [ ! -f "$JENKINS_FILE" ]; then
  echo "ERROR: Jenkins WAR not found at: $JENKINS_FILE"
  echo "Run your bootstrap (download WAR) first or set JENKINS_FILE."
  exit 1
fi

if [ ! -f "$PLUGINS_FILE" ]; then
  echo "ERROR: plugins file not found: $PLUGINS_FILE"
  exit 1
fi

if [ "$FORCE_PLUGINS" != "1" ] && [ -f "$PLUGINS_MARKER" ]; then
  echo "Plugins already installed for Jenkins ${JENKINS_VERSION}. (marker exists)"
  echo "To reinstall: FORCE_PLUGINS=1 $0"
  exit 0
fi

# Download tool if missing
if [ ! -f "$PIM_JAR" ]; then
  echo "Downloading plugin manager tool ${PIM_VERSION}..."
  curl -fsSL "$PIM_URL" -o "$PIM_JAR"
fi

mkdir -p "$JENKINS_HOME/plugins"

echo "Installing plugins (and dependencies) into $JENKINS_HOME/plugins ..."
java -jar "$PIM_JAR" \
  --war "$JENKINS_FILE" \
  --plugin-file "$PLUGINS_FILE" \
  --plugin-download-directory "$JENKINS_HOME/plugins"

touch "$PLUGINS_MARKER"
echo "Done. Plugins installed. (marker written)"
