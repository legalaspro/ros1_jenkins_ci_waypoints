#!/bin/bash
set -e

# Required:
#   export SMEE_URL="https://smee.io/xxxxxx"
# Optional:
#   export SLOT_PREFIX="02cce60b-c2d6-4b7f-a1ec-f53c383a905c"
#   export JENKINS_PORT="8080"
#   export SMEE_LOG="/path/to/smee.log"
#   export SMEE_PIDFILE="/path/to/smee.pid"

if [ -z "${SMEE_URL:-}" ]; then
  echo "ERROR: SMEE_URL is not set."
  echo "Usage: SMEE_URL=\"https://smee.io/xxxxxx\" $0"
  exit 1
fi

JENKINS_PORT="${JENKINS_PORT:-8080}"
SLOT_PREFIX="${SLOT_PREFIX:-}"

# Compute Jenkins webhook path (note trailing slash)
if [ -n "$SLOT_PREFIX" ]; then
  WEBHOOK_PATH="/${SLOT_PREFIX}/jenkins/github-webhook/"
else
  WEBHOOK_PATH="/github-webhook/"
fi

# Logs / pid
SMEE_LOG="${SMEE_LOG:-$HOME/smee.log}"
SMEE_PIDFILE="${SMEE_PIDFILE:-$HOME/smee.pid}"

# Sanity: docker required
if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found. Install Docker first."
  exit 1
fi

# If already running, exit nicely
if [ -f "$SMEE_PIDFILE" ] && kill -0 "$(cat "$SMEE_PIDFILE")" >/dev/null 2>&1; then
  echo "smee-client already running (PID $(cat "$SMEE_PIDFILE"))."
  echo "Log: $SMEE_LOG"
  exit 0
fi

echo "Starting smee-client..."
echo "SMEE_URL:        $SMEE_URL"
echo "Forwarding to:   http://localhost:${JENKINS_PORT}${WEBHOOK_PATH}"
echo "Log:            $SMEE_LOG"

nohup sudo docker run --rm --network host node:20-alpine \
  sh -lc "npx -y smee-client@4.4.3  --url '$SMEE_URL' --path '$WEBHOOK_PATH' --port '$JENKINS_PORT'" \
  >>"$SMEE_LOG" 2>&1 &

echo $! > "$SMEE_PIDFILE"
echo "Started (PID $(cat "$SMEE_PIDFILE"))."
echo "To stop: kill $(cat "$SMEE_PIDFILE")"
