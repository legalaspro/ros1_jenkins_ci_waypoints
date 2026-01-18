#!/bin/bash
set -e

SMEE_PIDFILE="${SMEE_PIDFILE:-$HOME/smee.pid}"

if [ ! -f "$SMEE_PIDFILE" ]; then
  echo "No PID file at $SMEE_PIDFILE"
  exit 1
fi

PID="$(cat "$SMEE_PIDFILE")"
if kill -0 "$PID" >/dev/null 2>&1; then
  kill "$PID"
  echo "Stopped smee-client (PID $PID)."
else
  echo "PID $PID not running."
fi

rm -f "$SMEE_PIDFILE"