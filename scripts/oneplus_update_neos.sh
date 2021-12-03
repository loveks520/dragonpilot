#!/usr/bin/bash

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"
echo "Installing NEOS update"
NEOS_PY="$DIR/selfdrive/hardware/eon/neos.py"
MANIFEST="$DIR/selfdrive/hardware/eon/oneplus.json"
$NEOS_PY --swap-if-ready $MANIFEST
$DIR/selfdrive/hardware/eon/updater $NEOS_PY $MANIFEST
