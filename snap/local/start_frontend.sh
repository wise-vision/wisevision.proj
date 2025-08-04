#!/bin/bash
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

export PORT=3000
export HOST=0.0.0.0
export WDS_SOCKET_PORT=0
export BROWSER=none

export SNAP_FRONTEND="$SNAP_COMMON/var/lib/wisevisionproj/frontend"
export SNAP_NODE_MODULES="$SNAP_FRONTEND/node_modules"
export ESLINT_CACHE_LOCATION="$SNAP_USER_COMMON/.eslintcache"
export HOME="$SNAP_USER_COMMON"
export TMPDIR="$SNAP_USER_COMMON/tmp"

mkdir -p "$SNAP_FRONTEND"
mkdir -p "$SNAP_NODE_MODULES"
mkdir -p "$ESLINT_CACHE_LOCATION"
mkdir -p "$TMPDIR"

if [ ! -f "$SNAP_FRONTEND/package.json" ]; then
  echo "Frontend files not found, copying from /snap/"
  cp -r "$SNAP/var/lib/wisevisionproj/frontend/"* "$SNAP_FRONTEND/"
fi

if [ ! -d "$SNAP_NODE_MODULES" ]; then
  echo "No node_modules/ directory found, installing dependencies..."
  cd "$SNAP_FRONTEND"
  npm install
fi

API_URL=$(snapctl get backend-api-url)
export API_URL
export REACT_APP_API_BASE_URL=${API_URL:-$BACKEND_API_DEFAULT}
cd "$SNAP_FRONTEND"

exec npm start