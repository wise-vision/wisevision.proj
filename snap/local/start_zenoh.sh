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

CONFIG_FILE=$(snapctl get config-file) 

export CONFIG_FILE
echo "CONFIG_FILE: $CONFIG_FILE"

if [[ -z "$CONFIG_FILE" || "$CONFIG_FILE" == "null" ]]; then
    echo "Error: Configuration file path is not set."
    echo "Set it using:"
    echo "snap set $SNAP_NAME config-file=/path/to/your/file.json5"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file '$CONFIG_FILE' does not exist or is not accessible."
    exit 1
fi



exec "$SNAP/bin/zenohd" --plugin-search-dir "$SNAP/lib" -c "$CONFIG_FILE"