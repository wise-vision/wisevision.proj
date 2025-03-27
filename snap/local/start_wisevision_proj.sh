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


ROS_CONFIG_FILE=$(snapctl get ros-config-file)
if [ -z "$ROS_CONFIG_FILE" ]; then
    echo "❌ Error: ROS Configuration file path is not set."
    echo "➡️  Set it using:"
    echo "    snap set $SNAP_NAME ros-config-file=/path/to/your/file.json5"
    exit 1
fi
USE_EMAIL_NOTIFIER="$(snapctl get use-email-notifier)"
USE_FIREBASE_NOTIFIER="$(snapctl get use-firebase-notifier)"
EMAIL_USERNAME_NOTIFICATION="$(snapctl get email-username-notification)"
EMAIL_PASSWORD_NOTIFICATION="$(snapctl get email-password-notification)"
EMAIL_RECIPIENTS_NOTIFICATION="$(snapctl get email-recipients-notification)"
DEVICE_TOKENS_FIREBASE="$(snapctl get device-tokens-firebase)"
CHIRPSTACK_API_KEY="$(snapctl get chirpstack-api-key)"

# export CONFIG_FILE
export ROS_CONFIG_FILE
# export library path
export LD_LIBRARY_PATH="$SNAP/usr/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SNAP/lib:$LD_LIBRARY_PATH"
# export for wisevision_notification_manager
export USE_EMAIL_NOTIFIER
export USE_FIREBASE_NOTIFIER
export EMAIL_USERNAME_NOTIFICATION 
export EMAIL_PASSWORD_NOTIFICATION 
export EMAIL_RECIPIENTS_NOTIFICATION 
export DEVICE_TOKENS_FIREBASE
# Export for wisevision_lorawan_bridge
export CHIRPSTACK_API_KEY

# Create confgi.json file for wisevision_data_black_box 
# (TODO: Remove this after Black Box update)
touch connfig.json
cat <<EOF > config.json
{
  "zenoh_url": "http://localhost:8000/"
}
EOF

EMAIL_USERNAME_NOTIFICATION="$EMAIL_USERNAME_NOTIFICATION" \
EMAIL_PASSWORD_NOTIFICATION="$EMAIL_PASSWORD_NOTIFICATION" \
EMAIL_RECIPIENTS_NOTIFICATION="$EMAIL_RECIPIENTS_NOTIFICATION" \
USE_EMAIL_NOTIFIER="$USE_EMAIL_NOTIFIER" \
USE_FIREBASE_NOTIFIER="$USE_FIREBASE_NOTIFIER" \
DEVICE_TOKENS_FIREBASE="$DEVICE_TOKENS_FIREBASE" \
CHIRPSTACK_API_KEY="$CHIRPSTACK_API_KEY" \
ros2 launch /snap/$SNAP_NAME/current/opt/launch/launch.py config_file:="$ROS_CONFIG_FILE"