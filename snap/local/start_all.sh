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

# Start all services
snapctl start wisevisionproj.influxdb
snapctl start wisevisionproj.zenoh
snapctl start wisevisionproj.wisevision-proj-ros
snapctl start wisevisionproj.backend
snapctl start wisevisionproj.frontend