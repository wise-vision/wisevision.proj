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


cd $SNAP/usr/src

exec gunicorn -k gthread -w 1 --threads 4 -b 0.0.0.0:5000 app.server.run:app