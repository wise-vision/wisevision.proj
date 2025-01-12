#!/usr/bin/env python3
"""
A script to check/add the MPL-2.0 license header to source files.
Usage:
  - To check all files:        ./add_license_header.py
  - To fix (add header):       ./add_license_header.py --fix
  - To specify file/directory: ./add_license_header.py [--fix] path/to/file_or_dir
  - To exclude directories:    ./add_license_header.py [--fix] --exclude-dirs path/to/exclude path/to/check
"""

import os
import sys
import argparse

EXACT_LICENSE_TEXT = """/*
 * Copyright (C) 2025  wisevision
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
"""

EXACT_LICENSE_TEXT_PY = """# 
#  Copyright (C) 2025  wisevision
# 
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""

# Some simple detection: we check if the file contains a key line from the header
HEADER_KEY_LINE = "This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0."

# File extensions we want to process
ALLOWED_EXTENSIONS = {".c", ".cpp", ".h", ".hpp", ".py", ".js", ".ts", ".java", ".rb", ".go", ".sh", ".cs"}

def has_license_header(content: str, ext: str) -> bool:
    if ext.lower() in {".py", ".rb", ".sh"}:
        return EXACT_LICENSE_TEXT_PY in content
    return EXACT_LICENSE_TEXT in content

def is_binary_file(filepath: str) -> bool:
    """Quick check if a file is likely binary by reading initial bytes."""
    try:
        with open(filepath, 'rb') as f:
            chunk = f.read(1024)
        # If it contains a null byte, it's probably binary
        return b'\0' in chunk
    except Exception:
        return True  # Fallback, treat as binary if we can't read it

def get_license_header(ext: str) -> str:
    if ext.lower() in {".py", ".rb", ".sh"}:
        return EXACT_LICENSE_TEXT_PY
    return EXACT_LICENSE_TEXT

def add_header_to_file(filepath: str) -> bool:
    """
    Checks if the file has the license header; if not, adds it.
    Returns True if we added the header, False if it was already present or couldn't process.
    """
    if is_binary_file(filepath):
        return False

    _, ext = os.path.splitext(filepath)
    # If the extension isn't in our list, skip
    if ext.lower() not in ALLOWED_EXTENSIONS:
        return False

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        return False  # Probably binary or unknown encoding

    if has_license_header(content, ext):
        return False  # Already has license

    license_header = get_license_header(ext)
    with open(filepath, 'w', encoding='utf-8') as f:
        print(f"Adding MPL header to: {filepath}")
        if content.startswith("#!"):
            first_line, rest = content.split("\n", 1)
            f.write(f"{first_line}\n{license_header}\n{rest}")
        else:
            f.write(f"{license_header}\n{content}")

    return True

def check_header_in_file(filepath: str) -> bool:
    """
    Checks if the file is (a) text-based and (b) contains the MPL license line.
    Returns True if the header is present or file is ignored; False if missing.
    """
    if is_binary_file(filepath):
        return True  # skip binary files, treat them as "passed"

    _, ext = os.path.splitext(filepath)
    if ext.lower() not in ALLOWED_EXTENSIONS:
        return True  # skip non-source files

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        return True  # skip unreadable text

    return has_license_header(content, ext)

def process_path(path: str, fix: bool, exclude_dirs: list) -> bool:
    if any(os.path.abspath(path).startswith(os.path.abspath(ex)) for ex in exclude_dirs):
        return True  # Skip excluded paths
    if os.path.isfile(path):
        if fix:
            added = add_header_to_file(path)
            return True  # Even if not added, not a "failure"
        else:
            return check_header_in_file(path)
    else:
        all_good = True
        for root, dirs, files in os.walk(path):
            # Filter out excluded directories
            dirs[:] = [d for d in dirs
                       if not any(os.path.abspath(os.path.join(root, d)).startswith(os.path.abspath(ex))
                                  for ex in exclude_dirs)]
            for filename in files:
                filepath = os.path.join(root, filename)
                if fix:
                    add_header_to_file(filepath)
                else:
                    if not check_header_in_file(filepath):
                        print(f"Missing MPL header: {filepath}")
                        all_good = False
        return all_good

def main():
    parser = argparse.ArgumentParser(description="Check or add MPL-2.0 license headers to source files.")
    parser.add_argument("--fix", action="store_true", help="Add the license header to files missing it.")
    parser.add_argument("--exclude-dirs", nargs="*", default=[],
                        help="List of directories to exclude from scanning.")
    parser.add_argument("paths", nargs="*", default=["."],
                        help="File or directory paths to process (default: current dir).")
    args = parser.parse_args()

    overall_success = True
    for p in args.paths:
        ok = process_path(p, args.fix, args.exclude_dirs)
        if not ok:
            overall_success = False

    if not overall_success and not args.fix:
        sys.exit(1)  # Non-zero exit if check fails

if __name__ == "__main__":
    main()
