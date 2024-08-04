#!/usr/bin/env bash
# This script is for generating Python files from QT's '.ui' files.
# Written by Samyar Sadat Akhavi, 2024.
set -e

# List of UI files.
UI_FILES_LIST=("about_dialog" "calibration_result_dialog" "joystick_dialog" "main_window"
               "selftest_result_dialog" "test_remote_buttons_dialog")

cd ../src/gui || exit 1
echo "Starting..."

# Remove old Python files and generate new ones.
for file in "${UI_FILES_LIST[@]}"; do
    PY_FILENAME="ui_$file.py"

    echo "Removing $PY_FILENAME..."
    rm -f "$PY_FILENAME"

    echo "Generating: $PY_FILENAME..."
    pyside6-uic "$file.ui" -o "$PY_FILENAME"
done

echo "Done!"
exit 0