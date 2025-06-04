#!/bin/bash
set -e

# Find the actual .so file name
SO_FILE=$(find build -name "pydiscocal*.so" | head -n 1)

if [ -z "$SO_FILE" ]; then
    echo "Error: .so file not found!"
    exit 1
fi

echo "Using SO file: $SO_FILE"

# Run PyInstaller for mono and stereo
pyinstaller --onefile --add-binary "$SO_FILE:." src/python/run_mono.py
pyinstaller --onefile --add-binary "$SO_FILE:." src/python/run_stereo.py