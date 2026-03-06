#!/usr/bin/env python3
"""Replace find_library() with direct path construction in ament cmake files.
QEMU arm64 emulation causes find_library to fail even when libraries exist.
This script bypasses find_library entirely by constructing the path directly.
"""
import glob, re, os

for path in glob.glob('/opt/ros/humble/share/*/cmake/ament_cmake_export_libraries-extras.cmake'):
    with open(path) as f:
        content = f.read()

    # Replace find_library blocks with direct path construction.
    # Pattern matches:
    #   set(_lib "NOTFOUND")
    #   find_library(
    #     _lib NAMES "${_library}"
    #     PATHS "<some_path>"
    #     NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
    #   )
    content = re.sub(
        r'set\(_lib "NOTFOUND"\)\s*\n'
        r'\s*find_library\(\s*\n'
        r'\s*_lib NAMES "\$\{_library\}"\s*\n'
        r'\s*PATHS "([^"]+)"\s*\n'
        r'\s*NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH\s*\n'
        r'\s*\)',
        r'set(_lib "\1/lib${_library}.so")\n'
        r'      if(NOT EXISTS "${_lib}")\n'
        r'        set(_lib "NOTFOUND")\n'
        r'      endif()',
        content
    )

    with open(path, 'w') as f:
        f.write(content)

print("Patched cmake files successfully.")
