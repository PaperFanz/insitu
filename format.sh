#!/bin/bash

set -e

function append_newline {
    if [[ -z "$(tail -c 1 "$1")" ]]; then
        echo "$1"
    else
        echo >> "$1"
    fi
}

if [ -z "$1" ]; then
    TARGET_DIR="."
else
    TARGET_DIR=$1
fi

pushd ${TARGET_DIR} >> /dev/null

# Find all source files using Git to automatically respect .gitignore
FILES=$(git ls-files "*.hpp" "*.cpp")

# Run clang-format
clang-format -i -style=file ${FILES}

# Check newlines
for f in ${FILES}; do
    append_newline $f
done

popd >> /dev/null

