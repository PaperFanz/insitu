find . -name '*.hpp' -or -name '*.cpp' | xargs clang-format-10 -i -style=file $1
