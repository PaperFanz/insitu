find . -name '*.hpp' -or -name '*.cpp' | xargs clang-format -i -style=file $1
