#!/usr/bin/env bash

set -euo pipefail

find src include core -type f \
  \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' \
  -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' -o -name '*.hxx' \) \
  -print0 | xargs -0 -r clang-format -i
