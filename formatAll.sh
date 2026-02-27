#!/bin/bash
 find ./source -name "*.c" -o -name "*.h" | xargs clang-format -i                                                                                                                      ─╯
