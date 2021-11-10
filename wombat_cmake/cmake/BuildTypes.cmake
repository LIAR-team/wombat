# Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
# All Rights Reserved.

# For more information see:
# https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Debugging-Options.html#Debugging-Options
# https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Optimize-Options.html#Optimize-Options

# Release
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Flags used by the C++ compiler for builds optimized for speed." FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Flags used by the C compiler for builds optimized for speed." FORCE)

# RelWithDebug
set(CMAKE_CXX_FLAGS_RELWITHDEBUG "-O2 -g -DDEBUG" CACHE STRING "Flags used by the C++ compiler for fast builds with full debug support." FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBUG "-O2 -g -DDEBUG" CACHE STRING "Flags used by the C compiler for fast builds with full debug support." FORCE)
