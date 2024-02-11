# wombat_cmake

The `wombat_cmake` package provides build-tools for all CMake-based packages in the `wombat` repository.
It's required to use it to ensure that packages are managed in a common way.

## Structure

The `cmake` directory contains both the CMake scripts used and exposed by this package.
The files named with the `Wombat` prefix constitute the public functions of this package, while the others are only for internal use.
Note that `Wombat` scripts can use access scripts.

To add a new `Wombat` script, after creating it in the `cmake` directory, you need to also create a corresponding `Extras.cmake` script pointing at it.
Then this just created `Extras.cmake` script needs to be installed as part of the `wombat_cmake` package installation step.
