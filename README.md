# wombat

[![Testing](https://github.com/LIAR-team/wombat/actions/workflows/ci.yaml/badge.svg)](https://github.com/LIAR-team/wombat/actions/workflows/ci.yaml)

### Requirements

The build dependencies of `wombat` are provided through Docker.
On Ubuntu systems the setup procedure will take care of installing Docker.
The setup procedure will automatically take care of installing Docker if you are running Ubuntu.
Alternatively, [install Docker](https://docs.docker.com/get-docker/) and make sure that you can run it as a non-root user.

### Setup Environment

This step updates the `wombat-base` Docker image.

```bash
./utils/dev-environment/setup-docker.sh
```

This script will by default try to install GPU support.
it's possible to skip it by passing the additional argument `--no-gpu` to the `setup-docker.sh` script.

### Build `wombat`

**The `wombat` software is built in a developer Docker container using the `colcon` build tool.**

Start a developer Docker container:

```bash
.devdocker/run.sh
```

From the `wombat` directory, build the software and test it.

```bash
cd ~/wombat
colcon build
colcon test
```

This will generate a `_ws` sub-directory containing the built artifacts.
For more advanced commands check `colcon --help`.
