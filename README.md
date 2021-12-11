# wombat

### Requirements

All the `wombat` dependencies are provided through Docker.
The setup procedure will automatically take care of installing Docker if you are running Ubuntu.

If you are not running Ubuntu, then you will have to manually install Docker and make sure that you can run it as a non-root user.

### Setup Environment

This step is only required the first time or whenever the `wombat-base` Docker image is updated.
It will build a Docker image with all the dependencies required to build the `wombat` software.

```bash
./utils/dev-environment/setup-docker.sh
```

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
