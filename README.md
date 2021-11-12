# wombat

### Setup Environment

This step is only required the first time or whenever the `wombat-base` Docker image is updated.
It will build a Docker image with all the dependencies required to build the `wombat` software.

```bash
./utils/dev-environment/setup-docker.sh
```

Then, whenever you want to start a developer Docker container you can use:

```bash
.devdocker/run.sh
```

### Build `wombat`

**The `wombat` software is built in a developer Docker container using the `colcon` build tool.**

From the `wombat` directory, build the software and test it.

```bash
cd ~/wombat
colcon build
colcon test
```

This will generate a `_ws` sub-directory containing the built artifacts.
For more advanced commands check `colcon --help`.
