# wombat

### Setup Docker environment

This step is only required the first time or whenever the `wombat-base` Docker image is updated.
It will build a Docker image with all the dependencies required to build the `wombat` software.

```bash
./utils/dev-environment/setup-docker.sh
```

To start a developer Docker container, then you can use:

```bash
.devdocker/run.sh
```

### Build `wombat`

To build the `wombat` software you can use the `build.py` script.
**Note: it is recommended to run this command in a developer Docker container, to make sure that all dependencies, with their correct versions are available.**

```bash
./build.py
```

This will generate a `_ws` directory containing the built artifacts.
