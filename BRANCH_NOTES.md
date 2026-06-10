# feature/ci-build

## What

Adds `.github/workflows/build.yml`: every push and PR targeting `main` or
`fable` builds the workspace and runs the test suite inside the repo's own
Docker image.

- **Image**: the root `Dockerfile` (the same `dev:humble` image used locally
  via docker-compose/Justfile) is built with `docker/build-push-action` and
  GitHub Actions layer caching (`type=gha, mode=max`), so the ~4 GB ROS
  desktop apt layer is only rebuilt when the Dockerfile changes.
- **Build**: `colcon build --event-handlers console_cohesion+` inside the
  container, run as the host uid so cache files stay owner-correct. The
  `build/` and `install/` dirs are cached with `actions/cache`, keyed on every
  `CMakeLists.txt`/`package.xml` plus the Dockerfile, with a prefix
  restore-key so partial reuse still works after a key change.
- **Test**: `colcon test --event-handlers console_cohesion+` followed by
  `colcon test-result --verbose` — but only when some package actually
  produced result files, since `test-result` errors out on a workspace with
  no test targets (the current state of this repo).
- **cubesat_watch is skipped** in both steps: it requires the libcamera
  (>= 0.1 API) that exists on the Pi image; neither the dev image nor Ubuntu
  jammy's `libcamera-dev` (which ships a 2021-era API) can compile it. The
  skip is documented in the workflow. Everything else — msgs, captain, pi_io,
  stm_bridge, radio, bringup — builds and gates the PR.

## Why

Several of the bugs fixed on `fable` (three packages whose install step
referenced a nonexistent `config/` directory, a launch script with NameErrors)
could not have survived a single CI run. The Pi deploy flow (`deploy.sh`)
builds in Docker anyway, so CI building in the same image keeps "passes CI"
meaning "will build for deploy".

## Testing

- Workflow YAML parses; both container commands were executed locally against
  the dev:humble image verbatim: 6 packages build, the no-test-results branch
  of the test step is exercised correctly.
- Caveat: the GHA layer/colcon caches can only be fully validated on a real
  Actions runner.
