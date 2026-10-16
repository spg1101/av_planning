# Contributing

## Workflow

1. Pick a task from the plan (IDs like `P1.4`). One task = one branch = one PR.
2. Branch from `main`: `feat/p1.4-world-model`, `fix/ekf-frame-id`, `docs/...`.
3. Write the algorithm in `planning_core` first, with unit tests. Wrap it in a node second.
4. Run before pushing:
   ```bash
   pre-commit run --all-files
   colcon build --symlink-install && colcon test && colcon test-result --verbose
   ```
5. Open a PR using the template. Fill in the "done when" test and the metric it hit.
6. CI must pass, and a code owner must approve. Squash-merge.

## Review rules

- The planning lead is a code owner of the whole repo and reviews every PR.
- The lead's own PRs (SLAM, behavior, tooling) are reviewed by Member A or Member B,
  which is why they are co-owners of those paths in `.github/CODEOWNERS`.
  GitHub never lets authors approve their own PRs.
- Keep PRs under ~400 changed lines. Split bigger tasks.

## Hard rules (these keep later team integration painless)

- Nothing in `planning_core` imports `rclpy`, ROS messages or any simulator.
- No message definitions in this repo. New messages go to `av_interfaces` (needs both leads).
- Topic names and frames follow `av_interfaces/docs/ICD.md`. Remap in launch files.
- Every tunable value is a ROS parameter or YAML, never a constant buried in code.
- Stand-ins for other subteams live only in `planning_sim`.

## GitHub settings to turn on (repo admin, once)

Settings -> Branches -> Add rule for `main`:

- [x] Require a pull request before merging
  - [x] Require approvals: 1
  - [x] Dismiss stale pull request approvals when new commits are pushed
  - [x] Require review from Code Owners
- [x] Require status checks to pass: `lint`, `build-and-test`
- [x] Require branches to be up to date before merging
- [x] Do not allow bypassing the above settings

Settings -> General -> Pull Requests: allow squash merging only, and
automatically delete head branches.
