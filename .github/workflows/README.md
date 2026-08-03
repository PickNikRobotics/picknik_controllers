# CI overview

| Workflow | Trigger | Blocking | What it covers |
|---|---|---|---|
| [`build_and_test.yaml`](build_and_test.yaml) | PR to `main`, push to `main`, manual | see below | Build + test on every distro `main` is released to |
| [`ci-format.yml`](ci-format.yml) | PR, manual | yes | `pre-commit` across all files |
| [`ci-ros-lint.yml`](ci-ros-lint.yml) | PR to `main`, manual | yes | `ament_copyright`, `ament_lint_cmake`, `ament_cpplint` on **lyrical** |
| [`jazzy-abi-compatibility.yml`](jazzy-abi-compatibility.yml) | PR to `main` | yes | ABI diff vs. the base branch |
| [`kilted-abi-compatibility.yml`](kilted-abi-compatibility.yml) | PR to `main` | yes | ABI diff vs. the base branch |
| [`rolling-abi-compatibility.yml`](rolling-abi-compatibility.yml) | PR to `main` | no | ABI diff vs. the base branch |
| [`prerelease-check.yml`](prerelease-check.yml) | manual | n/a | `industrial_ci` `PRERELEASE: true` — buildfarm dry-run before tagging |

## The three build tiers

`build_and_test.yaml` keeps the escalating-lookahead ladder from the original
stogl-robotics layout — each tier looks further into the future than the last —
but as one matrix instead of 20 files.

### Tier 1 — binary: released debs (`main` apt)

*"Can a user build this today?"* **These are the only jobs that gate merges.**

| Job | Base OS |
|---|---|
| `jazzy-main` | noble |
| `kilted-main + ccov` | noble |
| **`lyrical-main`** | **resolute** |

`lyrical` is the Resolute gate. It is a released distro, so its `main` apt is
populated, and every dependency this repo has is available there —
`ros2_control` 6.8.0, `realtime_tools` 5.2.0. This is the job that would have
caught the break that got us un-released from rolling.

### Tier 2 — binary against `testing` apt

*"What breaks at the next sync?"* `jazzy-testing`, `kilted-testing`,
`lyrical-testing`, `rolling-testing`. All non-blocking.

Rolling appears only at this tier, because Rolling's `main` apt has no Resolute
packages yet.

Non-blocking is deliberate: an upstream regression staged for release is
something we want to *see*, not something that should block an unrelated PR.
This repo has already been through one "7 PRs blocked on an ecosystem break"
episode.

### Tier 3 — semi-binary: immediate dependencies from source

*"What breaks in the mid future?"* `rolling-testing + upstream-source`.
Non-blocking.

Core ROS still comes from debs, but our immediate dependencies — `ros2_control`
and `realtime_tools` — are built from their development branches, listed in
[`picknik_controllers.rolling.repos`](../../picknik_controllers.rolling.repos).
This fails weeks before a breaking upstream change reaches any apt repo.

**This tier previously reported green without testing anything.** Its `.repos`
file was an entirely commented-out placeholder, which made the semi-binary job
byte-identical to the binary job. Wiring it up is what makes the ladder real.
It is the tier that would have caught
`LoanedCommandInterface::get_value()` being removed, months before it surfaced
as a buildfarm release failure.

Adding the same lookahead for another distro is one more matrix entry plus a
`picknik_controllers.<distro>.repos`.

### What was dropped

The old **source-build** tier (core ROS itself from source, via
`ros-tooling/action-ros-ci`) is gone. It was permanently red regardless of
tier value: it fetched its `.repos` with the deprecated `?token=` URL syntax
and got an HTTP 404 on every run. Rebuilding all of core ROS to test two
controllers is also poor value — tier 3 already covers the dependencies that
can realistically break us.

## Coverage

Runs on `kilted` (tier 1), not on rolling. Coverage attached to a non-blocking
job silently stops reporting whenever that job breaks, and rolling breakage is
precisely what this repo keeps hitting.

## Branches

`main` currently serves jazzy, kilted, lyrical and rolling. humble is served by
the [`humble`](https://github.com/PickNikRobotics/picknik_controllers/tree/humble)
branch, which carries its own copy of these workflows.

The intent is for `main` to serve **all** active distros using the source-level
compatibility guards described in
[moveit2#3751](https://github.com/moveit/moveit2/pull/3751). When that lands,
re-adding humble here is a single matrix entry — no new workflow files.

## No `schedule:` triggers

Deliberate. GitHub auto-disables cron-triggered workflows after 60 days of repo
inactivity, and it disables the **whole** workflow, not just the cron trigger.
That is what silently took all 20 of this repo's previous build workflows offline
between 2026-03 and 2026-08 — every workflow with a `schedule:` was disabled, and
every workflow without one survived.

This does cost nightly dependency-rot detection, which is what the cron was for.
The tiers above still catch the same breakages, just on PR and push rather than
on a timer. If a timer is wanted back, it should live somewhere that cannot
disable the PR gate with it — a separate repository-dispatch or a scheduled
workflow that only runs the non-blocking tiers.
