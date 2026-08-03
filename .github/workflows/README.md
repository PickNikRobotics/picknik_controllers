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

## Build matrix

`build_and_test.yaml` runs one `industrial_ci` job per distro:

| Job | apt repo | Base OS | Blocking |
|---|---|---|---|
| `jazzy-main` | `main` | noble | yes |
| `kilted-main + ccov` | `main` | noble | yes |
| `lyrical-main` | `main` | **resolute** | **yes** |
| `rolling-testing` | `testing` | **resolute** | no |

`lyrical` is the Resolute gate. It is a released distro, so its `main` apt is
populated and every dependency this repo has is available there. Rolling is
non-blocking because Rolling's `main` apt has no Resolute packages yet, so it
tracks `testing` instead.

Coverage runs on `kilted` rather than on `rolling`: coverage attached to a
non-blocking job silently stops reporting whenever that job breaks, and rolling
breakage is precisely what this repo keeps hitting.

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
every workflow without one survived. Nightly dependency-rot detection is not worth
losing PR coverage to.
