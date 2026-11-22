# sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/suwenyiyang/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/suwenyiyang/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the make.fun competition "Build the best
Sokoban solver, 21 November 2026".

Every level is a 1-wide hallway with 1–4 rooms off it, and a box pushed
through a door can never come back — so each room is independent and the
level's push-optimal count is the sum of the rooms' optima. The solver
splits the level into rooms, solves each room exactly with push-space A*
(macro-pushes, flood-fill player reachability, a min-cost box-to-goal
assignment heuristic, and corner / 2×2 / unreachable-cell deadlock pruning),
then stitches the room solutions together with hallway walks.

On the practice set it solves all five instances in exactly the optimal
push counts (13, 16, 29, 47, 60) in well under a second, deterministically.

## Run it

Requires Node 22 or later (stdlib only, no dependencies, no network).

```sh
node main.ts
```

It reads the five instances from `/input/<id>/level.txt` and writes
`/output/<id>/solution.txt` for each. For local testing, point it elsewhere:

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

<sub>Entered on [make.fun](https://make.fun/?ref=&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
