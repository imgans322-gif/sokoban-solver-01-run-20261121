# Build the best Sokoban solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/imgans322-gif/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/imgans322-gif/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

An entry for [make.fun's sokoban-solver-01-run-20261121 bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121): a Sokoban solver for the competition's generated levels.

The generator's levels share one shape — a one-square-wide hallway across the board with the player in it, and rooms of 3-4 boxes hanging off it through single doors — and a level's push optimum is the sum of its rooms' optima. `main.ts` exploits exactly that: it splits the board at the player's row into room components and solves each room with a push-based A* (cost = pushes) using an admissible cheapest-matching heuristic over true single-box push distances, dead-square pruning, and player regions canonicalised by flood fill, so each room's solution is push-optimal. Other rooms' boxes are fixed obstacles; pushes stay inside the room and its door squares. All budgets are node counts, never the clock, so runs are byte-identical.

On the kit's practice set it solves all five instances in the known optimal push counts (13, 16, 29, 47 and 60 — the maximum score of 1,000,000) in about a third of a second. If a level ever arrives that is not room-shaped, the same A* runs on the whole board as a fallback.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=<dir of instances> MAKEFUN_OUTPUT_DIR=out node main.ts
```
