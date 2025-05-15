# 15-Puzzle Solver in Go 

This repository contains an implementation of the **15-puzzle solver** using the **A\*** and **IDA\*** search algorithms in Go, supporting two heuristics: Manhattan distance and Misplaced tiles. It can solve both 3×3 and 4×4 puzzles and generate solvable puzzles either by backward random moves or full randomization.

---

## Features

- Supports both **3×3** and **4×4** sliding puzzles  
- Implements **IDA\*** (Iterative Deepening A*) for memory efficiency  
- Two heuristics:
  - Misplaced Tiles
  - Manhattan Distance
- Multiple puzzles can be solved in a batch for statistical analysis
-  Tracks and averages:
  - Execution time
  - States visited
  - Steps to solution

---

##  Usage

```bash
go run main.go [flags]
```

### Flags

| Flag       | Description                                                    | Example     |
|------------|----------------------------------------------------------------|-------------|
| `-dim`     | Puzzle dimension: `3` or `4`                                   | `-dim=4`    |
| `-h`       | Heuristic function: `1` for Misplaced, `2` for Manhattan       | `-h=2`      |
| `-rand`    | Number of random backward moves, or `-1` for full random       | `-rand=30`  |
| `-n`       | Number of puzzles to solve (for averaging stats)               | `-n=10`     |

### Example

```bash
go run main.go -dim=4 -h=2 -rand=30 -n=5
```

This command:
- Solves 5 random 4×4 puzzles
- Uses the Manhattan distance heuristic
- Generates puzzles by making 30 random valid moves from the solved state

---

## Sample Output

For a single puzzle:
```
Puzzle # 1
 1  2  3  4 
 5  6  7  8 
 9 10 11 12 
13 14    15 

Visited: 84, Steps: 6
Moves: [15 14 13 14 15 0]
Time: 537.308µs
```

For multiple puzzles:
```
Average over 5 puzzles:
  Visited states: 121.40
  Solution length: 10.20 moves
  Execution time: 1.43ms
```

---
