package main

import (
	"container/heap"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"strconv"
	"strings"
	"time"
)

type State struct {
	board     []int  // flat array of length n*n, 0 represents blank
	heuristic int    // chosen heuristic: 1=misplaced, 2=Manhattan
	gn        int    // cost so far
	hn        int    // heuristic estimate
	fn        int    // fn = gn + hn
	prev      *State // pointer to previous state
	move      int    // tile moved to reach here
	blank     int    // index of blank
}

// Priority queue for A*
type PriorityQueue []*State

func (pq PriorityQueue) Len() int           { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].fn < pq[j].fn }
func (pq PriorityQueue) Swap(i, j int)      { pq[i], pq[j] = pq[j], pq[i] }
func (pq *PriorityQueue) Push(x any)        { *pq = append(*pq, x.(*State)) }
func (pq *PriorityQueue) Pop() any {
	n := len(*pq)
	item := (*pq)[n-1]
	*pq = (*pq)[:n-1]
	return item
}

// isSolvable returns true if the puzzle of dimension n and state board
// (with 0 in the bottom right corner) is solvable.
// For odd width (e.g., 3x3): the number of inversions must be even.
// For even width (e.g., 4x4) and the blank in the last row,
// the rule boils down to the parity of inversions (because the row of the blank = 1 from the end).
func isSolvable(board []int, n int) bool {
	inv := 0
	for i := 0; i < len(board); i++ {
		if board[i] == 0 {
			continue
		}
		for j := i + 1; j < len(board); j++ {
			if board[j] != 0 && board[j] < board[i] {
				inv++
			}
		}
	}

	if n%2 == 1 {
		// odd dimension: even inversions → solvable
		return inv%2 == 0
	} else {
		// even dimension: solvability depends on the row of the blank tile
		// (counted from the bottom) and the parity of inversions.
		blankIdx := findBlank(board)

		blankRowFromBottom := n - (blankIdx / n)

		return inv%2 != blankRowFromBottom%2
	}
}

// Heuristic: count of misplaced tiles
func misplaced(board []int) int {
	count := 0
	for i, v := range board {
		if v != 0 && v != i+1 {
			count++
		}
	}
	return count
}

// Heuristic: Manhattan distance
func manhattan(board []int, n int) int {
	d := 0
	for i, v := range board {
		if v == 0 {
			continue
		}
		// target position
		t := v - 1
		xi, yi := i/n, i%n
		xt, yt := t/n, t%n
		d += abs(xi-xt) + abs(yi-yt)
	}
	return d
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// Generate neighbors by sliding tiles into blank
func neighbors(s *State, n int) []*State {
	var res []*State
	x := s.blank / n
	y := s.blank % n
	dirs := []int{-1, 1, -n, n} // left, right, up, down in flat index
	for _, d := range dirs {
		ni := s.blank + d
		// check move validity
		if (d == -1 && y == 0) || (d == 1 && y == n-1) || (d == -n && x == 0) || (d == n && x == n-1) {
			continue
		}
		if ni < 0 || ni >= len(s.board) {
			continue
		}
		// swap blank and tile
		newB := make([]int, len(s.board))
		copy(newB, s.board)
		newB[s.blank], newB[ni] = newB[ni], newB[s.blank]
		h := 0
		if s.heuristic == 1 {
			h = misplaced(newB)
		} else {
			h = manhattan(newB, n)
		}
		ns := &State{board: newB, heuristic: s.heuristic, gn: s.gn + 1, hn: h, fn: s.gn + 1 + h, prev: s, move: newB[s.blank], blank: ni}
		res = append(res, ns)
	}
	return res
}

// A* search
func astar(start []int, heuristic int, n int) (goal *State, visited int) {
	// initial state
	h0 := 0
	if heuristic == 1 {
		h0 = misplaced(start)
	} else {
		h0 = manhattan(start, n)
	}
	init := &State{board: start, heuristic: heuristic, gn: 0, hn: h0, fn: h0, prev: nil, move: -1, blank: findBlank(start)}

	pq := &PriorityQueue{init}
	heap.Init(pq)
	// visited map: board signature to best g
	visitedMap := make(map[string]int)
	visitedMap[key(start)] = 0

	for pq.Len() > 0 {
		s := heap.Pop(pq).(*State)
		visited++
		// check goal
		if isGoal(s.board) {
			return s, visited
		}
		for _, nb := range neighbors(s, n) {
			k := key(nb.board)
			if g, ok := visitedMap[k]; !ok || nb.gn < g {
				visitedMap[k] = nb.gn
				heap.Push(pq, nb)
			}
		}
	}
	return nil, visited
}

func idaStar(start []int, heuristic int, n int) (goal *State, visited int) {
	h0 := 0
	if heuristic == 1 {
		h0 = misplaced(start)
	} else {
		h0 = manhattan(start, n)
	}
	root := &State{board: start, heuristic: heuristic, gn: 0, hn: h0, fn: h0, prev: nil, move: -1, blank: findBlank(start)}

	threshold := root.fn
	for {
		visited = 0
		var found *State
		var nextThreshold = -1

		var dfs func(s *State, g, threshold int, visitedMap map[string]int) int
		dfs = func(s *State, g, threshold int, visitedMap map[string]int) int {
			visited++
			f := g + s.hn
			if f > threshold {
				if nextThreshold == -1 || f < nextThreshold {
					nextThreshold = f
				}
				return 0
			}
			if isGoal(s.board) {
				found = s
				return 1
			}
			visitedMap[key(s.board)] = g
			for _, nb := range neighbors(s, n) {
				k := key(nb.board)
				if vg, ok := visitedMap[k]; ok && nb.gn >= vg {
					continue
				}
				if dfs(nb, g+1, threshold, visitedMap) == 1 {
					return 1
				}
			}
			return 0
		}

		visitedMap := make(map[string]int)
		if dfs(root, 0, threshold, visitedMap) == 1 {
			return found, visited
		}
		if nextThreshold == -1 {
			break
		}
		threshold = nextThreshold
	}
	return nil, visited
}

func findBlank(board []int) int {
	for i, v := range board {
		if v == 0 {
			return i
		}
	}
	return -1
}

func isGoal(board []int) bool {
	for i := 0; i < len(board)-1; i++ {
		if board[i] != i+1 {
			return false
		}
	}
	return board[len(board)-1] == 0
}

func key(board []int) string {
	b := make([]byte, len(board)*2)
	for i, v := range board {
		b[2*i] = byte(v >> 8)
		b[2*i+1] = byte(v)
	}
	return string(b)
}

// generate solvable puzzle by k random moves from goal
type Puzzle struct {
	board []int
	n     int
	blank int
}

func printBoard(board []int, n int) {
	for i, v := range board {
		if v == 0 {
			fmt.Printf("   ")
		} else {
			fmt.Printf("%2d ", v)
		}
		if (i+1)%n == 0 {
			fmt.Println()
		}
	}
}

func (p *Puzzle) randomize(moves int) {
	size := p.n * p.n

	if moves >= 0 {
		for i := 0; i < moves; i++ {
			nbs := neighbors(&State{board: p.board, heuristic: 1, gn: 0, hn: 0, fn: 0, blank: p.blank}, p.n)
			nxt := nbs[rand.Intn(len(nbs))]
			p.board = nxt.board
			p.blank = nxt.blank
		}
		return
	}

	perm := make([]int, size-1)
	for i := range perm {
		perm[i] = i + 1
	}

	for {
		rand.Shuffle(len(perm), func(i, j int) {
			perm[i], perm[j] = perm[j], perm[i]
		})

		for i, v := range perm {
			p.board[i] = v
		}
		p.board[size-1] = 0
		p.blank = size - 1

		if isSolvable(p.board, p.n) {
			return
		}
		fmt.Println("Board: ")
		printBoard(p.board, p.n)
		fmt.Println("is NOT solvable")
	}
}

func newPuzzle(n, randMoves int) []int {
	// start from goal
	board := make([]int, n*n)
	for i := 0; i < n*n-1; i++ {
		board[i] = i + 1
	}
	board[n*n-1] = 0
	p := &Puzzle{board: board, n: n, blank: n*n - 1}
	p.randomize(randMoves)
	return p.board
}

// reconstruct path of moves
func reconstruct(goal *State) []int {
	var path []int
	for s := goal; s.prev != nil; s = s.prev {
		path = append([]int{s.move}, path...)
	}
	return path
}

func parseBoardString(boardStr string) ([]int, error) {
	parts := strings.Split(boardStr, ",")
	if len(parts) == 0 || (len(parts) == 1 && strings.TrimSpace(parts[0]) == "") {
		return nil, fmt.Errorf("board string is empty or contains only whitespace")
	}
	board := make([]int, len(parts))
	for i, s := range parts {
		val, err := strconv.Atoi(strings.TrimSpace(s))
		if err != nil {
			return nil, fmt.Errorf("invalid number '%s' in board string: %w", s, err)
		}
		board[i] = val
	}
	return board, nil
}

func isValidBoardContent(board []int, n int) bool {
	expectedSize := n * n
	if len(board) != expectedSize {
		// This case should ideally be caught before calling this function
		// by checking if len(board) is a perfect square.
		return false
	}

	seen := make(map[int]bool)
	for _, val := range board {
		if val < 0 || val >= expectedSize { // Numbers must be 0 to n*n-1
			return false
		}
		if seen[val] { // Check for duplicates
			return false
		}
		seen[val] = true
	}
	// Check if all numbers from 0 to n*n-1 are present
	return len(seen) == expectedSize
}

func main() {
	// flags
	dimFlag := flag.Int("dim", 4, "Dimension of puzzle (e.g., 3 for 3x3, 4 for 4x4). Used if -b is not provided.")
	heurFlag := flag.Int("h", 2, "Heuristic: 1=misplaced, 2=Manhattan")
	rmovesFlag := flag.Int("rand", 30, "Number of random moves to generate puzzle (if -b not used), -1 for full random puzzle")
	nTestsFlag := flag.Int("n", 1, "Number of puzzles to solve for stats")
	boardStrFlag := flag.String("b", "", "Board configuration as comma-separated numbers (e.g., \"1,2,3,4,5,6,7,8,0\"). Overrides -dim and -rand.")
	flag.Parse()

	var boardToSolve []int
	var n int // Actual dimension of the puzzle
	boardProvidedByFlag := false

	if *boardStrFlag != "" {
		boardProvidedByFlag = true
		parsedBoard, err := parseBoardString(*boardStrFlag)
		if err != nil {
			fmt.Printf("Error parsing board from -b flag: %v\n", err)
			return
		}

		sqrtLen := math.Sqrt(float64(len(parsedBoard)))
		if sqrtLen != math.Floor(sqrtLen) || len(parsedBoard) == 0 {
			fmt.Println("Error: Board from -b flag must have a length that is a perfect square and non-zero (e.g., 9 for 3x3, 16 for 4x4).")
			return
		}
		n = int(sqrtLen)

		if *dimFlag != n && *dimFlag != 0 { // Check if -dim was set and differs
			fmt.Printf("Warning: Dimension derived from -b flag (%d) is used, overriding -dim value (%d).\n", n, *dimFlag)
		}
		// Update dimFlag to reflect the actual dimension being used, though 'n' is the primary variable now.
		*dimFlag = n

		if !isValidBoardContent(parsedBoard, n) {
			fmt.Printf("Error: Board content from -b flag is invalid for an %dx%d puzzle. It must contain all numbers from 0 to %d exactly once.\n", n, n, n*n-1)
			return
		}

		boardToSolve = parsedBoard
		if !isSolvable(boardToSolve, n) {
			fmt.Println("The board configuration provided via -b is not solvable:")
			printBoard(boardToSolve, n)
			return
		}
		if *nTestsFlag > 1 {
			fmt.Printf("Warning: Solving the same board provided by -b flag %d times.\n", *nTestsFlag)
		}
	} else {
		// No -b flag, use -dim and -rand to generate puzzle(s)
		if *dimFlag != 3 && *dimFlag != 4 {
			fmt.Println("Dimension (-dim) must be 3 or 4 when -b is not used.")
			return
		}
		n = *dimFlag
		// Puzzle will be generated inside the loop if nTests > 1
	}

	totalVisited := 0
	totalSteps := 0
	var totalDuration time.Duration

	for i := 0; i < *nTestsFlag; i++ {
		var currentPuzzle []int
		if boardProvidedByFlag {
			currentPuzzle = boardToSolve // Use the same board from -b for all tests
		} else {
			currentPuzzle = newPuzzle(n, *rmovesFlag) // Generate a new puzzle for each test
		}

		fmt.Println("Puzzle #", i+1)
		printBoard(currentPuzzle, n)

		startTime := time.Now()
		goal, visited := idaStar(currentPuzzle, *heurFlag, n)
		elapsed := time.Since(startTime)
		totalDuration += elapsed

		if goal == nil {
			fmt.Println("No solution found for puzzle #", i+1)
			// Decide if you want to continue or stop if a puzzle is unsolvable
			// (though isSolvable should prevent this for generated/valid input)
			continue
		}

		path := reconstruct(goal)
		fmt.Printf("Visited: %d, Steps: %d\n", visited, len(path))
		if *nTestsFlag == 1 { // Show moves only for a single test run
			fmt.Println("Moves:", path)
		}
		fmt.Printf("Time: %s\n\n", elapsed)

		totalVisited += visited
		totalSteps += len(path)
	}

	if *nTestsFlag > 1 {
		avgVisited := float64(totalVisited) / float64(*nTestsFlag)
		avgSteps := float64(totalSteps) / float64(*nTestsFlag)
		avgTime := totalDuration / time.Duration(*nTestsFlag)

		fmt.Printf("Average over %d puzzles:\n", *nTestsFlag)
		fmt.Printf("  Visited states: %.2f\n", avgVisited)
		fmt.Printf("  Solution length: %.2f moves\n", avgSteps)
		fmt.Printf("  Execution time: %s\n", avgTime)
	}
}
