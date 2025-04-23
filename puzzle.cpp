#include <iostream>
#include <algorithm>
#include <random>
#include "puzzle.hpp"

Puzzle init_Puzzle() 
{
	Puzzle puzzle{};
	for (size_t i = 0; i < static_cast<unsigned long long>(BOARD_SIZE) - 1; i++)
		{
			puzzle.permutation[i] = i + 1;
		}

	/*std::random_device rd;
	std::mt19937 g(rd());*/
	unsigned int seed = 1234;
	std::mt19937 g(seed);
	std::shuffle(puzzle.permutation, puzzle.permutation + BOARD_SIZE - 1, g);

	puzzle.zero_index = BOARD_SIZE - 1;
	puzzle.permutation[BOARD_SIZE - 1] = 0;

	return puzzle;
}

Puzzle* find_Neighbours(Puzzle puzzle)
{
	static Puzzle neighbours[4]; 
	int zero_idx = puzzle.zero_index;
	int side_size = SIDE_SIZE;
	int neighbor_count = 0;

	// Define possible moves: {row_offset, col_offset}
	int moves[4][2] = {
		{-1, 0}, // Up
		{1, 0},  // Down
		{0, -1}, // Left
		{0, 1}   // Right
	};

	for (int i = 0; i < 4; i++) {
		int new_row = zero_idx / side_size + moves[i][0];
		int new_col = zero_idx % side_size + moves[i][1];

		if (new_row >= 0 && new_row < side_size && new_col >= 0 && new_col < side_size) {
			int new_idx = new_row * side_size + new_col;
			Puzzle neighbor = puzzle;
			std::swap(neighbor.permutation[zero_idx], neighbor.permutation[new_idx]);
			neighbor.zero_index = new_idx;
			neighbours[neighbor_count++] = neighbor;
		}
	}

	return neighbours;
}


int num_of_Inversions(Puzzle puzzle)
{
	int inversions = 0;
	for (size_t i = 0; i < static_cast<unsigned long long>(BOARD_SIZE) - 1; i++)
	{
		if (puzzle.permutation[i] != i + 1)
			inversions++;
	}
	return inversions;
}

bool is_Solvable(Puzzle puzzle)
{
	return (num_of_Inversions(puzzle) % 2) == 0;
}

bool permutate(Puzzle* puzzle, int mv_idx)
{
	if (mv_idx < 0 || mv_idx >= BOARD_SIZE)
		return false;

	int zero_idx = puzzle->zero_index;

	bool is_adjacent = 
        (mv_idx == zero_idx - 1 && zero_idx % SIDE_SIZE != 0) || // Left
        (mv_idx == zero_idx + 1 && (zero_idx + 1) % SIDE_SIZE != 0) || // Right
        (mv_idx == zero_idx - SIDE_SIZE) || // Up
        (mv_idx == zero_idx + SIDE_SIZE);   // Down

    if (!is_adjacent)
        return false;

    std::swap(puzzle->permutation[zero_idx], puzzle->permutation[mv_idx]);

    puzzle->zero_index = mv_idx;

    return true;
}

void print_Puzzle(Puzzle puzzle)
{
	int idx = 0;

	for (size_t i = 0; i < SIDE_SIZE; i++)
	{
		std::cout << "| ";
		for (size_t j = 0; j < SIDE_SIZE; j++) 
		{
			if (puzzle.permutation[idx] < 10)
				std::cout << 0;
			std::cout << puzzle.permutation[idx] << " ";
			idx++;
		}
		std::cout << "|" << std::endl;
	}
}




#include <iterator> // Include this header for std::begin and std::end

int main() {
   Puzzle puzzle = init_Puzzle();
   print_Puzzle(puzzle);
   auto ans = is_Solvable(puzzle) ? "is solvable" : "is NOT solvable";
   std::cout << "This puzzle " << ans << std::endl;

   Puzzle* neighbours = find_Neighbours(puzzle);

   for (int i = 0; i < 4; i++) {
       print_Puzzle(neighbours[i]);
	   std::cout << std::endl;
   }

   return 0;
}