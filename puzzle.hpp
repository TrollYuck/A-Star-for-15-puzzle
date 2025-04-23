#pragma once
constexpr int SIDE_SIZE = 3;
constexpr int BOARD_SIZE = SIDE_SIZE * SIDE_SIZE;

struct Puzzle
{
	int permutation[BOARD_SIZE];
	int zero_index;
};

Puzzle init_Puzzle();
Puzzle* find_Neighbours(Puzzle puzzle);
int num_of_Inversions(Puzzle puzzle);
bool is_Solvable(Puzzle puzzle);
bool permutate(Puzzle* puzzle, int mv_idx);
void print_Puzzle(Puzzle puzzle);