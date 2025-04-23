#pragma once
#include <array>
#include <functional> 

constexpr int SIDE_SIZE = 3;
constexpr int BOARD_SIZE = SIDE_SIZE * SIDE_SIZE;

struct Puzzle
{
	int permutation[BOARD_SIZE];
	int zero_index;
};

inline bool operator==(const Puzzle& lhs, const Puzzle& rhs) {
	return std::equal(std::begin(lhs.permutation), std::end(lhs.permutation), std::begin(rhs.permutation)) &&
		lhs.zero_index == rhs.zero_index;
}

template <>
struct std::hash<Puzzle> {
    size_t operator()(const Puzzle& puzzle) const {
        size_t hash_value = 0;
        for (int i = 0; i < BOARD_SIZE; ++i) {
            hash_value ^= std::hash<int>()(puzzle.permutation[i]) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        }
        hash_value ^= std::hash<int>()(puzzle.zero_index) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        return hash_value;
    }
};

Puzzle init_Puzzle();
Puzzle init_Goal();
Puzzle* find_Neighbours(Puzzle puzzle);
int num_of_Inversions(Puzzle puzzle);
bool is_Solvable(Puzzle puzzle);
bool permutate(Puzzle* puzzle, int mv_idx);
void print_Puzzle(Puzzle puzzle);