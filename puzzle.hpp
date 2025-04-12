#pragma once
#include <vector>
#include <functional>
#include <unordered_map>

class puzzle {
public:
	int size;
	std::vector<int> perm_curr;
	std::vector<int> perm_prev;
	int zero_position[2];
	std::vector<int> permute(int x, int y);
	std::vector<puzzle> get_neighbors() const;
	void restore_previous_permutation();
	void print_current_permutation();
	bool is_solvable() const;
	bool operator==(const puzzle& other) const;
	bool operator<(const puzzle& other) const;
	puzzle();
	puzzle(int size, bool is_goal);
};

int number_of_inversions(const puzzle& puzzle);

struct puzzle_hash {
	size_t operator()(const puzzle& puzzle) const {
		size_t hash = 0;
		for (int tile : puzzle.perm_curr) {
			hash ^= std::hash<int>{}(tile)+0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};

struct puzzle_compare {
	std::unordered_map<puzzle, int, puzzle_hash>& fScore;
	puzzle_compare(std::unordered_map<puzzle, int, puzzle_hash>& f) : fScore(f) {}

	bool operator()(const puzzle& a, const puzzle& b) const {
		return fScore.at(a) > fScore.at(b); // Min-heap behavior
	}
};