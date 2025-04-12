#pragma once
#include "puzzle.hpp"
#include <unordered_map>
#include <vector>

int manhattan_heuristic(const puzzle& puzzle);
int misplaced_tiles_heuristic(const puzzle& p);
std::vector<puzzle> reconstruct_path(std::unordered_map<puzzle, puzzle, puzzle_hash>& cameFrom, puzzle current);
std::vector<int> reconstruct_moves(std::unordered_map<puzzle, puzzle, puzzle_hash>& cameFrom, puzzle current);
std::vector<int> a_star_algorithm_moves(puzzle start, puzzle goal, std::function<int(const puzzle&)> heuristic);

