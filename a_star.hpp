#pragma once
#include "puzzle.hpp"
#include <vector>
#include <unordered_map>
int h_mnhtn(const Puzzle& puzzle);
int h_mnhtn_inv(const Puzzle& puzzle);
std::vector<Puzzle> reconstuct_path(const std::unordered_map<Puzzle, Puzzle>& cameFrom, Puzzle current);
std::vector<Puzzle> a_star(const Puzzle& start, const Puzzle& goal, int (*heuristic)(const Puzzle&));