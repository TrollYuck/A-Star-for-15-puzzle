#include "puzzle.hpp"
#include "a_star.hpp"
#include <iostream>

int const D = 4;
int main() {
	auto game = puzzle(D, false);
	auto goal = puzzle(D, true);
	game.print_current_permutation();
	std::cout << "Is this game solvable?:  " << game.is_solvable() << std::endl;
	goal.print_current_permutation();
	std::cout << "Is this game solvable? (should be, couse it\'s the goal):  " << goal.is_solvable() << std::endl;

	if (game.is_solvable()) {
		//std::vector<int> path_manhattan = a_star_algorithm_moves(game, goal, manhattan_heuristic);
		std::vector<int> path_misplaced = a_star_algorithm_moves(game, goal, misplaced_tiles_heuristic);

		//std::cout << "in how many moves the manhattan took: " << path_manhattan.size() << std::endl;
		std::cout << "in how many moves the inversions took: " << path_misplaced.size() << std::endl;
	}
	else {
		std::cout << "Game is not solvable" << std::endl;
	}

	return 0;
}
