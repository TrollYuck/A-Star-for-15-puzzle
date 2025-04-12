#include "puzzle.hpp"
#include "a_star.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <unordered_set>
#include <iostream>
#include <chrono>

int manhattan_heuristic(const puzzle& puzzle)
{
    int distance = 0;
    int size = puzzle.size;
    for (int i = 0; i < size * size; i++) {
        if (puzzle.perm_curr[i] == 0) continue; // Skip the blank tile
        int goal_x = (puzzle.perm_curr[i] - 1) / size;
        int goal_y = (puzzle.perm_curr[i] - 1) % size;
        int curr_x = i / size;
        int curr_y = i % size;
        distance += abs(curr_x - goal_x) + abs(curr_y - goal_y);
    }
    return distance;
}

int misplaced_tiles_heuristic(const puzzle& puzzle) {
    return number_of_inversions(puzzle);
}

std::vector<puzzle> reconstruct_path(std::unordered_map<puzzle, puzzle, puzzle_hash>& cameFrom, puzzle current)
{
    std::vector<puzzle> total_path = { current };
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom[current];
        total_path.insert(total_path.begin(), current);
    }
    return total_path;
}

std::vector<int> reconstruct_moves(std::unordered_map<puzzle, puzzle, puzzle_hash>& cameFrom, puzzle current) {
    std::vector<int> moves;

    while (cameFrom.find(current) != cameFrom.end()) {
        puzzle prev = cameFrom[current];

        int zero_x_prev = prev.zero_position[0];
        int zero_y_prev = prev.zero_position[1];
        int zero_x_curr = current.zero_position[0];
        int zero_y_curr = current.zero_position[1];

        // Find the tile that moved into the zero space
        int moved_tile = prev.perm_curr[zero_x_curr * current.size + zero_y_curr];

        std::cout << moved_tile << " | ";
        moves.push_back(moved_tile);

        current = prev;
    }

    std::reverse(moves.begin(), moves.end()); // Reverse to show moves in order
    return moves;
}

std::vector<int> a_star_algorithm_moves(puzzle start, puzzle goal, std::function<int(const puzzle&)> heuristic) {
    using pq_element = std::pair<int, puzzle>; // (fScore, puzzle)
    std::priority_queue<pq_element, std::vector<pq_element>, std::greater<pq_element>> openSet;

    std::unordered_map<puzzle, puzzle, puzzle_hash> cameFrom;
    std::unordered_map<puzzle, int, puzzle_hash> gScore;
    std::unordered_map<puzzle, int, puzzle_hash> fScore;

    gScore[start] = 0;
    fScore[start] = heuristic(start);
    openSet.push({ fScore[start], start });

    std::unordered_set<puzzle, puzzle_hash> visited; // Store visited states

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    while (!openSet.empty()) {
        puzzle current = openSet.top().second;
        openSet.pop();

        std::cout << "start" << std::endl;
        current.print_current_permutation();

        // Calculate and print elapsed time since the start
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        std::cout << "end (elapsed time: " << elapsed << " seconds)" << std::endl;

        if (current == goal) {
            // Calculate and print total time taken
            auto end_time = std::chrono::high_resolution_clock::now();
            double total_time = std::chrono::duration<double>(end_time - start_time).count();
            std::cout << "Total time: " << total_time << " seconds" << std::endl;

            return reconstruct_moves(cameFrom, current); // Return tile moves
        }

        if (visited.find(current) != visited.end()) continue; // Skip duplicate work
        visited.insert(current);

        for (puzzle neighbor : current.get_neighbors()) {
            int tentative_gScore = gScore[current] + 1;

            if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = tentative_gScore + heuristic(neighbor);
                openSet.push({ fScore[neighbor], neighbor });
            }
        }
    }

    // If we reach here, no solution was found
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();
    std::cout << "No solution found. Total time: " << total_time << " seconds" << std::endl;

    return {}; // Failure case
}


