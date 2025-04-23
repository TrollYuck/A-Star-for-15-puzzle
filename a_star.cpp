#include "a_star.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

int h_mnhtn(const Puzzle& puzzle)
{
    int distance = 0;
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (puzzle.permutation[i] != 0) {
            int target_row = (puzzle.permutation[i] - 1) / SIDE_SIZE;
            int target_col = (puzzle.permutation[i] - 1) % SIDE_SIZE;
            int current_row = i / SIDE_SIZE;
            int current_col = i % SIDE_SIZE;
            distance += abs(target_row - current_row) + abs(target_col - current_col);
        }
    }
    return distance;
}

int h_mnhtn_inv(const Puzzle& puzzle)
{
    int inversions = num_of_Inversions(puzzle);
    return h_mnhtn(puzzle) + inversions;
}

std::vector<Puzzle> reconstuct_path(const std::unordered_map<Puzzle, Puzzle>& cameFrom, Puzzle current)
{
    std::vector<Puzzle> total_path = { current };
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom.at(current);
        total_path.insert(total_path.begin(), current);
    }
    return total_path;
}

std::vector<Puzzle> a_star(const Puzzle& start, const Puzzle& goal, int(*heuristic)(const Puzzle&))
{
    auto compare = [](const std::pair<Puzzle, int>& a, const std::pair<Puzzle, int>& b) {
        return a.second > b.second;
        };
    std::priority_queue<std::pair<Puzzle, int>, std::vector<std::pair<Puzzle, int>>, decltype(compare)> openSet(compare);

    openSet.push({ start, heuristic(start) });

    std::unordered_map<Puzzle, Puzzle> cameFrom;
    std::unordered_map<Puzzle, int> gScore;
    std::unordered_map<Puzzle, int> fScore;

    gScore[start] = 0;
    fScore[start] = heuristic(start);

    while (!openSet.empty()) {
        Puzzle current = openSet.top().first;
        openSet.pop();

        if (current.permutation == goal.permutation) {
            return reconstuct_path(cameFrom, current);
        }

        Puzzle* neighbors = find_Neighbours(current);
        for (int i = 0; i < 4; ++i) {
            Puzzle neighbor = neighbors[i];
            if (neighbor.zero_index == -1) continue; // Skip invalid neighbors

            int tentative_gScore = gScore[current] + 1; // Assuming uniform cost for moves
            if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = tentative_gScore + heuristic(neighbor);
                openSet.push({ neighbor, fScore[neighbor] });
            }
        }
    }

    return {}; // Return empty path if no solution
}
