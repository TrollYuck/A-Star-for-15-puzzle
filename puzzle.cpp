#include "puzzle.hpp"  
#include <iostream>  
#include <algorithm> 
#include <vector>
#include <random> 
#include <stdexcept>
#include <sstream>

puzzle::puzzle() : size(0), zero_position{ 0, 0 } {}
 
puzzle::puzzle(int size, bool is_goal) : size(size), zero_position{ 0, 0 } {
    // Allocate memory for the board
    int totalValues = size * size;
    perm_curr.resize(totalValues);
    perm_prev.resize(totalValues);

    // Create an array to hold values from 1 to size * size - 1
    std::vector<int> values(totalValues - 1);
    for (int i = 0; i < totalValues - 1; ++i) {
        values[i] = i + 1;
    }

    if (!is_goal) {
        // Shuffle the array
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(values.begin(), values.end(), g);
    }

    // Fill the board with values
    for (int i = 0; i < totalValues - 1; ++i) {
        perm_curr[i] = values[i];
    }
    perm_curr[totalValues - 1] = 0;


    zero_position[0] = (totalValues - 1) / size;
    zero_position[1] = (totalValues - 1) % size;

    perm_prev = perm_curr;
}

std::vector<int> puzzle::permute(int x, int y) {
    if (x >= size || y >= size) {
        throw std::out_of_range("Index out of bounds in permute function");
    }

    int zero_x = zero_position[0];
    int zero_y = zero_position[1];
    int manhattan_distance = abs(zero_x - x) + abs(zero_y - y);

    if (manhattan_distance != 1) {
        throw std::invalid_argument("The position is not a neighbor of the zero position");
    }

    for (int i = 0; i < size * size; ++i) {
        perm_prev[i] = perm_curr[i];
    }

    std::swap(perm_curr[static_cast<std::vector<int, std::allocator<int>>::size_type>(zero_x) * size + zero_y], 
        perm_curr[static_cast<std::vector<int, std::allocator<int>>::size_type>(x) * size + y]);

    return perm_curr;
}

std::vector<puzzle> puzzle::get_neighbors() const {
    std::vector<puzzle> neighbors;
    int zero_x = zero_position[0];
    int zero_y = zero_position[1];

    // Possible moves: Up, Down, Left, Right
    std::vector<std::pair<int, int>> moves = {
        {zero_x - 1, zero_y}, // Up
        {zero_x + 1, zero_y}, // Down
        {zero_x, zero_y - 1}, // Left
        {zero_x, zero_y + 1}  // Right
    };

    for (const auto& move : moves) {
        if (move.first >= 0 && move.first < size && move.second >= 0 && move.second < size) {
            puzzle new_puzzle = *this; // Copy current state
            new_puzzle.permute(move.first, move.second);
            new_puzzle.zero_position[0] = move.first;
            new_puzzle.zero_position[1] = move.second;
            neighbors.push_back(new_puzzle);
        }
    }

    return neighbors;
}

void puzzle::restore_previous_permutation() {
    perm_curr = perm_prev;
}

void puzzle::print_current_permutation() {
    std::cout << "|";
    for (int i = 0; i < size * size; i++) {
        if (i % size == 0 && i != 0)
            std::cout << "|" << std::endl << "|";
        if (perm_curr[i] / 10 != 0 && perm_curr[i] != 0 || perm_curr[i] == 10) {
            std::cout << " " << perm_curr[i] << " ";
        }
        else {
            std::cout << " 0" << perm_curr[i] << " ";
        }
        
    }
    std::cout << "|" << std::endl;
}

bool puzzle::is_solvable() const {
    return (number_of_inversions(*this) % 2 == 0);
}

bool puzzle::operator==(const puzzle& other) const {
    return perm_curr == other.perm_curr;
}

bool puzzle::operator<(const puzzle& other) const {
    return number_of_inversions(*this) < number_of_inversions(other); // For priority queue sorting
}

int number_of_inversions(const puzzle& puzzle) {
    int inversions = 0;
    for (int i = 0; i < puzzle.size * puzzle.size; i++) {
        if (puzzle.perm_curr[i] != 0 && puzzle.perm_curr[i] != i + 1) {
            inversions++;
        }
    }
    return inversions;
}