#!/bin/bash
# filepath: /home/trollyuck/sem4/wsi/lista2/genstats.bash

# Compile the Go program
go build -o a_star a_star.go

# Create the stats folder if it doesn't exist
STATS_DIR="stats"
mkdir -p "$STATS_DIR"

# Output CSV file
OUTPUT_FILE="$STATS_DIR/stats.csv"

# Number of puzzles to generate for each configuration
NUM_PUZZLES=1000

# Dimensions to test
DIMENSIONS=(4)

# Heuristics to compare
HEURISTICS=(1 2)

# Random moves for puzzle generation
RANDOM_MOVES=30

# Write CSV header
echo "dimension,heuristic,steps,visited,created,time_ns" > "$OUTPUT_FILE"

# Run experiments
for dim in "${DIMENSIONS[@]}"; do
  for heuristic in "${HEURISTICS[@]}"; do
    for ((i = 1; i <= NUM_PUZZLES; i++)); do
      # Run the program and capture the output
      OUTPUT=$(./a_star -dim "$dim" -h "$heuristic" -rand "$RANDOM_MOVES" -n 1 2>/dev/null)

      # Extract relevant data from the output
      STEPS=$(echo "$OUTPUT" | grep -oP "Steps: \K\d+")
      VISITED=$(echo "$OUTPUT" | grep -oP "Visited: \K\d+")
      TIME=$(echo "$OUTPUT" | grep -oP "Time: \K[\d.]+")
      CREATED=$((VISITED + STEPS)) # Approximation for created states


      # Write data to CSV
      echo "$dim,$heuristic,$STEPS,$VISITED,$CREATED,$TIME_NS" >> "$OUTPUT_FILE"
    done
  done
done

echo "Statistics collected in $OUTPUT_FILE. You can now analyze the data in Python."