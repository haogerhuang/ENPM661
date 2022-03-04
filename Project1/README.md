# ENPM661 Project1

## Requirement
This code requires numpy, argparse, heapq

## Run code

Breadth First Search
```bash
python puzzle.py -i sample_input.txt -m bfs
```

A Star Search 
```bash
python puzzle.py -i sample_input.txt -m a*
```

sample_input.txt can be replaced by any text file that contains two lines, e.g.:
1 4 7 5 0 8 2 3 6
1 4 7 2 5 8 3 6 0
Which the first line is the start state and the second line is the goal state. Both follows the column-wise format.

