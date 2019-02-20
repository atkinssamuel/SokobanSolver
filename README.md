# Sokoban Solver
This project implements a heuristic function for sokoban: https://www.sokobanonline.com/ 

## Running the Solution:
To run the solution, edit the True/False values of the various tests at the top of the autograder.py file, then run autograder.py. 

To run one of the problems, copy the following into the main block of solution.py:
`anytime_gbfs(PROBLEMS[i], heur_alternate, timebound = 10)

weight = 10

anytime_weighted_astar(PROBLEMS[i], heur_alternate, weight, timebound = 10)`

Replace i with an integer, 0, 1, 2, 3, ..., 19 for which problem to run. Higher integers correspond with more difficult problems.
