# ENPM661 Project3-phase2-part1 A-star on a differential drive turtlebot 

## Team Members
117517842 Pin-Hao Huang
118307435 Po-Lun Chen

## Github Link
https://github.com/haogerhuang/ENPM661/tree/main/Project3_phase2_part1

## Requirement

- numpy==1.21
- opencv==4.5
- argparse
- tqdm
- heap

## Run code

Example 1:
```bash
python3 astar_search.py -s 1 1 0 -g 9 9 -c 0.1 -rpm 15 50
```

Example 2:
```bash
python3 astar_search.py -s 1 1 0 -g 1 7 -c 0.1 -rpm 15 50
```

Arguments:
```bash
-s: start position, should be three values: x [meter], y [meter], theta [degree]
-g: goal position, should be two values: x [meter], y [meter]
-c: clearance value, default: 0.1 [meter]
-rpm: RPMs to construct actions: rpm1 [rpm], rpm2 [rpm]
-f: file name to store the visualization video, default: 'visualize'
```


