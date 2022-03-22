# ENPM661 Project3-phase1 A-star

## Requirement 
## Requirement


- numpy==1.21
- opencv==4.5
- argparse
- tqdm
- heap

## Run code

```bash
python3 pinhaohuang.py -s 50 50 0 -g 90 230 210 -step 5
```

Arguments:
```bash
-s: start position, which should be three values: x, y, theta
-g: goal position, which should be three values: x, y, theta
-c: clearance value, default: 5
-r: radius of robot, default: 10
-step: step size for searching, default: 1
-f: file name to store the visualization video, default: 'visualize'
```
