## Datasets
Each dataset consists of a folder structure with individual csv files. 
A dataset is loaded recursively, therefore all csv files in subfolders are included for training.

The csv files are lists of single datapoints, where each datapoint is a vector including
- (*gripperX*, *gripperY*, *gripperRot*, *gripperGrasp*) as velocity label (i.e. user input), 
- (*targetX*, *targetY*) as position of the target circle, and
- (*box1X*, *box1Y*, *box1Rot*) and (*box2X*, *box2Y*, *box2Rot*) as pose of the two blue boxes respectively.

All positions and rotations are listed relative to the gripper, with rotation normalised between -1 and 1, such that a box with rotation=0 is aligned to the gripper.

All csv files represent independant simulation executions. 
Within a csv file, all-zero datapoints are added as separator of individual executions.