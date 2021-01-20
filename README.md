# Learning to Map Degrees of Freedom for Assistive User Control
## Towards an Adaptive DoF-Mapping Control for Assistive Robots

Code Repository for Adaptive DoF Mapping

For the publication, please see: [PETRA'21](http://www.petrae.org/)

For the video contribution, please see: [Uni-Bremen Link](http://www.informatik.uni-bremen.de/agebv2/downloads/videos/GoldauPetra21.m4v)

### This repository includes:
- the JavaScript simulation environment including simple python server
- python and ipynb files for training the adaptive controls
  - customLayers
  - custom metrics
  - dataset loading
- two datasets generated in the JS simulation
  - the datasets with square boxes follow the rule: rotate to a box, drive towards it, grasp, rotate to goal, drive forwards, open gripper, repeat  
  - the datasets with square boxes follow the rule: rotate to a box, drive towards it, rotate around it to optimal position, grasp, rotate to goal, drive forwards, open gripper, repeat  
- pretrained models and weights based on the datasets

---
Regarding any questions, please contact the publication authors directly.
