# Real-time event simulation with frame-based cameras (library)

This repository contains a library with which event data can be simulated given frames from a frame-based cameras.

#### Citing

If you use this code in an academic context, please cite the following publication:

Paper: [Real-time event simulation with frame-based cameras](https://arxiv.org/pdf/2209.04634.pdf)

Video: [YouTube](https://www.youtube.com/@ZellTuebingen)

Project: [See here](file:///data/repos/event-simulation-project-page/index.html)

```
@inproceedings{Ziegler2023icra,
	title = {Real-time event simulation with frame-based cameras},
	booktitle = {2023 {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
	publisher = {IEEE},
	author = {Ziegler, Andreas and Teigland, Daniel and Tebbe, Jonas and Gossard, Thomas and Zell, Andreas},
	month = {may},
	year = {2023},
}
```

## Using the code

### Software requirements

This code has been tested on Ubuntu 20.04 with ROS noetic.

Dependencies:
- Eigen
- OpenCV
- OpenMP
- CUDA (optional)

### Usage

This library is intended to be used in an application. For an example, please have a look at our [ROS wrapper](https://github.com/cogsys-tuebingen/event_simulator_ros).

## License

This software is issued under the Apache License Version 2.0.
