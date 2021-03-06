## Path Planning Project

This is my seventh project of [Self-Driving Car Engineer nanodegree program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) in udacity.

## Table of Contents

- [Path Planning Project](#path-planning-project)
- [Table of Contents](#table-of-contents)
- [Overview](#overview)
- [Dependencies](#dependencies)
- [Build steps](#build-steps)
- [Path planner pipeline](#path-planner-pipeline)
  - [1. Set initial lane and velocity](#1-set-initial-lane-and-velocity)
  - [2. Update the car's (s, d)Frenet Coordinates.](#2-update-the-cars-s-dfrenet-coordinates)
  - [3. See which lane the car can change](#3-see-which-lane-the-car-can-change)
  - [4 Detect if the car ahead is too close](#4-detect-if-the-car-ahead-is-too-close)
  - [5. Consider lane Change](#5-consider-lane-change)
  - [6. Increase / Decrease velocity](#6-increase--decrease-velocity)
  - [7. Expolarate points for a smooth transition](#7-expolarate-points-for-a-smooth-transition)
- [References](#references)
- [Author](#author)

---

## Overview

The goals / steps of this project are the following:

- Build a path planner using  its location, its sensor fusion data, and highway map data
- Safely navigate around a virtual highway with other traffic
- Summarize the results

This repo includes the following files.

| File     | Description |
|:--------:|:-----------:|
| [main.cpp](./src/main.cpp) | the script to compile this project|
| [spline.h](./src/spline.h)| the script to designe to extrapolate linearly|
| [cost.h](./src/cost.h)| the script to calculate cost for each tragectory|
|README.md| this file, a summary of the project|

The original [spline](./src/spline.h) library can be found at [this link](https://github.com/ttk592/spline/).

[//]: # (Image References)

[image0]: ./examples/run1.gif "Expected output"

---

## Dependencies

This project requires:

- [Udacity self-driving car simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

## Build steps

1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./path_planning.

## Path planner pipeline

### 1. Set initial lane and velocity

I only considered 3 lanes in this project, the first lane is 0, the second is 1, and the third one is 2.

The initial lane is the second lane on the right side, so I set it to 1. The initial velocity is 0.

```c++
// main.cpp

int lane = 1;
double ref_vel = 0.;
```

### 2. Update the car's (s, d)Frenet Coordinates.

If the car has previous path data, I set the last path (s, d) data as a current one for the smooth transition. It can work especially when higher latency is involved.

### 3. See which lane the car can change

First of all, I assumed that getting a trajectory in this project is the finite state machine. I considered 3 lanes in this highway simulator.

The car can change lanes based on its current lane. For example, if the car is currently in the first lane(0), it can change the second lane(1) or don't change(0).

```c++
//main.cpp

vector<int> available_lanes;
// The case of keeping forward
available_lanes.push_back(lane);
// Considers 3 cases.
if (lane == 0) {
  available_lanes.push_back(1);
} else if (lane == 1) {
  available_lanes.push_back(0);
  available_lanes.push_back(2);
} else if (lane == 2) {
  available_lanes.push_back(1);
}
```

### 4 Detect if the car ahead is too close

The car uses sensor fusion data to detect if a car ahead is too close or not. The car considers only if that tracked car is ahead of it. The distance between the tracked car and the car should be more than `30m` that buffer are set as `BUFFER_BETWEEN_CAR` in [cost.h](./src/cost.h).

```c++
// cost.h

const int BUFFER_BETWEEN_CAR = 30;
```

### 5. Consider lane Change

The considers lane changes only when a car ahead is too close. If the tracked car is too close, the decrease in its velocity by `- 0.224`.

The car behaves differently given a state and prediction. The car predicts the best trajectory calculating its cost like how the car is unlikely to collide with other traffic etc.

I decided to use 2 cost functions in [cost.h](./src/cost.h). Those 2 functions considered at the same time, and car use its cost to decide if it can change lane or keep forward. Those 2 functions, `collision_cost` and `buffer_cost` are following.

```c++
// cost.h

float collision_cost(double car_s, double check_car_s) {
  if (abs(check_car_s - car_s) < BUFFER_BETWEEN_CAR) {
    return 1.;
  }
  return 0.;
}

float buffer_cost(double car_s, double check_car_s) {
  return logistic(BUFFER_BETWEEN_CAR / abs(check_car_s - car_s));
}
```

On top of that, I set weights for each cost function. Both of the 2 cases, collision and buffer, can be called a safety problem but I prioritize collision more than taking buffer.

```c++
// cost.h

const float COLLISION = pow(10, 6);
const float BUFFER = pow(10, 4);
```

The car considered all possible cases calculating its corresponding cost functions, and then decide its lane change.

The best lane is the most minimum cost lane.

### 6. Increase / Decrease velocity

The car increases its velocity if it is less than `49.5` to keep its maximum speed as much as possible. If other traffic is ahead of the car and it is too close, the car keeps decreasing its velocity unless the car change lane.

### 7. Expolarate points for a smooth transition

I mostly take advantage of the spline library to extrapolate points for a smooth transition. The most of code can be seen from the Udacity lesson.

The path planner should output a list of x and y global map coordinates. I use 50 points to each x and y waypoints. The 50 waypoints contain a few previous paths and the rest is new generated one. The reason for that is to make the car move smoothly regardless of timing differences.

---

## References

- [Spline library](https://kluge.in-chemnitz.de/opensource/spline/)

---

## Author

- [Tsuyoshi Akiyama](https://github.com/Akitsuyoshi)
