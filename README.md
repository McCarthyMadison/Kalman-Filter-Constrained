# Kalman-Filter-With-Linear-Constraints

This repository contains a reference paper, presentation, and MATLAB simulations for implementing different linearly constrained Kalman filtering algorithms.

Contents of this repository:
1. Reference Paper
2. Preliminary Reference Presentation Slides
3. Matlab Simultation Code

### Prerequisites

MATLAB R2016b (not tested with other versions)

### Files for Simultation

The MGS.m script provides a function used by the MadisonMcCarthy_Kalman_Filter_Constrained.m.

## Running the Simultations

There are two modes for running the simultations:

1. Set the number of iterations for the script to 1 (line 6: for (0=1:1))

2. Start Matlab in no figures mode (matlab -noFigureWindows). Set the number of iterations for the script to 100 (line 6: for (0=1:100))

Additional information that may be useful for understanding the performance is commented out in lines 736 and onwards.

## Author

* **Madison McCarthy** 

## Acknowledgments

* Dan Simon (His MGS.m script is directly used): (http://academic.csuohio.edu/simond/ConstrKF/)
