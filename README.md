# DRCR

We implement many algorithms, including KSP, Lagrangian-Dual, Pulse+, etc., to solve the delay-range constrained routing (DRCR) problem.

All the test cases are in the "/data" directory.

# run tests
make test
./test

# DRCR test result
Class Pulse implements "Pulse+".

Class BidirectionalPulse implements "Pulse+ with joint delay and cost pruning"

For DRCR cases: Pulse > BidirectionalPulse > LagrangianKsp > CostKsp > DelayKsp (">" means "performs better")

# srlg-disjoint DRCR test result
CosePulse > SrlgDisjointPulse > Pulse > LagrangianKsp > CostKsp > DelayKsp

# DelayRange

- DelayRange is the folder that contains the test instances of DRCR

# README.md

- README.md is current file

# SrlgDisjoint

- SrlgDisjoint is the folder that contains the test instances of Srlg-DRCR
- large.csv stands for Srlg-DRCR_2 and small.csv stands for Srlg-DRCR_1

# Code

- Code is the folder of our main code

- you can run **make main** to compile the program and then use

  ```c++
   ./main ./"TopoFilePath" ./"TunnelFilePath" 
  ```

  to run experiment

- In main.cc, you can easily change the method you are going to apply to solve the test instance.

- e.g. if you want to use the Pulse+ to solve DRCR problem, you can set line 53 "LagrangianKsp pulse" to "Pulse pulse" and keep everything else the same.

- it is same when you are going to slove Srlg-DRCR problem.

- You may also use **test.sh** to do batch processing. 

  - You need to change the location of **data_dir** to the data directory you want 
  - You may edit line 23, for example "LagrangianKsp" to the name you want to use, the test results will output to the directory with name "LagrangianKsp" under the working directory
  - You may change the value of **tunnel_name** based on the test case you want. **flow_info.csv** for DRCR problem and **large.csv/small.csv** for Srlg-DRCR problem.

- If you want to get into the detail of our algorithm, you can check the annotation in our code or turn to our paper


# Data Processing

## check_res.cc

- based on the result of solving, analyze the result of srlg_log.csv and link_log.csv to give some basic information. - - The output includes four parts: average_ap_time, average_bp_time, average_total_time, average_optimality

## process_flow.cc & special_paths.cc & preprocess.sh
- special_paths.cc is used to get the min_delay_path and min_cost_path based on Algorithm.h
- process_flow.cc is designed for special_paths.cc to read in data about the graph
- preprocess.sh is a script to dig into information about flows automatically

## split.cc
- split the flows from a single tunnel.csv into several separate files(one flow each file) to enable system halt(I find this part mission, perhaps need to have a look later)

## make_dataset.cc & make_data.sh

- make_dataset.cc change the range of delay_lower_bound and delay_upped_bound as you want
- make_data.sh is a script that make the data according to the rules in make_dataset.cc

## rename.sh (abandoned)
- rename and reconstruct the structure of directory "./data"

## classify.cc

- after getting the result of test cases, you can use

  ```
  g++ -o clssify classify.cc --std=c++11
  ./classify ./FileName
  ```

- The program will get the average iteration and average time and print to "FileName"_log.log

- It will also sort the data according to time and print to "FileName"_time.log, and it will modify the data format to meet the requirement of the program of python to draw pdf plots

- if you want to see the performance of different DRCR Cases, you can find the "classify" function and then employ get_average and get_time to get the related data of the cases.

## get_pdf.cc

- It can calculate the data needed by pdf
- the input and output may be optimized

# draw_pic_functions

- draw_pic_functions is the folder that contains some functions used to draw plots in our paper

# experiment_data

- experiment_data contains the data we have achieved in the experiment the evaluation of these data can be seen in our paper



