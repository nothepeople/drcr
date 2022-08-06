# DRCR

- We implement Cost-KSP, Lagrangian-KSP, Pulse+, etc., to solve the delay-range constrained routing (DRCR) problem.

- All the test cases are in the "/data/DelayRange" directory.

- Result: Pulse > BidirectionalPulse (Pulse+ with joint delay and cost pruning) > LagrangianKsp > CostKsp

# SrlgDisjoint DRCR

- We implement Cost-KSP, Lagrangian-KSP, CoSE-Pulse+, etc., to solve the Srlg-Disjoint DRCR problem.

- All the test cases are in the "/data/SrlgDisjoint" directory.

- large.csv stands for Srlg-DRCR_2 and small.csv stands for Srlg-DRCR_1

- Result: CoSE-Pulse+ > CostKsp > LagrangianKsp

# Run Tests

There are three ways to run our code.
```make test
./test
```

```make main
./main ./"TopoFilePath" ./"TunnelFilePath" 
```

- In main.cc, you can easily change the method you are going to apply to solve the test instance.

- e.g. if you want to use the Pulse+ to solve DRCR problem, you can set line 53 "LagrangianKsp pulse" to "Pulse pulse" and keep everything else the same.

- it is same when you solve the Srlg-DRCR problem.

```
./batch_run.sh
```

- You may also use **batch_run.sh** to do batch processing. 

  - You need to change the location of **data_dir** to the data directory you want 

  - You may select the **type_name** between SrlgDisjoint and DelayRange.

  - You may edit line 23, for example "LagrangianKsp" to the name you want to use. Let's take the name "LagrangianKsp" for a example, the program will create a folder called "LagrangianKsp" the test results will output to the folder in the working directory

  - You may change the value of **tunnel_name** based on the test case you want. **flow_info** for DRCR problem(you can also use **tunnel.csv**) and **large.csv/small.csv** for Srlg-DRCR problem.**The tunnel_name you select should in accordance with the type_name**
