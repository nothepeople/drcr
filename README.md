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
```c++
make test
./test
```

```c++
make main
./main ./"TopoFilePath" ./"TunnelFilePath" "method_id"
```

- In main.cc, you can easily change the method you are going to apply to solve the test instance by change method_id.

- As for DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 7 for BidirectionalPulse.

- As for Srlg-Disjoint DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 6 for CosePulse+.

```
./batch_run.sh
```

- You may also use **batch_run.sh** to do batch processing. 

  - You may change the location of **data_dir** to the data directory you want 

  - You may select the **type_name** between SrlgDisjoint and DelayRange(the default value is DelayRange)

  - You shall select the method you are going to use by changing the value of **case_id**(the default value is 1)

  - The output of the program will be in the **output_folder** in the working directory. The value of **output_folder** depends on **type_name** and **case_id**(you can check detail information in batch_run.sh)
  
  - You may change the value of **tunnel_name** based on the test case you want. **flow_info** for DRCR problem(you can also use **tunnel.csv**) and **large.csv/small.csv** for Srlg-DRCR problem.**The tunnel_name you select should in accordance with the type_name**
