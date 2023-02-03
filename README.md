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

There are two ways to run our code.
```c++
make test
./test
```

```c++
make main
./main ./"TopoFilePath" ./"TunnelFilePath" "method_id"
```

- In main.cc, you can easily change the method you are going to apply to solve the test instance by change method_id.

- As for DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 7 for BidirectionalPulse, 8 for a heuristic algorithm developed in 1985.

- As for Srlg-Disjoint DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 6 for CosePulse+.
