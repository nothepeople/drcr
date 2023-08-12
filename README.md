# INTRODUCTION

This is the code for our paper titled "Efficient Routing Algorithm Design for Large DetNet". Our code is highly optimized, and has been adopted by our sponsor company.

# DRCR

- We implement Cost-KSP, Lagrangian-KSP, Delay-KSP, Pulse+, etc., to solve the delay-range constrained routing (DRCR) problem.

- All the test cases are in the "/data/DelayRange" directory.

- Result: Pulse > LagrangianKsp > CostKsp/DelayKSP

- Note: The following paper proposed a heuristic algorithm to solve the DRCR problem, but cannot offer optimality guarantee.

Celso C. Ribeiro and Michel Minoux. "A Heuristic Approach to Hard Constrained Shortest Path Problems." Discrete Applied Mathematics. 1985.

# SrlgDisjoint DRCR

- We implement Cost-KSP, Lagrangian-KSP, Delay-KSP, CoSE-Pulse+, etc., to solve the Srlg-Disjoint DRCR problem.

- All the test cases are in the "/data/SrlgDisjoint" directory.

- We use two models to general Srlgs. In SrlgDisjoint_star, we randomly select the egress links of a node to form an Srlg; in SrlgDisjoint_random, we randomly select links from all the links to form an Srlg.

- Result: CoSE-Pulse+ > LagrangianKsp > CostKSP/DelayKSP

# Try Our Code

```c++
make main
./main ./"TopoFilePath" ./"TunnelFilePath" "method_id" "flow_id"
```

For example, you could run 
```c++
main data/DelayRange/node1000/k1/Case0/topo.csv data/DelayRange/node1000/k1/Case0/tunnel.csv 1 1
```
to test Pulse+ for the first DRCR cases in the directory data/DelayRange/node1000/k1/Case0/. You could run
```c++
main data/SrlgDisjoint/SrlgDisjoint_star/node1000/k1/Case0/topo.csv data/SrlgDisjoint/SrlgDisjoint_star/node1000/k1/Case0/tunnel_trap.csv 6 0
```
to test Cose-Pulse+ for all the Srlg-disjoint DRCR cases in the directory data/SrlgDisjoint/SrlgDisjoint_star/node1000/k1/Case0/. (0 represents all the test cases.)


In main.cc, you can try different methods using different method_ids.

- As for DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 6 for BidirectionalPulse, 7 for a heuristic algorithm developed in 1985.

- As for Srlg-Disjoint DRCR problem, you can choose 1 for Pulse+, 2 for DelayKsp, 3 for CostKsp, 4 for CostKspPulse, 5 for LagtangianKsp, 6 for CosePulse+.
