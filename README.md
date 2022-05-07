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
