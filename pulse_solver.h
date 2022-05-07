#ifndef PULSE_SOLVER_H_
#define PULSE_SOLVER_H_

#include "algorithm.h"
#include "graph.h"

#include <unordered_set>
#include <vector>

class Pulse : public Algorithm {
 public:
    Pulse() {}
    ~Pulse() {}
    void SetupTopology(Graph *graph) override {
        graph_ = graph;
    }
    PathPair FindPathPair(const Flow &flow) override;
    Path FindPath(const Flow &flow) override;
};

class BidirectionalPulse : public Pulse {
 public:
    Path FindPath(const Flow &flow) override;
};

class SrlgDisjointPulse : public Pulse {
 public:
    PathPair FindPathPair(const Flow &flow) override;
};

class CosePulse : public Pulse {
 public:
    PathPair FindPathPair(const Flow &flow) override;

 private:
    struct Instance {
        explicit Instance(
            const std::unordered_set<int>& include,
            const std::vector<int>& exclude)
            : included_srlgs(include), excluded_srlgs(exclude), lb(0) {}
        std::unordered_set<int> included_srlgs;
        std::vector<int> excluded_srlgs;
        double lb;
    };
};

#endif  // PULSE_SOLVER_H_
