#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include "algorithm.h"
#include <unordered_set>
#include <map>

class EffSol : public Algorithm
{
 public:
    EffSol() {};
    ~EffSol() {};
    void SetupTopology(Graph *graph) override {
        graph_ = graph;
    }
    Path FindPath(const Flow &flow) override;
 private:
    std::unordered_set<NodeId> N1;
    std::unordered_set<NodeId> Nf;
    std::unordered_set<NodeId> N2;
    Graph *G1;
    Graph *G2;
    //record the shortest path tree;
    std::vector<Link*> parent;
    std::vector<double> C;
    std::vector<double> R;
    std::vector<double> C_;
    std::vector<double> R_;
    std::vector<std::unordered_map<int,std::vector<Link*>>> eff_part_path;
    //P[i] is the set of nodes k sunch that the node i belongs to the path from s to k.
    std::vector<std::unordered_set<int>> P;
    std::vector<std::unordered_set<int>> P_;
    //Initial all  variable.
    void InitialEff();
    // Nf will be taken as an s-t disconnecting set of vertices.
    void SplitGraph(const Flow &flow);
    //initalize the shortest path tree.
    void InitialTree(const Flow &flow);
    void UpdatePartPath(int &k);
    //generation of the efficient part solutions
    void CalPartPath(const Flow &flow);
    //get path vector from j to t;
    std::vector<std::vector<Link*>> GetRestPath(int u, const Flow &flow);
};

#endif