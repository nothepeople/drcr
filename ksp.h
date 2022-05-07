#ifndef KSP_H_
#define KSP_H_

#include <functional>
#include <queue>
#include <utility>
#include <vector>
#include <ctime>

#include "algorithm.h"
#include "graph.h"

class Ksp : public Algorithm
{
public:
    Ksp() {}
    ~Ksp();
    void SetupTopology(Graph *graph) override;
    // PathPair FindPathPair(const Flow &flow) override;
    Path FindPath(const Flow &flow) override;
    void print();
    double get_optimality();

private:
    // template <class T>
    // struct Node {
    //     Node(T id_in, double distance_in)
    //         : id(id_in), distance(distance_in) {}

    //     friend bool operator<(const Node<T> &a, const Node<T> &b) {
    //         return a.distance > b.distance;
    //     }
    //     T id;
    //     double distance;
    // };
    struct Pathnode
    {
        NodeId pre;
        NodeId cur;
        Link *prev_link;
        double dis;
        // heuristic algorithm's value
        double value;
        Pathnode(int s, int t, double d, Link *l, double v) : pre(s), cur(t), dis(d), prev_link(l), value(v) {}
        friend bool operator<(const Pathnode &a, const Pathnode &b)
        {
            // return a.dis > b.dis;
            return a.value > b.value;
        }
    };

    struct PulseInfo
    {
        PulseInfo() : src_delay_to_cost_map(kMaxDelay, kMaxValue) {}
        void Clear()
        {
            for (int i = 0; i < kMaxDelay; ++i)
            {
                src_delay_to_cost_map[i] = kMaxValue;
            }
        }
        std::vector<double> src_delay_to_cost_map;
    };
    // update node path when doing dijkstra
    void update_node_path(NodeId cur_node, NodeId, Link *link);
    // dijkstra
    // void HeuristicDijkstra(NodeId source_id, NodeId to_id);
    
    void DstDijkstra(NodeId dst_node, int to_id, double *node_to_dst_dis);
    // update partial path's node path
    void update_partial_node_path(NodeId pre_node, NodeId cur_node, Link *link);
    // do dijkstra in patial path
    void PartialDijkstra(NodeId source_id, NodeId to_id, double *node_to_dst_dis);
    // do ksp algorithm
    void ksp(const Flow &flow);

    Path find_kth_path(const Path &path);

    void remove_used_paths(NodeId id, const Path &cur_path);

    void retrieve_used_paths(NodeId id, const Path &cur_path);
    // update cost and delay of ap/bp path
    void get_cost_and_delay(Path &path);

    Path revert_path(Path &path);
    // Variables for initialization
    int num_nodes_;

    // Variables for pulse search
    clock_t start_time_;
    clock_t end_time_;

    bool *dij_visited_;

    Flow flow_;

    double total_cost_;

    double optimality_;
    // use vector to store path of each nodes
    std::vector<Path> node_to_dis_;
    std::vector<Path> partial_dis_;
    std::vector<std::pair<double, double>> time_ms_cost_pairs_;
    std::vector<Path> ap_;
    std::vector<Path> bp_;
    Path results_;
    double *dst_delay_map_;
    double *partial_dst_delay_map_;
    // variables for srlg

    bool *bp_visited_;
};

#endif // KSP_H_