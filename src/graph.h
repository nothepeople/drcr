#ifndef GRAPH_H_
#define GRAPH_H_

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using LinkId = int;
using NodeId = int;
using FlowId = int;

const double kMaxValue = 10000000.0;

enum LinkStatus
{
    Available = 0,
    Used = 1,
    Affected = 2,
    Conflict = 3
};

enum DisjointType
{
    LinkDisjoint = 0,
    NodeDisjoint = 1,
    SrlgDisjoint = 2
};

struct Link
{
    Link(LinkId link_id_in, NodeId source_id_in, NodeId ter_id_in,
         double cost_in, int bandwidth_in, double delay_in, int srlg_num_in)
        : link_id(link_id_in), source_id(source_id_in),
          ter_id(ter_id_in), cost(cost_in), bandwidth(bandwidth_in),
          delay(delay_in), srlg_num(srlg_num_in)
    {
        weight = 0;
        status = Available;
    }
    ~Link() {}
    LinkId link_id;
    NodeId source_id;
    NodeId ter_id;
    double cost;
    int bandwidth;
    double delay;
    int srlg_num;
    std::vector<int> srlgs;
    std::vector<Link *> cousins; // 记录所有与该link信息相同的link（包含本身）。
    double weight;
    LinkStatus status;
};

struct LogInfo
{
    LogInfo()
    {
        total_time = 0;
        // average_time = 0;
        iteration_num = 0;
        iteration_average_time = 0;
    }
    clock_t total_time;
    clock_t average_time;
    int iteration_num;
    double iteration_average_time;
};

struct Path
{
    Path()
    {
        cost = kMaxValue;
        delay = kMaxValue;
        path_info.reserve(50);
    }

    void clear()
    {
        path_info.clear();
        cost = kMaxValue;
        delay = kMaxValue;
    }

    friend bool operator==(const Path &a, const Path &b)
    {
        bool flag;
        if (a.path_link.size() != b.path_link.size())
            return false;
        for (int i = 0; i < a.path_link.size(); ++i)
        {
            if (a.path_link[i]->link_id != b.path_link[i]->link_id)
                return false;
        }
        return true;
    }

    void Print() const;
    bool Verify() const;
    void CompletePath();
    bool Empty() const
    {
        return path_info.empty() && path_link.empty();
    }

    std::vector<NodeId> path_info;
    std::vector<Link *> path_link;
    double cost;
    double delay;
};

struct PathPair
{
    void clear()
    {
        ap_path.clear();
        bp_path.clear();
    }

    void Print() const
    {
        ap_path.Print();
        bp_path.Print();
    }

    bool VerfyLinkDisjoint() const;

    Path ap_path;
    Path bp_path;
};

class Graph
{
public:
    explicit Graph(const std::string &file_path);

    int NodeSize() const
    {
        return size_;
    }

    int LinkSize() const
    {
        return links_.size();
    }

    void SortLinks();

    const std::vector<Link *> &GetEgressLinks(
        NodeId node) const
    {
        return node_to_egress_links_.at(node);
    }
    const std::vector<Link *> &GetBpEgressLinks(
        NodeId node) const
    {
        return bp_node_to_egress_links_.at(node);
    }
    const std::vector<Link *> &GetIngressLinks(
        NodeId node) const
    {
        return node_to_ingress_links_.at(node);
    }
    // 根据Egress Links的状态更新BP搜索中边的遍历顺序
    void UpdateBpEgressLinks();

    void ChangeLinkStatus(int srlg_id, LinkStatus status)
    {
        for (int i = 0; i < srlg_group_[srlg_id].size(); ++i)
        {
            srlg_group_[srlg_id][i]->status = status;
        }
    }

    std::vector<Link *> &GetSrlgGroup(int srlg_id)
    {
        return srlg_group_[srlg_id];
    }

    int GetSrlgGroupSize(int srlg_id) const
    {
        return srlg_group_[srlg_id].size();
    }

    std::vector<Link> &GetMutableLinks()
    {
        return links_;
    }

    int GetMaxNodeId()
    {
        return max_node_id_;
    }

    int GetMaxLinkId()
    {
        return max_link_id_;
    }

    int GetMaxSrlgId()
    {
        return max_srlg_id_;
    }

private:
    void FindCousinLinks();
    bool IsCousinLinkPair(const Link *a, const Link *b);

    std::vector<Link> links_;
    std::unordered_set<NodeId> nodes_;
    std::vector<std::vector<Link *>> node_to_egress_links_;
    std::vector<std::vector<Link *>> node_to_ingress_links_;
    std::vector<std::vector<Link *>> bp_node_to_egress_links_;
    std::vector<std::vector<Link *>> srlg_group_;
    int size_;
    int max_node_id_;
    int max_link_id_;
    int max_srlg_id_;
    Graph(const Graph &) = delete;
    Graph &operator=(const Graph &) = delete;
};

struct Flow
{
    Flow() {}
    Flow(FlowId id_in, NodeId from_in, NodeId to_in,
         double lb, double ub, int bandwidth_in, bool is_diff_in,
         double diff_in, DisjointType type_in, double opt_cost_in,
         int type_range = -1, double min_cost = -1, double min_delay = -1)
        : id(id_in), from(from_in), to(to_in), delay_lb(lb),
          delay_ub(ub), bandwidth(bandwidth_in), is_diff(is_diff_in),
          diff(diff_in), type(type_in), opt_cost(opt_cost_in),
          range_type(type_range), min_cost_delay(min_cost),
          min_delay_delay(min_delay) {}
    
    void Print() const;
    void PrintToCsv() const;
    inline bool CheckDelayUb(double delay) const
    {
        return (delay <= delay_ub + 1e-5);
    }
    inline bool CheckDelayLb(double delay) const
    {
        return (delay >= delay_lb - 1e-5);
    }
    FlowId id;
    NodeId from;
    NodeId to;
    double delay_lb;
    double delay_ub;
    int bandwidth;
    bool is_diff;
    double diff;
    DisjointType type;
    double opt_cost;
    double min_cost_delay;
    double min_delay_delay;
    // type = 0: lb < ub < min_delay < min_cost
    // type = 1: lb < min_delay < ub < min_cost
    // type = 2: lb < min_delay < min_cost < ub
    // type = 3: min_delay < lb < ub < min_cost
    // type = 4: min_delay < lb < min_cost < ub
    // type = 5: min_delay < min_cost < lb < ub
    int range_type;
};

class Demand
{
public:
    // Read network flows from a file.
    explicit Demand(const std::string &file_path);

    int NumFlows()
    {
        return flows_.size();
    }
    const Flow &GetFlow(FlowId id)
    {
        return flows_[id];
    }

private:
    std::vector<Flow> flows_;
};

class ConflictSet
{
public:
    explicit ConflictSet(int size) : conflict(size, false), num_conflicts(0) {}
    void MarkConflict(int id)
    {
        if (conflict[id] == false)
        {
            conflict[id] = true;
            ++num_conflicts;
        }
    }
    void Reset()
    {
        num_conflicts = 0;
        for (int i = 0; i < conflict.size(); ++i)
        {
            conflict[i] = false;
        }
    }
    bool CheckConflict(int id) const
    {
        return conflict[id];
    }
    int NumConflicts() const
    {
        return num_conflicts;
    }
    void Print() const
    {
        std::cout << "Conflict set: ";
        for (int i = 0; i < conflict.size(); ++i)
        {
            if (conflict[i])
            {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
    }
    std::vector<int> GetConflictSrlgs() const
    {
        std::vector<int> result;
        result.reserve(num_conflicts);
        for (int i = 0; i < conflict.size(); ++i)
        {
            if (conflict[i])
            {
                result.push_back(i);
            }
        }
        return result;
    }

private:
    std::vector<bool> conflict;
    int num_conflicts;
};

class ConflictStatus
{
public:
    explicit ConflictStatus(ConflictSet *conflict_set)
        : conflict_set_(conflict_set), num_nozero_ids_(0) {}
    void Init(const Path &path);
    void Init(const std::vector<Link *> &path);
    void Add(const Link *link);
    void Remove(const Link *link);
    void Reset()
    {
        id_to_cnt_.clear();
        num_nozero_ids_ = 0;
    }
    bool CoverConflictSet();

private:
    ConflictSet *conflict_set_;
    std::unordered_map<int, int> id_to_cnt_;
    int num_nozero_ids_;
};

class VisitInfo
{
public:
    explicit VisitInfo(double range) : range_(range)
    {
        delta_ = range / kNumSlicesInRange;
    }
    // Fast assumes that @cost is no smaller than the existing ones.
    bool FastCheckDominanceAndUpdate(double delay, double cost);
    bool CheckDominanceAndUpdate(double delay, double cost);
    void Process();
    double GetMinCost(double delay_lb) const;
    bool CheckMinCost(double delay_lb, double cost_ub) const;
    void Print() const;

    void SortEgressLinksBasedOnDelayBudget(
        const std::vector<Link*> all_egress_links,
        const std::vector<VisitInfo>& rechability_info);
    const std::vector<Link *> &GetEgressLinksBasedOnDelayLb(
        double delay_lb) const;

private:
    inline int DelayToIndex(double delay) const
    {
        return std::floor((delay - delay_min_) / delta_);
    }
    double range_;
    std::map<double, double> delay_to_cost_;
    std::vector<double> min_cost_vector_;
    double delay_min_;
    double delta_;
    static const int kNumSlicesInRange = 10;

    // 下面变量用于pulse搜索
    std::vector<std::vector<Link *>> delay_to_egress_links_;
    std::vector<Link *> empty_link_set_;
};

#endif // GRAPH_H_
