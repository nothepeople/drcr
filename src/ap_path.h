#ifndef AP_PATH_H_
#define AP_PATH_H_
#include <assert.h>
#include <functional>
#include <unordered_set>
#include <vector>
#include "bp_path.h"
#include "graph.h"
#include "shortest_path.h"

class ApPath
{
public:
    virtual ~ApPath() {}
    // If bp_path_solver == nullptr, then ApPath only finds one path.
    virtual void Init(Graph *graph, BpPath *bp_path_solver,
                      const double *min_cost_to_dst,
                      const double *min_delay_to_dst);
    // Find an optimal path for @flow with cost smaller than @cost_ub.
    virtual double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) = 0;
    // Find a close-to-optimal path for @flow with cost smaller
    // than @cost_ub.
    virtual double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) = 0;
    virtual std::vector<Link *> GetApPath()
    {
        return ap_path_;
    }
    virtual std::vector<Link *> GetBpPath()
    {
        assert(bp_path_solver_ != nullptr);
        return bp_path_;
    }

protected:
    struct Snapshot
    {
        Link *link;
        int stage;
    };
    Graph *graph_;
    BpPath *bp_path_solver_;
    const double *min_cost_to_dst_;
    const double *min_delay_to_dst_;
    std::vector<Link *> ap_path_;
    std::vector<Link *> bp_path_;
};

// GenericAp guarantees optimality, but may take longer time.
class GenericAp : public ApPath
{
public:
    GenericAp() {}
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;
    double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;

private:
    // The following variable is only used in FindCloseToOptPath.
    std::vector<std::vector<double>> visited_best_cost_;
};

// BiDirectionAp
class BiDirectionAp : public ApPath
{
public:
    BiDirectionAp() {}
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;
    double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override
    {
        std::cout << "FindCloseToOptPath is not implemented!\n";
        assert(false);
    }
};

// SrlgAp can be also used for finding link disjoint path pairs.
// However, the efficiency is much lower.
class SrlgAp : public ApPath
{
public:
    explicit SrlgAp(BpPath *conflict_set_solver)
        : conflict_set_solver_(conflict_set_solver)
    {
        conflict_sets_.reserve(kMaxConflictSet);
        conflict_status_.reserve(kMaxConflictSet);
    }
    double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;
    double FindCloseToOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;

private:
    static const int kMaxConflictSet = 6;
    BpPath *conflict_set_solver_;
    std::vector<ConflictSet> conflict_sets_;
    std::vector<ConflictStatus> conflict_status_;
    // The following variable is only used in FindCloseToOptPath.
    std::vector<std::vector<double>> visited_best_cost_;
};

class SrlgIncludeExcludeAp
{
public:
    explicit SrlgIncludeExcludeAp(Graph *graph) : graph_(graph) {}
    virtual bool Init(const std::unordered_set<int> &included_srlgs,
                      const std::vector<int> &excluded_srlgs,
                      const std::vector<ConflictSet> &conflict_sets);
    virtual double FindOptPath(
        const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info);
    virtual std::vector<Link *> GetApPath()
    {
        return ap_path_;
    }

protected:
    struct Snapshot
    {
        Link *link;
        int stage;
    };
    // Verify if a path contains all the srlgs in @included_srlgs_.
    bool VerifyPath(const std::vector<Link *> &path);
    // Compute the number of included srlgs in @link.
    int NumIncludedSrlgsInLink(const Link &link);
    Graph *graph_;
    std::unordered_set<int> included_srlgs_;
    std::vector<int> excluded_srlgs_;
    std::vector<Link *> ap_path_;
    std::vector<ConflictSet> conflict_sets_;
    std::vector<ConflictStatus> conflict_status_;
};

class NewSrlgIncludeExcludeAp : public SrlgIncludeExcludeAp
{
public:
    NewSrlgIncludeExcludeAp(Graph *graph, const Flow& flow)
        : SrlgIncludeExcludeAp(graph), reachability_info_(graph, flow) {
        visit_info_ = reachability_info_.GetReachabilityInfo();
    }
    double FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info) override;

private:
    ReachabilityInfo reachability_info_;
    const std::vector<QuickVisitInfo>* visit_info_;
};

#endif // AP_PATH_H_
