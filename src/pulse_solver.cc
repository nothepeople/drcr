#include <list>

#include "ap_path.h"
#include "bp_path.h"
#include "pulse_solver.h"
#include "shortest_path.h"

namespace {

void PrintSrlgs(const std::vector<int>& srlgs) {
    for (int srlg : srlgs) {
        std::cout << srlg << " ";
    }
    std::cout << "\n";
}

void PrintSrlgs(const std::unordered_set<int>& srlgs) {
    for (int srlg : srlgs) {
        std::cout << srlg << " ";
    }
    std::cout << "\n";
}

}  // namespace

Path Pulse::FindPath(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    GenericAp ap;
    ap.Init(graph_, nullptr, a_star_cost.GetCostVector(),
            a_star_delay.GetCostVector());
    double min_cost = ap.FindOptPath(flow, kMaxValue, ap_info_, bp_info_);
    Path result;
    if (min_cost < kMaxValue) {
        result.path_link = ap.GetApPath();
        result.CompletePath();
    }
    end_time = clock();
    ap_info_.total_time += end_time - start_time;
    std::cout << "Pulse takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    return result;
}

PathPair Pulse::FindPathPair(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    // std::cout << "---Stage 1 search---\n";
    GenericAp ap;
    FastDisjointBp fast_bp(graph_);
    ap.Init(graph_, &fast_bp, a_star_cost.GetCostVector(),
            a_star_delay.GetCostVector());
    double min_cost_stage1 = ap.FindCloseToOptPath(flow, kMaxValue, ap_info_, bp_info_);
    // std::cout << "cost in stage 1: " << min_cost_stage1 << "\n";
    end_time = clock();
    // std::cout << "Pulse stage-1 takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    // std::cout << "---Stage 2 search---\n";
    DisjointBp bp(graph_);
    ap.Init(graph_, &bp, a_star_cost.GetCostVector(),
            a_star_delay.GetCostVector());
    double min_cost_stage2 = ap.FindOptPath(flow, min_cost_stage1, ap_info_, bp_info_);
    PathPair result;
    if (min_cost_stage2 < kMaxValue) {
        result.ap_path.path_link = ap.GetApPath();
        result.bp_path.path_link = ap.GetBpPath();
        result.ap_path.CompletePath();
        result.bp_path.CompletePath();
    }
    end_time = clock();
    std::cout << "In total, Pulse takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    return result;
}

Path BidirectionalPulse::FindPath(const Flow &flow) {
    // clock_t start_time;
    // clock_t end_time;
    // start_time = clock();
    // AStar a_star_delay(graph_, LinkDelay);
    // a_star_delay.InitWithDst(flow.to);
    BiDirectionAp ap;
    ap.Init(graph_, nullptr, nullptr, nullptr);
    double min_cost = ap.FindOptPath(flow, kMaxValue, ap_info_, bp_info_);
    Path result;
    if (min_cost < kMaxValue) {
        result.path_link = ap.GetApPath();
        result.CompletePath();
    }
    // end_time = clock();
    // std::cout << "BidirectionalPulse takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}

PathPair SrlgDisjointPulse::FindPathPair(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    std::cout << "---Stage 1 search---\n";
    SrlgDisjointBp conflict_set_bp(graph_, a_star_delay.GetCostVector());
    SrlgAp srlg_ap(&conflict_set_bp);
    DisjointBp bp(graph_);
    srlg_ap.Init(graph_, &bp, a_star_cost.GetCostVector(),
                 a_star_delay.GetCostVector());
    double min_cost_stage1 = srlg_ap.FindCloseToOptPath(flow, kMaxValue, ap_info_, bp_info_);
    // std::cout << "cost in stage 1: " << min_cost_stage1 << "\n";
    end_time = clock();
    std::cout << "SrlgDisjointPulse stage-1 takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    std::cout << "---Stage 2 search---\n";
    double min_cost_stage2 = srlg_ap.FindOptPath(flow, min_cost_stage1, ap_info_, bp_info_);
    PathPair result;
    if (min_cost_stage2 < kMaxValue) {
        result.ap_path.path_link = srlg_ap.GetApPath();
        result.bp_path.path_link = srlg_ap.GetBpPath();
        result.ap_path.CompletePath();
        result.bp_path.CompletePath();
    }
    end_time = clock();
    std::cout << "In total, SrlgDisjointPulse takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    return result;
}

PathPair CosePulse::FindPathPair(const Flow &flow) {
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    std::list<Instance> all_instances;
    all_instances.emplace_back(std::unordered_set<int>(), std::vector<int>());
    double best_cost_so_far = kMaxValue;
    PathPair result;
    std::vector<ConflictSet> conflict_sets;
    conflict_sets.reserve(20);
    int num_iterations = 0;
    SrlgIncludeExcludeAp srlg_inc_exc_ap(graph_);
    // NewSrlgIncludeExcludeAp srlg_inc_exc_ap(graph_, flow);
    DisjointBp bp(graph_);
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    SrlgDisjointBp srlg_bp(graph_, a_star_delay.GetCostVector());
    while (!all_instances.empty()) {
        // clock_t start_time_in;
        // clock_t end_time_in;
        // start_time_in = clock();
        ++num_iterations;
        std::cout << "---------round: " << num_iterations << "-------\n";
        ++ap_info_.iteration_num;
        Instance instance = all_instances.front();
        all_instances.pop_front();
        if (instance.lb >= best_cost_so_far) {
            continue;
        }
        if (!srlg_inc_exc_ap.Init(instance.included_srlgs,
                                  instance.excluded_srlgs,
                                  conflict_sets)) {
            continue;
        }
        // std::cout << "included srlgs: ";
        // PrintSrlgs(instance.included_srlgs);
        // std::cout << "excluded srlgs: ";
        // PrintSrlgs(instance.excluded_srlgs);
        double ap_cost = srlg_inc_exc_ap.FindOptPath(flow, best_cost_so_far, ap_info_, bp_info_);
        // std::cout << "Ap cost: " << ap_cost << "\n";
        // end_time_in = clock();
        // std::cout << "Iteration " << num_iterations << " ap path takes: "
        //           << double(end_time_in - start_time_in) / CLOCKS_PER_SEC * 1000
        //           << "(ms).\n";
        if (ap_cost >= best_cost_so_far) {
            continue;
        }
        std::vector<Link*> ap_path = srlg_inc_exc_ap.GetApPath();
        // Path p;
        // p.path_link = ap_path;
        // p.CompletePath();
        // p.Print();
        double ap_delay = ComputeDelay(ap_path);
        Flow bp_flow = flow;
        bp_flow.delay_lb =
            ap_delay - flow.diff > flow.delay_lb ?
            ap_delay - flow.diff : flow.delay_lb;
        bp_flow.delay_ub =
            ap_delay + flow.diff < flow.delay_ub ?
            ap_delay + flow.diff : flow.delay_ub;
        clock_t start_bp_time = clock();
        double bp_delay = bp.FindBpPath(ap_path, bp_flow, bp_info_.iteration_num);
        // std::cout << "bp delay: " << bp_delay << "\n";
        if (bp_delay < kMaxValue) {
            // std::cout << "bp path found: " << bp_delay << "\n";
            best_cost_so_far = ap_cost;
            result.ap_path.path_link = ap_path;
            result.bp_path.path_link = bp.GetBpPath();
        } else {
            // std::cout << "bp path not found!!!!";
            if (srlg_bp.FindBpPath(ap_path, flow, bp_info_.iteration_num) == kMaxValue) {
                conflict_sets.push_back(srlg_bp.GetConflictSet());
                ConflictSet& conflict_set = conflict_sets.back();
                srlg_bp.GetConflictSet().Print();
                std::vector<int> conflict_srlgs =
                    conflict_set.GetConflictSrlgs();
                instance.lb = ap_cost;
                for (int srlg : conflict_srlgs) {
                    if (instance.included_srlgs.find(srlg) !=
                        instance.included_srlgs.end()) {
                        continue;
                    }
                    instance.excluded_srlgs.push_back(srlg);
                    all_instances.push_back(instance);
                    instance.included_srlgs.insert(srlg);
                    instance.excluded_srlgs.pop_back();
                }
            } else {
                for (Link* link : ap_path) {
                    // 每条link的最后一个srlg为其专属srlg
                    int srlg = link->srlgs.back();
                    if (instance.included_srlgs.find(srlg) !=
                        instance.included_srlgs.end()) {
                        continue;
                    }
                    instance.excluded_srlgs.push_back(srlg);
                    all_instances.push_back(instance);
                    instance.included_srlgs.insert(srlg);
                    instance.excluded_srlgs.pop_back();
                }
            }
        }
        clock_t end_bp_time = clock();
        bp_info_.total_time += end_bp_time - start_bp_time;
        // end_time_in = clock();
        // std::cout << "Iteration " << num_iterations << " takes: "
        //           << double(end_time_in - start_time_in) / CLOCKS_PER_SEC * 1000
        //           << "(ms).\n";
    }
    if (best_cost_so_far < kMaxValue) {
        result.ap_path.CompletePath();
        result.bp_path.CompletePath();
    }
    end_time = clock();
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    std::cout << "In total, CosePulse takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    return result;
}
