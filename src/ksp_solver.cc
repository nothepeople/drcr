#include "bp_path.h"
#include "ksp_solver.h"
#include "shortest_path.h"

#include <vector>

Path DelayKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkDelay);
    double delay = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    double min_cost = kMaxValue;
    Path result;
    while (flow.CheckDelayUb(delay))
    {
        if (flow.CheckDelayLb(delay))
        {
            const std::vector<Link *> &cur_path = ksp.GetPath();
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                result.path_link = cur_path;
                min_cost = cost;
            }
        }
        ++ap_info_.iteration_num;
        delay = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    // std::cout << "Delay KSP takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    if (min_cost < kMaxValue)
    {
        result.CompletePath();
    }
    return result;
}

PathPair DelayKsp::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkDelay);
    KspBp ksp_bp(graph_);
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double delay = ksp.Init(flow.from, flow.to);
    // int num_iterations = 0;
    double min_cost = kMaxValue;
    PathPair result;
    while (flow.CheckDelayUb(delay))
    {
        if (flow.CheckDelayLb(delay))
        {
            const std::vector<Link *> &cur_path = ksp.GetPath();
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                Flow bp_flow = flow;
                bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
                bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
                bp_start_time = clock();
                double bp_delay = ksp_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
                bp_end_time = clock();
                bp_info_.total_time += bp_end_time - bp_start_time;
                bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                              CLOCKS_PER_SEC * 1000;
                if (bp_delay < kMaxValue)
                {
                    result.ap_path.path_link = cur_path;
                    result.bp_path.path_link = ksp_bp.GetBpPath();
                    min_cost = cost;
                }
            }
        }
        ++ap_info_.iteration_num;
        delay = ksp.FindNextPath();
    }
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time += end_time - start_time - bp_info_.total_time;
    // std::cout << "Delay KSP takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    if (min_cost < kMaxValue)
    {
        result.ap_path.CompletePath();
        result.bp_path.CompletePath();
    }
    return result;
}

Path CostKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    double cost = ksp.Init(flow.from, flow.to);
    // int num_iterations = 0;
    Path result;
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            result.path_link = cur_path;
            result.CompletePath();
            break;
        }
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    // std::cout << "Cost KSP takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    ap_path_ = &result;
    return result;
}

PathPair CostKsp::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    KspBp ksp_bp(graph_);
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double cost = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    PathPair result;
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            Flow bp_flow = flow;
            bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
            bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
            bp_start_time = clock();
            double bp_delay = ksp_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
            bp_end_time = clock();
            bp_info_.total_time += bp_end_time - bp_start_time;
            bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                          CLOCKS_PER_SEC * 1000;
            if (bp_delay < kMaxValue)
            {
                result.ap_path.path_link = cur_path;
                result.ap_path.CompletePath();
                result.bp_path.path_link = ksp_bp.GetBpPath();
                result.bp_path.CompletePath();
                break;
            }
        }
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    ap_path_ = &result.ap_path;
    bp_path_ = &result.bp_path;
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    // std::cout << "KSP (including BP search) takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    return result;
}

PathPair CostKspPulse::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    KShortestPath ksp(graph_, LinkCost);
    DisjointBp pulse_bp(graph_);
    AStar a_star(graph_, LinkDelay);
    a_star.InitWithDst(flow.to);
    SrlgDisjointBp conflict_set_bp(graph_, a_star.GetCostVector());
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double cost = ksp.Init(flow.from, flow.to);
    // int num_iterations = 0;
    PathPair result;
    std::vector<ConflictSet> conflict_sets;
    conflict_sets.reserve(20);
    while (cost < kMaxValue)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            bool skip = false;
            for (ConflictSet &cs : conflict_sets)
            {
                ConflictStatus status(&cs);
                status.Init(cur_path);
                if (status.CoverConflictSet())
                {
                    skip = true;
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            Flow bp_flow = flow;
            bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
            bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
            bp_start_time = clock();
            double bp_delay = pulse_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
            bp_end_time = clock();
            bp_info_.total_time += bp_end_time - bp_start_time;
            bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                          CLOCKS_PER_SEC * 1000;
            if (bp_delay < kMaxValue)
            {
                result.ap_path.path_link = cur_path;
                result.ap_path.CompletePath();
                result.bp_path.path_link = pulse_bp.GetBpPath();
                result.bp_path.CompletePath();
                break;
            }
            else
            {
                double bp_delay_original_flow =
                    conflict_set_bp.FindBpPath(cur_path, flow, bp_info_.iteration_num);
                if (bp_delay_original_flow == kMaxValue)
                {
                    conflict_sets.push_back(
                        conflict_set_bp.GetConflictSet());
                    ksp.AddConflictSet(conflict_sets.back());
                }
            }
        }
        ++ap_info_.iteration_num;
        cost = ksp.FindNextPath();
    }
    // std::cout << "Total number of AP iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    // std::cout << "KSP (including BP search) takes: "
    //   << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //   << "(ms).\n";
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    return result;
}

Path LagrangianKsp::FindPath(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    double u;
    if (default_u_ != kMaxValue)
    {
        u = default_u_;
    }
    else
    {
        u = CalculateMultiplier(graph_, flow);
        if (u == kMaxValue)
        {
            return Path();
        }
    }
    auto weight_func = [u](Link *link)
    {
        return link->cost + u * link->delay;
    };
    KShortestPath ksp(graph_, weight_func);
    double weight = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    Path result;
    double min_cost = kMaxValue;
    double weight_ub = kMaxValue;
    while (weight < weight_ub)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            double cost = ComputeCost(cur_path);
            if (cost < min_cost)
            {
                result.path_link = cur_path;
                min_cost = cost;
                weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
            }
        }
        ++ap_info_.iteration_num;
        weight = ksp.FindNextPath();
    }
    result.CompletePath();
    ap_path_ = &result;
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time;
    // std::cout << "LagrangianKsp with u=" << u << " takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}

PathPair LagrangianKsp::FindPathPair(const Flow &flow)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    clock_t bp_start_time;
    clock_t bp_end_time;
    double bp_time_ms = 0;
    double u;
    if (default_u_ != kMaxValue)
    {
        u = default_u_;
    }
    else
    {
        u = CalculateMultiplier(graph_, flow);
        if (u == kMaxValue)
        {
            return PathPair();
        }
    }
    auto weight_func = [u](Link *link)
    {
        return link->cost + u * link->delay;
    };
    KShortestPath ksp(graph_, weight_func);
    KspBp ksp_bp(graph_);
    double weight = ksp.Init(flow.from, flow.to);
    int num_iterations = 0;
    PathPair result;
    double min_cost = kMaxValue;
    double weight_ub = kMaxValue;
    while (weight < weight_ub)
    {
        const std::vector<Link *> &cur_path = ksp.GetPath();
        double delay = ComputeDelay(cur_path);
        // if (num_iterations % 1000 == 1)
        //     std::cout << "iterations " << num_iterations << ", delay: "
        //               << delay << ", weight: " << weight << ", weight ub: "
        //               << weight_ub << "\n";
        if (flow.CheckDelayUb(delay) && flow.CheckDelayLb(delay))
        {
            Flow bp_flow = flow;
            bp_flow.delay_lb = delay - flow.diff > flow.delay_lb ? delay - flow.diff : flow.delay_lb;
            bp_flow.delay_ub = delay + flow.diff < flow.delay_ub ? delay + flow.diff : flow.delay_ub;
            bp_start_time = clock();
            double bp_delay = ksp_bp.FindBpPath(cur_path, bp_flow, bp_info_.iteration_num);
            // std::cout << "bp_iteration_num: " << bp_info_.iteration_num << std::endl;
            bp_end_time = clock();
            bp_info_.total_time += bp_end_time - bp_start_time;
            bp_time_ms += static_cast<double>(bp_end_time - bp_start_time) /
                          CLOCKS_PER_SEC * 1000;
            if (bp_delay < kMaxValue)
            {
                double cost = ComputeCost(cur_path);
                if (cost < min_cost)
                {
                    result.ap_path.path_link = cur_path;
                    result.bp_path.path_link = ksp_bp.GetBpPath();
                    min_cost = cost;
                    weight_ub = u < 0 ? cost + u * flow.delay_lb : cost + u * flow.delay_ub;
                }
            }
        }
        ++ap_info_.iteration_num;
        weight = ksp.FindNextPath();
    }
    result.ap_path.CompletePath();
    result.bp_path.CompletePath();
    ap_path_ = &result.ap_path;
    bp_path_ = &result.bp_path;
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    ap_info_.total_time = end_time - start_time - bp_info_.total_time;
    // std::cout << "BP search takes: " << bp_time_ms << "(ms).\n";
    // std::cout << "LagrangianKsp with u=" << u << " takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    return result;
}
