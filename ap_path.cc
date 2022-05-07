#include "ap_path.h"

namespace
{

    void SortLinks(const double *min_delay_to_dst, Graph *graph)
    {
        for (Link &link : graph->GetMutableLinks())
        {
            link.weight = link.delay + min_delay_to_dst[link.ter_id];
        }
        graph->SortLinks();
    }

    void Print(const std::vector<Link *> &links)
    {
        std::cout << "links: ";
        for (Link *link : links)
        {
            std::cout << link->link_id << " ";
        }
        std::cout << "\n";
    }

    int MaxConflictSetIdx(
        const std::vector<ConflictSet> &conflict_sets)
    {
        int max_size = -1;
        int idx = -1;
        for (int i = 0; i < conflict_sets.size(); ++i)
        {
            const ConflictSet &conflict_set = conflict_sets.at(i);
            if (conflict_set.NumConflicts() > max_size)
            {
                max_size = conflict_set.NumConflicts();
                idx = i;
            }
        }
        return idx;
    }

} // namespace

void ApPath::Init(Graph *graph, BpPath *bp_path_solver,
                  const double *min_cost_to_dst,
                  const double *min_delay_to_dst)
{
    graph_ = graph;
    bp_path_solver_ = bp_path_solver;
    min_cost_to_dst_ = min_cost_to_dst;
    min_delay_to_dst_ = min_delay_to_dst;
    if (min_delay_to_dst_)
    {
        SortLinks(min_delay_to_dst_, graph_);
    }
}

double GenericAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    // Start DFS search
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    // Start BP search
    int num_iterations = 0;
    int bp_time = 0;
    while (!ap_stack.empty())
    {
        // ++num_iterations;
        ++ap_info.iteration_num;
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "---search bp path---\n";
                        // Print(ap_path);
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            // std::cout << "find a bp path\n";
                            // Print(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far)
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    // std::cout << "Total number of iterations: " << num_iterations << "\n";
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

double GenericAp::FindCloseToOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Start DFS search
    clock_t start_time;
    clock_t end_time;
    int bp_time = 0;
    start_time = clock();
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    // Initialize visited_best_cost_
    visited_best_cost_.clear();
    visited_best_cost_.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i)
    {
        visited_best_cost_.emplace_back(flow.delay_ub + 1, kMaxValue);
    }
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    // Start BP search
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        ap_path.push_back(ap_link);
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

double BiDirectionAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    ReachabilityInfo reachability_info(graph_, flow);
    clock_t start_time;
    clock_t end_time;
    int bp_time = 0;
    start_time = clock();
    const std::vector<VisitInfo> *visit_info =
        reachability_info.GetReachabilityInfo();
    end_time = clock();
    std::cout << "Calculating reachability info takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    // Start DFS search
    start_time = clock();
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    // Start BP search
    int num_iterations = 0;
    while (!ap_stack.empty())
    {
        // ++num_iterations;
        ++ap_info.iteration_num;
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            ap_visited[u] = false;
        }
        else
        {
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    if (bp_path_solver_ == nullptr)
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size());
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                        ap_path_.push_back(ap_link);
                    }
                    else
                    {
                        Flow bp_flow = flow;
                        bp_flow.delay_lb =
                            new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                        bp_flow.delay_ub =
                            new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                        clock_t start_bp = clock();
                        double bp_delay =
                            bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "---search bp path---\n";
                        // Print(ap_path);
                        if (bp_delay < kMaxValue)
                        {
                            double bp_cost =
                                ComputeCost(bp_path_solver_->GetBpPath());
                            // std::cout << "find a bp path\n";
                            // Print(bp_path_solver_->GetBpPath());
                            if (bp_cost >= new_cost)
                            {
                                best_cost_so_far = new_cost;
                                ap_path_.clear();
                                ap_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    ap_path_.push_back(ap_path[i]);
                                }
                                bp_path_ = bp_path_solver_->GetBpPath();
                            }
                            else
                            {
                                best_cost_so_far = bp_cost;
                                ap_path_ = bp_path_solver_->GetBpPath();
                                bp_path_.clear();
                                bp_path_.reserve(ap_path.size() - 1);
                                for (int i = 1; i < ap_path.size(); ++i)
                                {
                                    bp_path_.push_back(ap_path[i]);
                                }
                            }
                        }
                        ap_path.pop_back();
                    }
                }
            }
            else
            {
                // prune
                // CheckMinCost隐含了feasibility + optimality剪枝
                if (visit_info->at(u).CheckMinCost(
                        flow.delay_lb - new_delay,
                        best_cost_so_far - new_cost))
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                        // 新的排序方法效果不明显
                        // visit_info->at(u).GetEgressLinksBasedOnDelayLb(
                        //     flow.delay_lb - new_delay);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

double SrlgAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Start DFS search
    clock_t start_time = clock();
    clock_t end_time;
    int bp_time = 0;
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    int new_conf_set_id = 0;
    // Start BP search
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            for (int i = 0; i < conflict_sets_.size(); ++i)
            {
                conflict_status_[i].Remove(ap_link);
            }
            ap_visited[u] = false;
        }
        else
        {
            bool skip = false;
            for (int i = 0; i < conflict_sets_.size(); ++i)
            {
                conflict_status_[i].Add(ap_link);
                if (conflict_status_[i].CoverConflictSet())
                {
                    skip = true;
                    for (int j = 0; j <= i; ++j)
                    {
                        conflict_status_[j].Remove(ap_link);
                    }
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                for (int i = 0; i < conflict_sets_.size(); ++i)
                {
                    conflict_status_[i].Remove(ap_link);
                }
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    Flow bp_flow = flow;
                    bp_flow.delay_lb =
                        new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                    bp_flow.delay_ub =
                        new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                    ap_path.push_back(ap_link);
                    clock_t start_bp = clock();
                    double bp_delay =
                        bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                    clock_t end_bp = clock();
                    bp_time += end_bp - start_bp;
                    // Print(ap_path);
                    if (bp_delay < kMaxValue)
                    {
                        double bp_cost =
                            ComputeCost(bp_path_solver_->GetBpPath());
                        // std::cout << "find a bp path\n";
                        // Print(bp_path_solver_->GetBpPath());
                        if (bp_cost >= new_cost)
                        {
                            best_cost_so_far = new_cost;
                            ap_path_.clear();
                            ap_path_.reserve(ap_path.size() - 1);
                            for (int i = 1; i < ap_path.size(); ++i)
                            {
                                ap_path_.push_back(ap_path[i]);
                            }
                            bp_path_ = bp_path_solver_->GetBpPath();
                        }
                        else
                        {
                            best_cost_so_far = bp_cost;
                            ap_path_ = bp_path_solver_->GetBpPath();
                            bp_path_.clear();
                            bp_path_.reserve(ap_path.size() - 1);
                            for (int i = 1; i < ap_path.size(); ++i)
                            {
                                bp_path_.push_back(ap_path[i]);
                            }
                        }
                        ap_path.pop_back();
                    }
                    else
                    {
                        clock_t start_bp = clock();
                        double bp_delay_original_flow =
                            conflict_set_solver_->FindBpPath(ap_path, flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        // std::cout << "cannot find a bp path\n";
                        // bp_path_solver_->GetConflictSet().Print();
                        ap_path.pop_back();
                        if (bp_delay_original_flow == kMaxValue)
                        {
                            // std::cout << "-----current conflict set-----\n";
                            // for (ConflictSet cs : conflict_sets_) {
                            //     cs.Print();
                            // }
                            if (conflict_sets_.size() < kMaxConflictSet)
                            {
                                new_conf_set_id = conflict_sets_.size();
                                conflict_sets_.push_back(
                                    conflict_set_solver_->GetConflictSet());
                                conflict_status_.emplace_back(
                                    &conflict_sets_[new_conf_set_id]);
                            }
                            else
                            {
                                new_conf_set_id =
                                    MaxConflictSetIdx(conflict_sets_);
                                conflict_sets_[new_conf_set_id] =
                                    conflict_set_solver_->GetConflictSet();
                                conflict_status_[new_conf_set_id] =
                                    ConflictStatus(
                                        &conflict_sets_[new_conf_set_id]);
                            }
                            conflict_status_[new_conf_set_id].Init(ap_path);
                        }
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far)
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < conflict_sets_.size(); ++i)
                        {
                            conflict_status_[i].Remove(ap_link);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < conflict_sets_.size(); ++i)
                    {
                        conflict_status_[i].Remove(ap_link);
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

double SrlgAp::FindCloseToOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Clear conflict sets
    clock_t start_time = clock();
    clock_t end_time;
    int bp_time = 0;
    conflict_sets_.clear();
    conflict_status_.clear();
    // Start DFS search
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    int new_conf_set_id = 0;
    // Initialize visited_best_cost_
    visited_best_cost_.clear();
    visited_best_cost_.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i)
    {
        visited_best_cost_.emplace_back(flow.delay_ub + 1, kMaxValue);
    }
    // Start BP search
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            for (int i = 0; i < conflict_sets_.size(); ++i)
            {
                conflict_status_[i].Remove(ap_link);
            }
            ap_visited[u] = false;
        }
        else
        {
            bool skip = false;
            for (int i = 0; i < conflict_sets_.size(); ++i)
            {
                conflict_status_[i].Add(ap_link);
                if (conflict_status_[i].CoverConflictSet())
                {
                    skip = true;
                    for (int j = 0; j <= i; ++j)
                    {
                        conflict_status_[j].Remove(ap_link);
                    }
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                for (int i = 0; i < conflict_sets_.size(); ++i)
                {
                    conflict_status_[i].Remove(ap_link);
                }
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    Flow bp_flow = flow;
                    bp_flow.delay_lb =
                        new_delay - flow.diff > flow.delay_lb ? new_delay - flow.diff : flow.delay_lb;
                    bp_flow.delay_ub =
                        new_delay + flow.diff < flow.delay_ub ? new_delay + flow.diff : flow.delay_ub;
                    ap_path.push_back(ap_link);
                    clock_t start_bp = clock();
                    double bp_delay =
                        bp_path_solver_->FindBpPath(ap_path, bp_flow, bp_info.iteration_num);
                    clock_t end_bp = clock(); // Print(ap_path);
                    bp_time += end_bp - start_bp;
                    if (bp_delay < kMaxValue)
                    {
                        double bp_cost =
                            ComputeCost(bp_path_solver_->GetBpPath());
                        // std::cout << "find a bp path\n";
                        // Print(bp_path_solver_->GetBpPath());
                        if (bp_cost >= new_cost)
                        {
                            best_cost_so_far = new_cost;
                            ap_path_.clear();
                            ap_path_.reserve(ap_path.size() - 1);
                            for (int i = 1; i < ap_path.size(); ++i)
                            {
                                ap_path_.push_back(ap_path[i]);
                            }
                            bp_path_ = bp_path_solver_->GetBpPath();
                        }
                        else
                        {
                            best_cost_so_far = bp_cost;
                            ap_path_ = bp_path_solver_->GetBpPath();
                            bp_path_.clear();
                            bp_path_.reserve(ap_path.size() - 1);
                            for (int i = 1; i < ap_path.size(); ++i)
                            {
                                bp_path_.push_back(ap_path[i]);
                            }
                        }
                        ap_path.pop_back();
                    }
                    else
                    {
                        // std::cout << "cannot find a bp path\n";
                        // bp_path_solver_->GetConflictSet().Print();
                        clock_t start_bp = clock();
                        double bp_delay_original_flow =
                            conflict_set_solver_->FindBpPath(ap_path, flow, bp_info.iteration_num);
                        clock_t end_bp = clock();
                        bp_time += end_bp - start_bp;
                        ap_path.pop_back();
                        if (bp_delay_original_flow == kMaxValue)
                        {
                            // conflict_set_solver_->GetConflictSet().Print();
                            if (conflict_sets_.size() < kMaxConflictSet)
                            {
                                new_conf_set_id = conflict_sets_.size();
                                conflict_sets_.push_back(
                                    conflict_set_solver_->GetConflictSet());
                                conflict_status_.emplace_back(
                                    &conflict_sets_[new_conf_set_id]);
                            }
                            else
                            {
                                new_conf_set_id =
                                    MaxConflictSetIdx(conflict_sets_);
                                conflict_sets_[new_conf_set_id] =
                                    conflict_set_solver_->GetConflictSet();
                                conflict_status_[new_conf_set_id] =
                                    ConflictStatus(
                                        &conflict_sets_[new_conf_set_id]);
                            }
                            conflict_status_[new_conf_set_id].Init(ap_path);
                        }
                    }
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst_[u] + new_delay) &&
                    min_cost_to_dst_[u] + new_cost < best_cost_so_far &&
                    visited_best_cost_[u][new_delay] > new_cost)
                {
                    visited_best_cost_[u][new_delay] = new_cost;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < conflict_sets_.size(); ++i)
                        {
                            conflict_status_[i].Remove(ap_link);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < conflict_sets_.size(); ++i)
                    {
                        conflict_status_[i].Remove(ap_link);
                    }
                }
            }
        }
    }
    end_time = clock();
    // std::cout << "GenericAp takes: "
    //           << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
    //           << "(ms).\n";
    bp_info.total_time += bp_time;
    ap_info.total_time += end_time - start_time - bp_time;
    return best_cost_so_far;
}

bool SrlgIncludeExcludeAp::Init(
    const std::unordered_set<int> &included_srlgs,
    const std::vector<int> &excluded_srlgs,
    const std::vector<ConflictSet> &conflict_sets)
{
    included_srlgs_ = included_srlgs;
    excluded_srlgs_ = excluded_srlgs;
    conflict_sets_.clear();
    conflict_sets_.reserve(conflict_sets.size());
    for (const ConflictSet &cs : conflict_sets)
    {
        std::vector<int> cs_srlgs = cs.GetConflictSrlgs();
        std::unordered_set<int> cs_srlgs_not_included;
        for (int srlg : cs_srlgs)
        {
            if (included_srlgs_.find(srlg) == included_srlgs_.end())
            {
                cs_srlgs_not_included.insert(srlg);
            }
        }
        if (cs_srlgs_not_included.empty())
        {
            return false;
        }
        // 检查cs_srlgs_not_included的大小，如果为1加入excluded_srlgs
        if (cs_srlgs_not_included.size() == 1)
        {
            auto it = cs_srlgs_not_included.begin();
            excluded_srlgs_.push_back(*it);
            continue;
        }
        // Construct a new conflict set.
        conflict_sets_.emplace_back(graph_->GetMaxSrlgId() + 1);
        for (int srlg : cs_srlgs_not_included)
        {
            conflict_sets_.back().MarkConflict(srlg);
        }
    }
    // If a conflict set is included in the excluded set, then
    // this problem instance does not have a solution.
    for (const ConflictSet &cs : conflict_sets)
    {
        if (cs.NumConflicts() > excluded_srlgs.size())
        {
            continue;
        }
        int num_srlg_included = 0;
        for (int srlg : excluded_srlgs)
        {
            if (cs.CheckConflict(srlg))
            {
                ++num_srlg_included;
            }
        }
        if (num_srlg_included == cs.NumConflicts())
        {
            return false;
        }
    }
    conflict_status_.clear();
    conflict_status_.reserve(conflict_sets_.size());
    for (ConflictSet &cs : conflict_sets_)
    {
        bool exclude = false;
        for (int srlg : excluded_srlgs)
        {
            if (cs.CheckConflict(srlg))
            {
                exclude = true;
                break;
            }
        }
        if (exclude)
        {
            continue;
        }
        conflict_status_.emplace_back(&cs);
    }
    return true;
}

double SrlgIncludeExcludeAp::FindOptPath(const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Disable links in excluded srlgs
    clock_t start_time = clock();
    for (int srlg : excluded_srlgs_)
    {
        graph_->ChangeLinkStatus(srlg, Conflict);
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        bool able_to_include = false;
        for (Link *link : links)
        {
            if (link->status == Available)
            {
                able_to_include = true;
            }
        }
        if (!able_to_include)
        {
            for (int srlg : excluded_srlgs_)
            {
                graph_->ChangeLinkStatus(srlg, Available);
            }
            return kMaxValue;
        }
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        if (links.size() == 1)
        {
            for (Link *link : graph_->GetEgressLinks(links[0]->source_id))
            {
                if (link != links[0])
                {
                    link->status = Conflict;
                }
            }
        }
    }
    // Prepare the Pulse search
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    const double *min_cost_to_dst = a_star_cost.GetCostVector();
    const double *min_delay_to_dst = a_star_delay.GetCostVector();
    for (Link &link : graph_->GetMutableLinks())
    {
        link.weight = NumIncludedSrlgsInLink(link) * kMaxValue +
                      link.delay + min_delay_to_dst[link.ter_id];
    }
    graph_->SortLinks();
    // Start DFS search
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    /*
    // Start AP search with dominance check.
    std::vector<std::vector<bool>> visit_cnt;
    visit_cnt.reserve(graph_->GetMaxNodeId() + 1);
    for (int i = 0; i <= graph_->GetMaxNodeId(); ++i) {
        visit_cnt.emplace_back(flow.delay_ub + 1, false);
    }
    while (!ap_stack.empty()) {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link* ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        if (ap_snap.stage == 1) {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            for (int i = 0; i < conflict_status_.size(); ++i) {
                conflict_status_[i].Remove(ap_link);
            }
            ap_visited[u] = false;
        } else {
            bool skip = false;
            for (int i = 0; i < conflict_status_.size(); ++i) {
                conflict_status_[i].Add(ap_link);
                if (conflict_status_[i].CoverConflictSet()) {
                    skip = true;
                    for (int j = 0; j <= i; ++j) {
                        conflict_status_[j].Remove(ap_link);
                    }
                    break;
                }
            }
            if (skip) {
                continue;
            }
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to) {
                for (int i = 0; i < conflict_status_.size(); ++i) {
                    conflict_status_[i].Remove(ap_link);
                }
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far) {
                    ap_path.push_back(ap_link);
                    if (VerifyPath(ap_path)) {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size() - 1);
                        for (int i = 1; i < ap_path.size(); ++i) {
                            ap_path_.push_back(ap_path[i]);
                        }
                    }
                    ap_path.pop_back();
                }
            } else {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay) &&
                    min_cost_to_dst[u] + new_cost < best_cost_so_far &&
                    !visit_cnt[u][new_delay]) {
                    visit_cnt[u][new_delay] = true;
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty()) {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links) {
                            if (link->status == Conflict) {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id]) {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    } else {
                        for (int i = 0; i < conflict_status_.size(); ++i) {
                            conflict_status_[i].Remove(ap_link);
                        }
                    }
                } else {
                    for (int i = 0; i < conflict_status_.size(); ++i) {
                        conflict_status_[i].Remove(ap_link);
                    }
                }
            }
        }
    }
    // Start AP search again without dominance check
    ap_stack.push_back(snapshot);*/
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            for (int i = 0; i < conflict_status_.size(); ++i)
            {
                conflict_status_[i].Remove(ap_link);
            }
            ap_visited[u] = false;
        }
        else
        {
            bool skip = false;
            for (int i = 0; i < conflict_status_.size(); ++i)
            {
                conflict_status_[i].Add(ap_link);
                if (conflict_status_[i].CoverConflictSet())
                {
                    skip = true;
                    for (int j = 0; j <= i; ++j)
                    {
                        conflict_status_[j].Remove(ap_link);
                    }
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                for (int i = 0; i < conflict_status_.size(); ++i)
                {
                    conflict_status_[i].Remove(ap_link);
                }
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    ap_path.push_back(ap_link);
                    if (VerifyPath(ap_path))
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size() - 1);
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                    }
                    ap_path.pop_back();
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay) &&
                    min_cost_to_dst[u] + new_cost < best_cost_so_far)
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < conflict_status_.size(); ++i)
                        {
                            conflict_status_[i].Remove(ap_link);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < conflict_status_.size(); ++i)
                    {
                        conflict_status_[i].Remove(ap_link);
                    }
                }
            }
        }
    }
    // Enable links in excluded srlgs
    for (int srlg : excluded_srlgs_)
    {
        graph_->ChangeLinkStatus(srlg, Available);
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        if (links.size() == 1)
        {
            for (Link *link : graph_->GetEgressLinks(links[0]->source_id))
            {
                link->status = Available;
            }
        }
    }
    ap_info.total_time += clock() - start_time;
    return best_cost_so_far;
}

bool SrlgIncludeExcludeAp::VerifyPath(const std::vector<Link *> &path)
{
    std::unordered_set<int> srlgs_in_path;
    for (Link *link : path)
    {
        for (int srlg : link->srlgs)
        {
            if (included_srlgs_.find(srlg) != included_srlgs_.end())
            {
                srlgs_in_path.insert(srlg);
            }
        }
    }
    return (srlgs_in_path.size() == included_srlgs_.size());
}

int SrlgIncludeExcludeAp::NumIncludedSrlgsInLink(const Link &link)
{
    int total = 0;
    for (int srlg : link.srlgs)
    {
        if (included_srlgs_.find(srlg) != included_srlgs_.end())
        {
            ++total;
        }
    }
    return total;
}

double NewSrlgIncludeExcludeAp::FindOptPath(
    const Flow &flow, double cost_ub, LogInfo &ap_info, LogInfo &bp_info)
{
    // Disable links in excluded srlgs
    clock_t start_time = clock();
    for (int srlg : excluded_srlgs_)
    {
        graph_->ChangeLinkStatus(srlg, Conflict);
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        bool able_to_include = false;
        for (Link *link : links)
        {
            if (link->status == Available)
            {
                able_to_include = true;
            }
        }
        if (!able_to_include)
        {
            for (int srlg : excluded_srlgs_)
            {
                graph_->ChangeLinkStatus(srlg, Available);
            }
            return kMaxValue;
        }
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        if (links.size() == 1)
        {
            for (Link *link : graph_->GetEgressLinks(links[0]->source_id))
            {
                if (link != links[0])
                {
                    link->status = Conflict;
                }
            }
        }
    }
    // Prepare the Pulse search
    AStar a_star_delay(graph_, LinkDelay);
    a_star_delay.InitWithDst(flow.to);
    AStar a_star_cost(graph_, LinkCost);
    a_star_cost.InitWithDst(flow.to);
    const double *min_cost_to_dst = a_star_cost.GetCostVector();
    const double *min_delay_to_dst = a_star_delay.GetCostVector();
    for (Link &link : graph_->GetMutableLinks())
    {
        link.weight = NumIncludedSrlgsInLink(link) * kMaxValue +
                      link.delay + min_delay_to_dst[link.ter_id];
    }
    graph_->SortLinks();
    // Start DFS search
    std::vector<Snapshot> ap_stack;
    ap_stack.reserve(1000000);
    // Create a fake link with delay = 0
    Link fake_link(-1, -1, flow.from, 0, 0, 0, 0);
    Snapshot snapshot;
    snapshot.link = &fake_link;
    snapshot.stage = 0;
    ap_stack.push_back(snapshot);
    // Track the searched path
    std::vector<Link *> ap_path;
    double ap_path_delay = 0;
    double ap_path_cost = 0;
    ap_path.reserve(100);
    std::vector<bool> ap_visited(graph_->GetMaxNodeId() + 1, false);
    double best_cost_so_far = cost_ub;
    while (!ap_stack.empty())
    {
        Snapshot ap_snap = ap_stack.back();
        ap_stack.pop_back();
        Link *ap_link = ap_snap.link;
        NodeId u = ap_link->ter_id;
        ++ap_info.iteration_num;
        if (ap_snap.stage == 1)
        {
            ap_path.pop_back();
            ap_path_delay -= ap_link->delay;
            ap_path_cost -= ap_link->cost;
            for (int i = 0; i < conflict_status_.size(); ++i)
            {
                conflict_status_[i].Remove(ap_link);
            }
            ap_visited[u] = false;
        }
        else
        {
            bool skip = false;
            for (int i = 0; i < conflict_status_.size(); ++i)
            {
                conflict_status_[i].Add(ap_link);
                if (conflict_status_[i].CoverConflictSet())
                {
                    skip = true;
                    for (int j = 0; j <= i; ++j)
                    {
                        conflict_status_[j].Remove(ap_link);
                    }
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            double new_delay = ap_path_delay + ap_link->delay;
            double new_cost = ap_path_cost + ap_link->cost;
            if (u == flow.to)
            {
                for (int i = 0; i < conflict_status_.size(); ++i)
                {
                    conflict_status_[i].Remove(ap_link);
                }
                if (flow.CheckDelayUb(new_delay) &&
                    flow.CheckDelayLb(new_delay) &&
                    new_cost < best_cost_so_far)
                {
                    ap_path.push_back(ap_link);
                    if (VerifyPath(ap_path))
                    {
                        best_cost_so_far = new_cost;
                        // 注意bp_path的第一跳是fake_link
                        ap_path_.clear();
                        ap_path_.reserve(ap_path.size() - 1);
                        for (int i = 1; i < ap_path.size(); ++i)
                        {
                            ap_path_.push_back(ap_path[i]);
                        }
                    }
                    ap_path.pop_back();
                }
            }
            else
            {
                // prune
                if (flow.CheckDelayUb(min_delay_to_dst[u] + new_delay) &&
                    min_cost_to_dst[u] + new_cost < best_cost_so_far &&
                    visit_info_->at(u).CheckMinCost(
                        flow.delay_lb - new_delay,
                        best_cost_so_far - new_cost))
                {
                    const std::vector<Link *> &egress_links =
                        graph_->GetEgressLinks(u);
                            // 新的排序方法效果不明显
                            // visit_info_->at(u).GetEgressLinksBasedOnDelayLb(
                            //     flow.delay_lb - new_delay);
                    if (!egress_links.empty())
                    {
                        ap_path.push_back(ap_link);
                        ap_path_delay += ap_link->delay;
                        ap_path_cost += ap_link->cost;
                        ap_visited[u] = true;
                        ap_snap.stage = 1;
                        ap_stack.push_back(ap_snap);
                        for (Link *link : egress_links)
                        {
                            if (link->status == Conflict)
                            {
                                continue;
                            }
                            NodeId node_id = link->ter_id;
                            if (ap_visited[node_id])
                            {
                                continue;
                            }
                            Snapshot snap;
                            snap.stage = 0;
                            snap.link = link;
                            ap_stack.push_back(snap);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < conflict_status_.size(); ++i)
                        {
                            conflict_status_[i].Remove(ap_link);
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < conflict_status_.size(); ++i)
                    {
                        conflict_status_[i].Remove(ap_link);
                    }
                }
            }
        }
    }
    // Enable links in excluded srlgs
    for (int srlg : excluded_srlgs_)
    {
        graph_->ChangeLinkStatus(srlg, Available);
    }
    for (int srlg : included_srlgs_)
    {
        std::vector<Link *> &links = graph_->GetSrlgGroup(srlg);
        if (links.size() == 1)
        {
            for (Link *link : graph_->GetEgressLinks(links[0]->source_id))
            {
                link->status = Available;
            }
        }
    }
    ap_info.total_time += clock() - start_time;
    return best_cost_so_far;
}
