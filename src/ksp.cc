#include <algorithm>
#include <cstdlib> // Header file needed to use srand and rand
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stack>
#include <unordered_map>
#include <algorithm>
#include <unistd.h>
// #include <stack>
#include "ksp.h"

// namespace
// {
//     void mem_usage(double &vm_usage, double &resident_set)
//     {
//         vm_usage = 0.0;
//         resident_set = 0.0;
//         // get info from proc directory
//         std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);
//         // create some variables to get info
//         std::string pid, comm, state, ppid, pgrp, session, tty_nr;
//         std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
//         std::string utime, stime, cutime, cstime, priority, nice;
//         std::string O, itrealvalue, starttime;
//         unsigned long vsize;
//         long rss;
//         stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session
//                     >> tty_nr >> tpgid >> flags >> minflt >> cminflt
//                     >> majflt >> cmajflt >> utime >> stime >> cutime
//                     >> cstime >> priority >> nice >> O >> itrealvalue
//                     >> starttime >> vsize >> rss;
//         stat_stream.close();
//         // for x86-64 is configured to use 2MB pages
//         long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
//         vm_usage = vsize / 1024.0;
//         resident_set = rss * page_size_kb;
//     }

// } // namespace

Ksp::~Ksp()
{
    if (bp_visited_)
    {
        delete[] bp_visited_;
        bp_visited_ = nullptr;
    }
    if (dij_visited_)
    {
        delete[] dij_visited_;
        dij_visited_ = nullptr;
    }
}

void Ksp::SetupTopology(Graph *graph)
{
    graph_ = graph;
    num_nodes_ = graph_->NodeSize();
    dij_visited_ = new bool[num_nodes_];
    bp_visited_ = new bool[num_nodes_];
    dst_delay_map_ = new double[num_nodes_];
    partial_dst_delay_map_ = new double[num_nodes_];
    node_to_dis_.reserve(num_nodes_);
    partial_dis_.reserve(num_nodes_);
    for (NodeId i = 0; i < num_nodes_; ++i)
    {
        node_to_dis_.push_back({});
        partial_dis_.push_back({});
    }
}

void Ksp::remove_used_paths(NodeId id, const Path &cur_path)
{
    for (int i = 0; i < ap_.size(); ++i)
    {
        int k = 0;
        for (int j = 0; j < ap_[i].path_info.size() - 1; ++j)
        {
            if (ap_[i].path_info[j] != cur_path.path_info[k])
                break;
            if (ap_[i].path_info[j] == id)
            {
                ap_[i].path_link[j]->status = Conflict;
                break;
            }
            ++k;
            if (k > cur_path.path_info.size() - 1)
                break;
        }
    }
}

void Ksp::retrieve_used_paths(NodeId id, const Path &cur_path)
{
    for (int i = 0; i < ap_.size(); ++i)
    {
        int k = 0;
        for (int j = 0; j < ap_[i].path_info.size() - 1; ++j)
        {
            if (ap_[i].path_info[j] != cur_path.path_info[k])
                break;
            if (ap_[i].path_info[j] == id)
            {
                ap_[i].path_link[j]->status = Available;
                break;
            }
            ++k;
            if (k > cur_path.path_info.size() - 1)
                break;
        }
    }
}

void Ksp::get_cost_and_delay(Path &path)
{
    double cost = 0;
    double delay = 0;
    for (Link *link : path.path_link)
    {
        delay += link->delay;
        cost += link->cost;
    }
    path.cost = cost;
    path.delay = delay;
}

Path Ksp::revert_path(Path &path)
{
    Path res;
    for (int i = path.path_info.size() - 1; i >= 0; --i)
    {
        res.path_info.push_back(path.path_info[i]);
    }
    for (int i = path.path_link.size() - 1; i >= 0; --i)
    {
        res.path_link.push_back(path.path_link[i]);
    }
    return res;
}

Path Ksp::find_kth_path(const Path &path)
{
    Path part_path;
    for (int i = 0; i < path.path_info.size() - 1; ++i)
    {
        Path total_path;
        int from = path.path_info[i];
        int to = path.path_info[path.path_info.size() - 1];
        remove_used_paths(path.path_info[i], path);
        path.path_link[i]->status = Conflict;

        for (int j = 0; j < num_nodes_; ++j)
        {
            dij_visited_[j] = false;
        }
        for (int j = 0; j < i; ++j)
        {
            dij_visited_[path.path_info[j]] = true;
        }

        PartialDijkstra(from, to, partial_dst_delay_map_);
        retrieve_used_paths(path.path_info[i], path);
        Path tmp = partial_dis_[to];
        path.path_link[i]->status = Available;
        if (partial_dis_[to].path_info.size() == 0)
        {
            part_path.path_info.push_back(path.path_info[i]);
            part_path.path_link.push_back(path.path_link[i]);
            continue;
        }
        for (int j = 0; j < part_path.path_info.size(); ++j)
        {
            total_path.path_info.push_back(part_path.path_info[j]);
        }
        for (int j = 0; j < part_path.path_link.size(); ++j)
        {
            total_path.path_link.push_back(part_path.path_link[j]);
        }

        for (int j = 0; j < partial_dis_[to].path_info.size(); ++j)
        {
            total_path.path_info.push_back(partial_dis_[to].path_info[j]);
        }
        for (int j = 0; j < partial_dis_[to].path_link.size(); ++j)
        {
            total_path.path_link.push_back(partial_dis_[to].path_link[j]);
        }
        part_path.path_info.push_back(path.path_info[i]);
        part_path.path_link.push_back(path.path_link[i]);
        get_cost_and_delay(total_path);
        bp_.push_back(total_path);
    }
    int cnt = 0;
    int mini_delay = kMaxDelay;
    std::sort(bp_.begin(), bp_.end(), [](const Path &pa, const Path &pb)
              { return pa.delay > pb.delay; });
    bp_.erase(unique(bp_.begin(), bp_.end()), bp_.end());
    std::vector<Path>::iterator itr = bp_.begin();
    while (itr != bp_.end())
    {
        bool flag = false;
        for (int j = 0; j < ap_.size(); ++j)
        {
            if (*itr == ap_[j])
            {
                itr = bp_.erase(itr);
                flag = true;
                break;
            }
        }
        if (!flag)
            itr++;
    }
    for (int i = bp_.size() - 1; i >= 0; --i)
    {
        if (bp_[i].delay > mini_delay)
            break;
        if (bp_[i].delay < mini_delay)
        {
            cnt = i;
            mini_delay = bp_[i].delay;
        }
        else if (bp_[i].delay == mini_delay && bp_[i].path_info.size() < bp_[cnt].path_info.size())
        {
            cnt = i;
            mini_delay = bp_[i].delay;
        }
    }
    Path res = bp_[cnt];
    std::vector<Path>::iterator it = bp_.begin() + cnt;
    bp_.erase(it);

    return res;
}

void Ksp::ksp(const Flow &flow)
{
    Path tmp = node_to_dis_[flow.from];
    tmp = revert_path(tmp);
    get_cost_and_delay(tmp);
    ap_.push_back(tmp);
    int cnt1 = 0;
    int cnt2 = 0;
    bool flag1 = false;
    bool flag2 = false;
    int num = 0;
    while (!flag1 || !flag2)
    {
        ++num;
        // std::cout << num << std::endl;
        if (num == 300)
            int x = 9;
        Path bp_path = find_kth_path(ap_[ap_.size() - 1]);
        ap_.push_back(bp_path);
        // std::vector<Path>::iterator itr = bp_.begin();
        // while (itr != bp_.end())
        // {
        //     if (*itr == bp_[cnt])
        //     {
        //         itr = bp_.erase(itr);
        //         break;
        //     }
        //     itr++;
        // }
        // // bp_.pop_back();
        if (!flag1 && ap_[ap_.size() - 1].delay > flow.delay_lb)
        {
            flag1 = true;
            cnt1 = ap_.size() - 1;
        }
        if (!flag2 && ap_[ap_.size() - 1].delay > flow.delay_ub)
        {
            flag2 = true;
            cnt2 = ap_.size() - 2;
        }
    }
    std::vector<Path> p;
    for (int i = 0; i < ap_.size(); ++i)
    {
        if (ap_[i].delay >= flow.delay_lb && ap_[i].delay <= flow.delay_ub)
        {
            p.push_back(ap_[i]);
        }
    }
    int cnt = 0;
    if(p.size() == 0)
    {
        std::cout << "No Result!" << std::endl;
        return;
    }
    for (int i = 0; i < p.size(); ++i)
    {
        if (p[i].cost < p[cnt].cost)
            cnt = i;
    }
    
    // Path res;
    // for (int i = p[cnt].path_info.size() - 1; i >= 0; --i)
    // {
    //     res.path_info.push_back(p[cnt].path_info[i]);
    // }
    // for (int i = p[cnt].path_link.size() - 1; i >= 0; --i)
    // {
    //     res.path_link.push_back(p[cnt].path_link[i]);
    // }
    // res.cost = p[cnt].cost;
    // res.delay = p[cnt].delay;
    results_ = p[cnt];
    std::cout << "Ksp iteration time: " << num << std::endl;
}

Path Ksp::FindPath(const Flow &flow)
{
    // Initialization
    // do dijkstra from end to every other nodes,
    // from two perspectives--cost and delay
    // store the result of dijkstras in cost_map and delay_map
    flow_ = flow;

    start_time_ = clock();
    for (int i = 0; i < num_nodes_; ++i)
    {
        dij_visited_[i] = false;
    }
    ap_.clear();
    bp_.clear();
    results_.clear();
    DstDijkstra(flow_.to, flow_.from, dst_delay_map_);
    end_time_ = clock();
    std::cout << "Ksp initialization time: "
              << double(end_time_ - start_time_) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";

    total_cost_ = kMaxValue;
    time_ms_cost_pairs_.clear();
    // Timer starts
    start_time_ = clock();
    ksp(flow);
    // Print time elapse
    end_time_ = clock();
    std::cout << "Ksp time: "
              << double(end_time_ - start_time_) / CLOCKS_PER_SEC * 1000
              << "(ms).\n\n";

    return results_;
}

void Ksp::update_node_path(NodeId pre_node, NodeId cur_node, Link *link)
{
    if (pre_node == -1)
    {
        node_to_dis_[cur_node].path_info.push_back(cur_node);
    }
    else
    {
        node_to_dis_[cur_node].path_info = node_to_dis_[pre_node].path_info;
        node_to_dis_[cur_node].path_info.push_back(cur_node);
        node_to_dis_[cur_node].path_link = node_to_dis_[pre_node].path_link;
        node_to_dis_[cur_node].path_link.push_back(link);
    }
    int size = node_to_dis_[cur_node].path_info.size();
}

void Ksp::update_partial_node_path(NodeId pre_node, NodeId cur_node, Link *link)
{
    if (pre_node == -1)
    {
        partial_dis_[cur_node].path_info.push_back(cur_node);
    }
    else
    {
        partial_dis_[cur_node].path_info = partial_dis_[pre_node].path_info;
        partial_dis_[cur_node].path_info.push_back(cur_node);
        partial_dis_[cur_node].path_link = partial_dis_[pre_node].path_link;
        partial_dis_[cur_node].path_link.push_back(link);
    }
    int size = partial_dis_[cur_node].path_info.size();
}

// void Ksp::HeuristicDijkstra(NodeId source_id, NodeId to_id)
// {

// }

void Ksp::DstDijkstra(
    NodeId source_id, NodeId to_id, double *node_to_dst_dis)
{

    for (int i = 0; i < num_nodes_; ++i)
    {
        node_to_dst_dis[i] = kMaxValue;
        node_to_dis_[i].clear();
    }
    node_to_dst_dis[source_id] = 0;
    std::priority_queue<Pathnode, std::vector<Pathnode>> heap;
    heap.push(Pathnode(-1, source_id, 0, nullptr, 0));
    int cnt = 0;
    while (!heap.empty())
    {
        Pathnode u = heap.top();
        double weight_u_to_dst = heap.top().dis;
        heap.pop();
        if (dij_visited_[u.cur])
            continue;
        // std::cout << ++cnt << std::endl;
        dij_visited_[u.cur] = true;
        update_node_path(u.pre, u.cur, u.prev_link);
        if (u.cur == to_id)
            break;
        for (Link *link : graph_->GetIngressLinks(u.cur))
        {
            if (link->status == Conflict)
            {
                continue;
            }
            NodeId v = link->source_id;
            if (dij_visited_[v])
                continue;
            double weight = link->delay;
            if (node_to_dst_dis[v] > weight_u_to_dst + weight)
            {
                node_to_dst_dis[v] = weight_u_to_dst + weight;
                heap.push(Pathnode(u.cur, v, node_to_dst_dis[v], link, node_to_dst_dis[v]));
            }
        }
    }
}

void Ksp::PartialDijkstra(
    NodeId source_id, NodeId to_id, double *node_to_dst_dis)
{
    for (int i = 0; i < num_nodes_; ++i)
    {
        node_to_dst_dis[i] = kMaxValue;
        partial_dis_[i].path_info.clear();
        partial_dis_[i].path_link.clear();
        partial_dis_[i].cost = kMaxValue;
        partial_dis_[i].delay = kMaxDelay;
    }
    node_to_dst_dis[source_id] = 0;
    std::priority_queue<Pathnode, std::vector<Pathnode>> heap;
    heap.push(Pathnode(-1, source_id, 0, nullptr, dst_delay_map_[source_id]));
    while (!heap.empty())
    {
        Pathnode u = heap.top();
        double weight_u_to_dst = heap.top().dis;
        heap.pop();
        if (dij_visited_[u.cur])
            continue;
        dij_visited_[u.cur] = true;
        update_partial_node_path(u.pre, u.cur, u.prev_link);
        if (u.cur == to_id)
            break;
        for (Link *link : graph_->GetEgressLinks(u.cur))
        {
            if (link->status == Conflict)
            {
                continue;
            }
            NodeId v = link->ter_id;
            if (dij_visited_[v])
                continue;
            double weight = link->delay;
            if (node_to_dst_dis[v] > weight_u_to_dst + weight)
            {
                node_to_dst_dis[v] = weight_u_to_dst + weight;
                heap.push(Pathnode(u.cur, v, node_to_dst_dis[v], link, dst_delay_map_[v] + node_to_dst_dis[v]));
            }
        }
    }
}

double Ksp::get_optimality()
{
    return optimality_;
}

void Ksp::print()
{
    std::cout << "\nTime(ms)  Optimality\n";
    std::pair<double, double> best_res_in_20ms;
    for (const std::pair<double, double> &time_cost : time_ms_cost_pairs_)
    {
        optimality_ = static_cast<double>(total_cost_) / time_cost.second;
        if (time_cost.first < 20)
            best_res_in_20ms = time_cost;
        if (optimality_ > 0.8)
        {
            std::cout << time_cost.first << "   " << optimality_ << std::endl;
        }
    }
    optimality_ =
        static_cast<double>(total_cost_) / best_res_in_20ms.second;
    std::cout << "Best Result In 20(ms):  Time: " << best_res_in_20ms.first
              << " (ms)  Optimality: " << optimality_ << std::endl;
}
