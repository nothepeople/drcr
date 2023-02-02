#include <algorithm> // std::random_shuffle
#include <assert.h>
#include <cstdlib> // std::rand, std::srand
#include <fstream>
#include <iostream>

#include "graph.h"

namespace
{
    void ReadLinksFromFile(const std::string &file_path,
                           std::vector<Link> *links)
    {
        std::string line;
        std::ifstream in(file_path);
        if (in.fail())
        {
            std::cout << "File not found" << std::endl;
            return;
        }
        int line_cnt = 0;
        links->reserve(10000);
        while (getline(in, line) && in.good())
        {
            ++line_cnt;
            if (line_cnt == 1)
                continue;
            // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
            std::vector<std::string> buffer;
            std::string str;
            for (int i = 0; i < line.size(); ++i)
            {
                if (line[i] == ',')
                {
                    buffer.push_back(str);
                    str.clear();
                    continue;
                }
                str.push_back(line[i]);
            }
            assert(buffer.size() >= 8);
            links->emplace_back(
                atoi(buffer[0].c_str()), atoi(buffer[1].c_str()),
                atoi(buffer[2].c_str()), atof(buffer[4].c_str()),
                atoi(buffer[5].c_str()), atof(buffer[6].c_str()),
                atoi(buffer[7].c_str()));
            if (atoi(buffer[7].c_str()) > 0)
            {
                std::string tmp;
                for (int i = 0; i < str.size(); ++i)
                {
                    if (str[i] == '|')
                    {
                        links->back().srlgs.push_back(atoi(tmp.c_str()));
                        tmp.clear();
                        continue;
                    }
                    tmp.push_back(str[i]);
                }
                links->back().srlgs.push_back(atoi(tmp.c_str()));
            }
            // Sort link's srlgs
            std::sort(links->back().srlgs.begin(), links->back().srlgs.end());
        }
        in.close();
    }

    bool Compare(Link *a, Link *b)
    {
        return a->weight < b->weight;
    }

} // namespace

void Path::Print() const
{
    if (path_info.empty())
    {
        std::cout << "Empty Path\n";
        return;
    }
    std::cout << "Path:\nNode List: " << path_info[0];
    for (int i = 1; i < path_info.size(); ++i)
    {
        std::cout << "->" << path_info[i];
    }
    if (!path_link.empty())
    {
        std::cout << "\nLink List: ";
        for (int i = 0; i < path_link.size(); ++i)
        {
            if (i > 0)
            {
                std::cout << "->";
            }
            std::cout << path_link[i]->link_id;
            std::vector<int> srlgs = path_link[i]->srlgs;
            if (srlgs.size() > 1)
            {
                std::cout << "(" << srlgs[0];
                for (int j = 1; j < srlgs.size() - 1; ++j)
                {
                    std::cout << "," << srlgs[j];
                }
                std::cout << ")";
            }
        }
    }
    std::cout << ".\nCost is: " << cost << ", Delay is: "
              << delay << ".\n";
}

bool Path::Verify() const
{
    if (path_link.size() + 1 != path_info.size())
    {
        return false;
    }
    for (int i = 0; i < path_link.size(); ++i)
    {
        Link *link = path_link[i];
        if (link->source_id != path_info[i])
        {
            return false;
        }
        if (link->ter_id != path_info[i + 1])
        {
            return false;
        }
    }
    return true;
}

bool PathPair::VerfyLinkDisjoint() const
{
    if (ap_path.Empty() && bp_path.Empty())
    {
        return true;
    }
    if (!ap_path.Verify())
    {
        return false;
    }
    if (!bp_path.Verify())
    {
        return false;
    }
    std::unordered_set<Link *> links;
    if (ap_path.path_info.front() != bp_path.path_info.front())
    {
        return false;
    }
    if (ap_path.path_info.back() != bp_path.path_info.back())
    {
        return false;
    }
    for (Link *link : ap_path.path_link)
    {
        links.insert(link);
    }
    for (Link *link : bp_path.path_link)
    {
        links.insert(link);
    }
    if (links.size() < ap_path.path_link.size() +
                           bp_path.path_link.size())
    {
        return false;
    }
    return true;
}

void Graph::UpdateBpEgressLinks()
{
    bp_node_to_egress_links_ = node_to_egress_links_;
    for (NodeId i = 0; i < size_; ++i)
    {
        std::sort(bp_node_to_egress_links_[i].begin(),
                  bp_node_to_egress_links_[i].end(), Compare);
    }
    // for (int i = 0; i < bp_node_to_egress_links_.size(); ++i) {
    //     const std::vector<Link *>& egress_links = node_to_egress_links_[i];
    //     if (egress_links.size() > 0) {
    //         std::vector<Link *> available_links;
    //         available_links.reserve(egress_links.size());
    //         bp_node_to_egress_links_[i].clear();
    //         for (Link* link : egress_links) {
    //             if (link->status != Available) {
    //                 bp_node_to_egress_links_[i].push_back(link);
    //             } else {
    //                 available_links.push_back(link);
    //             }
    //         }
    //         bp_node_to_egress_links_[i].insert(
    //             bp_node_to_egress_links_[i].end(),
    //             available_links.begin(), available_links.end());
    //     }
    // }
}

bool Graph::IsCousinLinkPair(const Link *a, const Link *b)
{
    if (a->source_id == b->source_id && a->ter_id == b->ter_id &&
        a->cost == b->cost && a->delay == b->delay &&
        a->srlg_num == b->srlg_num)
    {
        for (int i = 0; i < a->srlgs.size(); ++i)
        {
            if (a->srlgs[i] != b->srlgs[i])
                return false;
        }
        return true;
    }
    return false;
}

Graph::Graph(const std::string &file_path)
{
    srlg_group_.reserve(100000);
    for (int i = 0; i < 10000; ++i)
    {
        srlg_group_.push_back({});
    }
    ReadLinksFromFile(file_path, &links_);
    // std::srand(0);
    // std::random_shuffle(links_.begin(), links_.end());
    max_node_id_ = -1;
    max_link_id_ = -1;
    max_srlg_id_ = -1;
    for (Link &link : links_)
    {
        if (link.source_id > max_node_id_)
            max_node_id_ = link.source_id;
        if (link.ter_id > max_node_id_)
            max_node_id_ = link.ter_id;
        if (link.link_id > max_link_id_)
        {
            max_link_id_ = link.link_id;
        }
        // if (link.link_id == 15125) {
        //     link.delay = 100;
        // }
        if (link.srlg_num != 0)
        {
            for (int i = 0; i < link.srlgs.size(); ++i)
            {
                if (link.srlgs[i] > max_srlg_id_)
                {
                    max_srlg_id_ = link.srlgs[i];
                }
                srlg_group_[link.srlgs[i]].push_back(&link);
                // if (link.srlgs[i] == 10) {
                //     link.delay = 6;
                // }
            }
        }
        nodes_.insert(link.source_id);
        nodes_.insert(link.ter_id);
    }
    // 为每个link设置一个专属的srlg，该srlg仅包含一条link.
    for (Link &link : links_)
    {
        link.srlg_num += 1;
        link.srlgs.push_back(++max_srlg_id_);
        srlg_group_[max_srlg_id_].push_back(&link);
    }
    size_ = max_node_id_ + 1;
    node_to_egress_links_.clear();
    node_to_ingress_links_.clear();
    node_to_egress_links_cost_.clear();
    node_to_ingress_links_cost_.clear();
    bp_node_to_egress_links_.clear();
    node_to_egress_links_.reserve(size_);
    node_to_ingress_links_.reserve(size_);
    node_to_egress_links_cost_.reserve(size_);
    node_to_ingress_links_cost_.reserve(size_);
    bp_node_to_egress_links_.reserve(size_);
    for (NodeId i = 0; i < size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        node_to_egress_links_.push_back({});
        bp_node_to_egress_links_.push_back({});
        node_to_ingress_links_.push_back({});
    }
    for (Link &link : links_)
    {
        node_to_egress_links_[link.source_id].push_back(&link);
        node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    node_to_egress_links_cost_ = node_to_egress_links_;
    node_to_ingress_links_cost_ = node_to_ingress_links_;
    FindCousinLinks();
}

Graph::Graph(const std::vector<Link> &link)
{
    srlg_group_.reserve(100000);
    for (int i = 0; i < 10000; ++i)
    {
        srlg_group_.push_back({});
    }
    links_ = link;
    // std::srand(0);
    // std::random_shuffle(links_.begin(), links_.end());
    max_node_id_ = -1;
    max_link_id_ = -1;
    max_srlg_id_ = -1;
    std::vector<Link *> index;
    index.reserve(5000);
    for (Link &link : links_)
    {
        if (link.source_id > max_node_id_)
            max_node_id_ = link.source_id;
        if (link.ter_id > max_node_id_)
            max_node_id_ = link.ter_id;
        if (link.link_id > max_link_id_)
        {
            max_link_id_ = link.link_id;
        }
        if (link.srlg_num != 0)
        {
            for (int i = 0; i < link.srlgs.size(); ++i)
            {
                if (link.srlgs[i] > max_srlg_id_)
                {
                    max_srlg_id_ = link.srlgs[i];
                }
                srlg_group_[link.srlgs[i]].push_back(&link);
            }
        }

        nodes_.insert(link.source_id);
        nodes_.insert(link.ter_id);
    }
    // 为每个link设置一个专属的srlg，该srlg仅包含一条link.
    for (Link &link : links_)
    {
        link.srlg_num += 1;
        link.srlgs.push_back(++max_srlg_id_);
        srlg_group_[max_srlg_id_].push_back(&link);
    }
    size_ = max_node_id_ + 1;
    node_to_egress_links_.clear();
    node_to_ingress_links_.clear();
    node_to_egress_links_cost_.clear();
    node_to_ingress_links_cost_.clear();
    bp_node_to_egress_links_.clear();
    node_to_egress_links_.reserve(size_);
    node_to_ingress_links_.reserve(size_);
    node_to_egress_links_cost_.reserve(size_);
    node_to_ingress_links_cost_.reserve(size_);
    bp_node_to_egress_links_.reserve(size_);
    for (NodeId i = 0; i < size_; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        node_to_egress_links_.push_back({});
        bp_node_to_egress_links_.push_back({});
        node_to_ingress_links_.push_back({});
    }
    for (Link &link : links_)
    {
        node_to_egress_links_[link.source_id].push_back(&link);
        node_to_ingress_links_[link.ter_id].push_back(&link);
    }
    node_to_egress_links_cost_ = node_to_egress_links_;
    node_to_ingress_links_cost_ = node_to_ingress_links_;
    FindCousinLinks();
}

void Graph::FindCousinLinks()
{
    std::vector<std::vector<Link *>> equivalent_groups;
    std::unordered_map<Link *, int> link_to_idx_map;
    equivalent_groups.reserve(links_.size());
    for (Link &link : links_)
    {
        bool flag = false;
        for (int idx = 0; idx < equivalent_groups.size(); ++idx)
        {
            std::vector<Link *> &equivalent_links = equivalent_groups[idx];
            if (IsCousinLinkPair(&link, equivalent_links[0]))
            {
                flag = true;
                equivalent_links.push_back(&link);
                link_to_idx_map.emplace(&link, idx);
                break;
            }
        }
        if (!flag)
        {
            link_to_idx_map.emplace(&link, equivalent_groups.size());
            equivalent_groups.push_back({&link});
        }
    }
    for (Link &link : links_)
    {
        int idx = link_to_idx_map.at(&link);
        link.cousins = equivalent_groups.at(idx);
    }
}

void Graph::SortLinks()
{
    for (NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_[i].begin(),
                  node_to_egress_links_[i].end(), Compare);
        std::sort(node_to_ingress_links_[i].begin(),
                  node_to_ingress_links_[i].end(), Compare);
    }
}

void Graph::SortLinks_cost()
{
    for(NodeId i = 0; i < size_; ++i)
    {
        std::sort(node_to_egress_links_cost_[i].begin(),
                  node_to_egress_links_cost_[i].end(),Compare);
        std::sort(node_to_ingress_links_cost_[i].begin(),
                  node_to_ingress_links_cost_[i].end(),Compare);
    }
}

void Flow::Print() const
{
    std::cout << "\n"
              << "--------"
              << "\n";
    std::cout << "Flow Id: " << id << ", Source node: " << from
              << ", Destination node: " << to << ", Delay upperbound: "
              << delay_ub << ", Delay lowerbound: " << delay_lb
              << ", Bandwidth requirement: " << bandwidth << ",\n"
              << "Is Delaydiff: " << is_diff << ", Diff Range: "
              << diff << ", Separation type: " << type << ", Theory opt cost: "
              << opt_cost << ", Range Type: " << range_type << ", Min Cost Delay:"
              << min_cost_delay << ", Min Delay Delay: " << min_delay_delay << "\n";
}

void Flow::PrintToCsv() const
{
    std::cout << id << "," << from << "," << to << "," << delay_ub
              << "," << delay_lb << "," << bandwidth << "," << is_diff << ","
              << diff << "," << type << "," << opt_cost << "," << range_type << ","
              << min_cost_delay << "," << min_delay_delay << ",";
}

Demand::Demand(const std::string &file_path)
{
    std::string line;
    std::ifstream in(file_path);
    if (in.fail())
    {
        std::cout << "File not found" << std::endl;
        return;
    }
    int case_cnt = 0;
    while (getline(in, line))
    {
        ++case_cnt;
        if (case_cnt == 1)
            continue;
        // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
        std::vector<std::string> buffer;
        std::string str;
        for (int i = 0; i < line.size(); ++i)
        {
            if (line[i] == ',')
            {
                buffer.push_back(str);
                str.clear();
                continue;
            }
            str.push_back(line[i]);
        }
        buffer.push_back(str);
        assert(buffer.size() >= 6);
        flows_.emplace_back(
            atoi(buffer[0].c_str()), atoi(buffer[1].c_str()),
            atoi(buffer[2].c_str()), atof(buffer[3].c_str()),
            atof(buffer[4].c_str()), atoi(buffer[5].c_str()),
            atoi(buffer[6].c_str()), atof(buffer[7].c_str()),
            static_cast<DisjointType>(atoi(buffer[8].c_str())),
            atof(buffer[9].c_str()));
        if (buffer.size() > 10)
        {
            flows_.back().range_type = atoi(buffer[10].c_str());
            flows_.back().min_cost_delay = atof(buffer[11].c_str());
            flows_.back().min_delay_delay = atof(buffer[12].c_str());
        }
    }

    in.close();
}

void ConflictStatus::Init(const Path &path)
{
    id_to_cnt_.clear();
    num_nozero_ids_ = 0;
    for (Link *link : path.path_link)
    {
        Add(link);
    }
}

void ConflictStatus::Init(const std::vector<Link *> &path)
{
    id_to_cnt_.clear();
    num_nozero_ids_ = 0;
    for (Link *link : path)
    {
        Add(link);
    }
}

void ConflictStatus::Add(const Link *link)
{
    for (int srlg_id : link->srlgs)
    {
        if (conflict_set_->CheckConflict(srlg_id))
        {
            if (id_to_cnt_[srlg_id] == 0)
            {
                ++num_nozero_ids_;
            }
            id_to_cnt_[srlg_id] += 1;
        }
    }
}
void ConflictStatus::Remove(const Link *link)
{
    for (int srlg_id : link->srlgs)
    {
        if (conflict_set_->CheckConflict(srlg_id))
        {
            id_to_cnt_[srlg_id] -= 1;
            if (id_to_cnt_[srlg_id] == 0)
            {
                --num_nozero_ids_;
            }
        }
    }
}

bool ConflictStatus::CoverConflictSet()
{
    return (num_nozero_ids_ >= conflict_set_->NumConflicts());
}

void Path::CompletePath()
{
    cost = 0;
    delay = 0;
    path_info.clear();
    path_info.push_back(path_link.front()->source_id);
    for (Link *link : path_link)
    {
        path_info.push_back(link->ter_id);
        delay += link->delay;
        cost += link->cost;
    }
}

// bool VisitInfo::FastCheckDominanceAndUpdate(double delay, double cost) {
//     auto it = delay_to_cost_.lower_bound(delay);
//     if (it == delay_to_cost_.end() || it->first >= delay + range_) {
//         delay_to_cost_.emplace(delay, cost);
//         return true;
//     }
//     if (it->first == delay) {
//         return false;
//     }
//     auto it2 = delay_to_cost_.lower_bound(it->first - range_);
//     if (it2->first < delay) {
//         return false;
//     }
//     delay_to_cost_.emplace(delay, cost);
//     return true;
// }

bool VisitInfo::FastCheckDominanceAndUpdate(double delay, double cost)
{
    auto it = delay_to_cost_.lower_bound(delay);
    if (it == delay_to_cost_.end() || it->first >= delay + range_)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    if (it->first == delay)
    {
        return false;
    }
    if (it != delay_to_cost_.begin())
    {
        double range_lb = it->first - range_;
        --it;
        if (it->first >= range_lb)
        {
            return false;
        }
    }
    delay_to_cost_.emplace(delay, cost);
    return true;
}

bool VisitInfo::CheckDominanceAndUpdate(double delay, double cost)
{
    auto it = delay_to_cost_.lower_bound(delay);
    if (it == delay_to_cost_.end())
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    if (it->first == delay)
    {
        if (it->second <= cost)
        {
            return false;
        }
        else
        {
            it->second = cost;
            return true;
        }
    }
    auto it2 = delay_to_cost_.upper_bound(delay - range_);
    while (it2->first < delay && it2->second > cost)
    {
        ++it2;
    }
    if (it2->first > delay)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    double range_ub = it2->first + range_;
    while (it->first <= range_ub && it->second > cost)
    {
        ++it;
    }
    if (it->first > range_ub)
    {
        delay_to_cost_.emplace(delay, cost);
        return true;
    }
    return false;
}

void VisitInfo::Process()
{
    auto it = delay_to_cost_.begin();
    delay_min_ = it->first - range_;
    min_cost_vector_.clear();
    min_cost_vector_.reserve(500);
    double delay = delay_min_;
    while (it != delay_to_cost_.end())
    {
        double min_cost = kMaxValue;
        auto it2 = it;
        // 加上1e-6解决浮点数的精度问题
        while (delay <= it->first + 1e-6)
        {
            while (it2 != delay_to_cost_.end() &&
                   it2->first < delay + range_ + delta_)
            {
                if (it2->second < min_cost)
                {
                    min_cost = it2->second;
                }
                ++it2;
            }
            min_cost_vector_.push_back(min_cost);
            delay += delta_;
        }
        ++it;
    }
}

double VisitInfo::GetMinCost(double delay_lb) const {
    if (delay_lb < delay_min_) {
        return kMaxValue;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= min_cost_vector_.size()) {
        return kMaxValue;
    }
    return min_cost_vector_.at(idx);
}

bool VisitInfo::CheckMinCost(double delay_lb, double cost_ub) const
{
    if (delay_lb < delay_min_)
    {
        return false;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= min_cost_vector_.size())
    {
        return false;
    }
    return min_cost_vector_.at(idx) < cost_ub;
}

void VisitInfo::Print() const
{
    for (auto &delay_and_cost : delay_to_cost_)
    {
        std::cout << "(" << delay_and_cost.first << ", "
                  << delay_and_cost.second << ") ";
    }
    std::cout << "\n";
}

void VisitInfo::SortEgressLinksBasedOnDelayBudget(
    const std::vector<Link*> all_egress_links,
    const std::vector<VisitInfo>& rechability_info) {
    delay_to_egress_links_.clear();
    delay_to_egress_links_.reserve(min_cost_vector_.size());
    double delay_lb = delay_min_;
    for (int j = 0; j < min_cost_vector_.size(); ++j) {
        delay_to_egress_links_.emplace_back();
        std::vector<Link*>& egress_links = delay_to_egress_links_.back();
        for (Link* link : all_egress_links) {
            NodeId u = link->ter_id;
            double cost = link->cost +
                rechability_info[u].GetMinCost(delay_lb - link->delay);
            link->weight = -cost;
        }
        egress_links = all_egress_links;
        std::sort(egress_links.begin(), egress_links.end(), Compare);
        delay_lb += delta_;
    }
}

const std::vector<Link *>& VisitInfo::GetEgressLinksBasedOnDelayLb(
    double delay_lb) const {
    if (delay_lb < delay_min_) {
        return empty_link_set_;
    }
    int idx = DelayToIndex(delay_lb);
    if (idx >= delay_to_egress_links_.size()) {
        return empty_link_set_;
    }
    return delay_to_egress_links_.at(idx);
}
