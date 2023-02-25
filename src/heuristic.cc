#include "heuristic.h"
#include "shortest_path.h"
#include <queue>
#include <stack>

namespace
{
    double INF = 10000000000.0;
    struct NodeCostPair
    {
        NodeCostPair(NodeId id_in, double cost_in)
            : id(id_in), cost(cost_in) {}
        friend bool operator<(const NodeCostPair &a, const NodeCostPair &b)
        {
            return a.cost > b.cost;
        }
        NodeId id;
        double cost;
    };

    struct LinkEtaPair
    {
        LinkEtaPair(Link link_in, double eta_in)
            : link(link_in), eta(eta_in) {}
        friend bool operator<(const LinkEtaPair &a, const LinkEtaPair &b)
        {
            return a.eta > b.eta;
        }
        Link link;
        double eta;
    };

    struct LinkNegetaPair
    {
        LinkNegetaPair(Link link_in, double eta_in)
            : link(link_in), eta(eta_in) {}
        friend bool operator<(const LinkNegetaPair &a, const LinkNegetaPair &b)
        {
            return a.eta < b.eta;
        }
        Link link;
        double eta;
    };

    struct simplelink
    {
        int to;
        int cap;
        simplelink(int to_, int cap_):to(to_),cap(cap_){}
    };

    bool SearchAugPath(std::vector<std::vector<simplelink>> &egresslink,
                int source, int destination, std::vector<int> &parent, std::vector<bool> &vis)
    {
        if(source == destination)
        {
            return true;
        }
        vis[source] = true;
        for(int i = 0; i < egresslink[source].size(); i++)
        {   
            if((egresslink[source][i].cap == 1) && (vis[egresslink[source][i].to] == false))
            {
                int temp = parent[egresslink[source][i].to];
                parent[egresslink[source][i].to] = source;
                if(SearchAugPath(egresslink, egresslink[source][i].to, destination, parent, vis))
                {
                    vis[source] = false;
                    return true;
                }
                parent[egresslink[source][i].to] = temp;
            }
        }
        vis[source] = false;
        return false;
    }

    struct LinkTagPair
    {   
        LinkTagPair(Link* link_in, int tag_in):
            link(link_in),tag(tag_in){}
        Link *link;
        int tag;
    };

    double CalCost(const std::vector<Link*> &path)
    {
        double cost = 0;
        for(Link *link : path)
        {
            cost += link->cost;
        }
        return cost;
    }

    double CalDelay(const std::vector<Link*> &path)
    {
        double delay = 0;
        for(Link *link : path)
        {
            //std::cout<<link->source_id<<"->";
            delay += link->delay;
        }
        //std::cout<<"*";
        return delay;
    }

    bool IsPathEqual(const std::vector<Link*> &P1, const std::vector<Link*> &P2)
    {
        if(P1.size() != P2.size())
        {
            return false;
        }
        for(int i = 0; i < P1.size(); i++)
        {
            if(P1[i] != P2[i])
            {
                return false;
            }
        }
        return true;
    }
}//namespace

void EffSol::InitialEff()
{
    N1.clear();
    N2.clear();
    Nf.clear();
    G1 = nullptr;
    G2 = nullptr;
    eff_part_path.clear();
}

void EffSol::SplitGraph(const Flow &flow)
{
    std::vector<std::vector<simplelink>> Egresslink(graph_->GetMaxNodeId()+1);
    Egresslink.reserve(graph_->GetMaxNodeId()+1);
    for (NodeId i = 0; i < graph_->GetMaxNodeId()+1; ++i)
    {
        // assert(nodes_.find(i) != nodes_.end());
        Egresslink.push_back({});
    }
    for(Link link: graph_->GetLink())
    {
        Egresslink[link.source_id].emplace_back(link.ter_id,1);
        Egresslink[link.ter_id].emplace_back(link.source_id,0);
    }
    std::vector<int> parent(graph_->GetMaxNodeId()+1,-1);
    std::vector<bool> vis(graph_->GetMaxNodeId()+1, false);
    while(SearchAugPath(Egresslink, flow.from, flow.to, parent, vis))
    {
        int v;
        for(v = flow.to; v != flow.from; v = parent[v])
        {
            int u = parent[v];
            for(simplelink &link : Egresslink[u])
            {
                if((link.to == v) && (link.cap == 1))
                {
                    link.cap = 0;
                    break;
                }
            }
            for(simplelink &link : Egresslink[v])
            {
                if((link.to == u) && (link.cap == 0))
                {
                    link.cap = 1;
                    break;
                }
            }
        }
        std::fill(vis.begin(), vis.end(), false);
    }
    std::vector<bool> visited(graph_->GetMaxNodeId()+1, false);
    std::queue<int> q;
    q.push(flow.from);
    while(!q.empty())
    {
        int u = q.front();
        q.pop();
        visited[u] = true;
        for(int i = 0; i < Egresslink[u].size(); i++)
        {
            if(Egresslink[u][i].cap != 0)
            {
                if(visited[Egresslink[u][i].to] == false)
                {
                    q.push(Egresslink[u][i].to);
                }
            }
        }
    }
    if(visited[flow.from] && visited[flow.to])
    {
        std::cout<<"can not split the graph!"<<std::endl;
        assert(true);
    }
    for(Link link : graph_->GetLink())
    {
        if((visited[link.source_id] == true) && (visited[link.ter_id] == false))
        {
            if(link.source_id == flow.from)
            {
                Nf.insert(link.ter_id);
            }else{
                Nf.insert(link.source_id);
            }
        }
    }
    for(int i = 0; i <= graph_->GetMaxNodeId(); i++)
    {
        if(Nf.find(i) != Nf.end())
        {
            continue;
        }
        else if(visited[i] == true)
        {
            N1.insert(i);
        }else if(visited[i] == false)
        {
            N2.insert(i);
        }
    }
    std::vector<Link> G1_link;
    std::vector<Link> G2_link;
    G1_link.reserve(graph_->GetMaxLinkId()+1);
    G2_link.reserve(graph_->GetMaxLinkId()+1);
    for(Link link : graph_->GetLink())
    {
        if((N2.find(link.ter_id) != N2.end()) || (N2.find(link.source_id) != N2.end()))
        {
            G2_link.push_back(link);
        }else{
            G1_link.push_back(link);
        }
    }
    G1 = new Graph(G1_link);
    G2 = new Graph(G2_link);
    /*std::cout<<G1->GetMaxNodeId()<<std::endl;*/
    /*std::cout<<"N1:";
    for(auto it = N1.begin();it!=N1.end();it++)
    {
        std::cout<<*it<<" ";
    }
    std::cout<<std::endl;
    std::cout<<"Nf:";
    for(auto it = Nf.begin();it!=Nf.end();it++)
    {
        std::cout<<*it<<" ";
    }
    
    std::cout<<std::endl;
    std::cout<<"N2:";
    for(auto it = N2.begin();it!=N2.end();it++)
    {
        std::cout<<*it<<" ";
    }
    std::cout<<std::endl;*/
    /*for(Link link : G1->GetLink())
    {
        std::cout<<link.source_id<<"->"<<link.ter_id<<std::endl;
    }*/
}

void EffSol::UpdatePartPath(int &k)
{
    for(auto it = Nf.begin(); it != Nf.end(); it++)
    {
        std::vector<Link*> reverse_path;
        std::vector<Link*> path;
        for(int u = *it; parent[u] != nullptr; u = parent[u]->source_id)
        {
            //std::cout<<u<<" ";
            reverse_path.push_back(parent[u]);
        }
        //std::cout<<std::endl;
        path.reserve(reverse_path.size());
        for(int i = reverse_path.size()-1; i >= 0; i--)
        {
            path.push_back(reverse_path[i]);
        }
        if(k == 0)
        {
            eff_part_path[*it].insert({k,path});
        }else if(k < 0){
            std::vector<Link*> old_path = eff_part_path[*it][k+1];
            if(IsPathEqual(old_path,path)){
                k = k + 1;
            }else{
                eff_part_path[*it].insert({k,path});
            }
        }else if(k > 0){
            std::vector<Link*> old_path = eff_part_path[*it][k-1];
            if(IsPathEqual(old_path,path)){
                k = k - 1;
            }else{
                eff_part_path[*it].insert({k,path});
            }
        }
    }
}

void EffSol::InitialTree(const Flow &flow)
{
    C.clear();
    R.clear();
    P.clear();
    C_.clear();
    R_.clear();
    P_.clear();
    parent.clear();
    //1.generate the shortest path tree.
    //lamda = 0 at first.
    std::priority_queue<NodeCostPair,std::vector<NodeCostPair>> pq;
    pq.push(NodeCostPair(flow.from,0));
    //std::vector<Link*> parent(G1->GetMaxNodeId()+1, nullptr);
    std::vector<bool> vis(G1->GetMaxNodeId()+1, false);
    std::vector<double> weight(G1->GetMaxNodeId()+1, kMaxValue);
    parent.reserve(G1->GetMaxNodeId()+1);
    for(int i = 0; i < G1->GetMaxNodeId()+1; i++)
    {
        parent.push_back(nullptr);
    }
    while(!pq.empty())
    {
        NodeId u = pq.top().id;
        double weight_src_to_u = pq.top().cost;
        pq.pop();
        if(vis[u] == true)
        {
            continue;
        }
        vis[u] = true;
        const std::vector<Link*> &egress_link = G1->GetEgressLinks(u);
        for(Link *link : egress_link)
        {
            NodeId v = link->ter_id;
            if(vis[v] == true)
            {
                continue;
            }
            if(weight[v] > weight_src_to_u + link->cost)
            {
                weight[v] = weight_src_to_u + link->cost;
                parent[v] = link;
                pq.push(NodeCostPair(v,weight[v]));
            }
        }
    }

    //2.update P according to parent array.
    //3.update C and R according to parent array.
    P.reserve(G1->GetMaxNodeId()+1);
    P_.reserve(G1->GetMaxNodeId()+1);
    C.reserve(G1->GetMaxNodeId()+1);
    C_.reserve(G1->GetMaxNodeId()+1);
    R.reserve(G1->GetMaxNodeId()+1);
    R_.reserve(G1->GetMaxNodeId()+1);
    for(int i = 0; i <= G1->GetMaxNodeId(); i++)
    {
        P.push_back(std::unordered_set<int> ());
        P_.push_back(std::unordered_set<int> ());
        C.push_back(kMaxValue);
        C_.push_back(kMaxValue);
        R.push_back(kMaxValue);
        R_.push_back(kMaxValue);
    }
    std::unordered_set<int> G1_node = G1->GetNode();
    for(auto it = G1_node.begin(); it!= G1_node.end(); it++)
    {
        if(*it == flow.from)
        {
            C[*it] = 0;
            R[*it] = 0;
            continue;
        }
        double C_u_to_it = 0;
        double R_u_to_it = 0;
        for(int u = *it; parent[u] != nullptr; u = parent[u]->source_id)
        {
            P[u].insert(*it);
            C_u_to_it += parent[u]->cost;
            R_u_to_it += parent[u]->delay;
        }
        P[flow.from].insert(*it);
        C[*it] = C_u_to_it;
        R[*it] = R_u_to_it;
    }
}

void EffSol::CalPartPath(const Flow &flow)
{
    eff_part_path.reserve(G1->GetMaxNodeId()+1);
    for(int i = 0; i < G1->GetMaxNodeId()+1; i++)
    {
        eff_part_path.push_back(std::unordered_map<int,std::vector<Link*>>());
    }
    double lamda = 0;
    int k = 0;
    InitialTree(flow);
    UpdatePartPath(k);
    std::priority_queue<LinkEtaPair, std::vector<LinkEtaPair>> heap;
    for(Link link : G1->GetLink())
    {
        double d1 = C[link.source_id] + link.cost - C[link.ter_id];
        double d2 = R[link.source_id] + link.delay - R[link.ter_id];
        if(d2 < 0)
        {
            heap.push(LinkEtaPair(link,-d1/d2));
        }else{
            heap.push(LinkEtaPair(link,INF));
        }
    }
    lamda = heap.top().eta;
    Link lamda_link = heap.top().link;
    heap.pop();
    while(abs(lamda - INF) > 0.001)
    {
        //std::cout<<"hah"<<parent[8]->source_id<<std::endl;
        //std::cout<<lamda<<" "<<lamda_link.source_id<<" "<<lamda_link.ter_id<<std::endl;
        //if A after changed not a tree, i \in P(j)
        if(P[lamda_link.ter_id].find(lamda_link.source_id) == P[lamda_link.ter_id].end())
        {
            //is a tree.
            for(auto it = G1->GetNode().begin(); it != G1->GetNode().end(); it++)
            {
                if(P[lamda_link.ter_id].find(*it) == P[lamda_link.ter_id].end())
                {
                    C_[*it] = C[*it];
                    R_[*it] = R[*it];
                }else{
                    C_[*it] = C[*it] + 
                                C[lamda_link.source_id] + 
                                lamda_link.cost - 
                                C[lamda_link.ter_id];
                    R_[*it] = R[*it] + 
                                R[lamda_link.source_id] + 
                                lamda_link.delay - 
                                R[lamda_link.ter_id];
                }
                if(*it == lamda_link.source_id)
                {
                    //P_[i] = P[i] + P[j]
                    P_[lamda_link.source_id] = P[lamda_link.source_id];
                    P_[lamda_link.source_id].insert(P[lamda_link.ter_id].begin(),
                                                P[lamda_link.ter_id].end());
                }else if(*it == parent[lamda_link.ter_id]->source_id)
                {
                    //P_[l] = P[l] - P[j]
                    P_[*it].clear();
                    for(auto it_ = P[*it].begin(); it_ != P[*it].end(); it_++)
                    {
                        if(P[lamda_link.ter_id].find(*it_) == P[lamda_link.ter_id].end())
                        {
                            P_[*it].insert(*it_);
                        }
                    }

                }else{
                    P_[*it] = P[*it];
                }
            }
            //update the tree
            /*std::cout<<lamda_link.source_id<<" "<<lamda_link.ter_id<<std::endl;
            std::cout<<parent[lamda_link.source_id]->source_id<<" "<<parent[lamda_link.source_id]->ter_id<<std::endl;
            std::cout<<parent[lamda_link.ter_id]->source_id<<" "<<parent[lamda_link.ter_id]->ter_id<<std::endl;*/
            parent[lamda_link.ter_id] = new Link(lamda_link.link_id,lamda_link.source_id,lamda_link.ter_id, lamda_link.cost, lamda_link.bandwidth
                                                ,lamda_link.delay,lamda_link.srlg_num);
            k += 1;
            //std::cout<<"l"<<std::endl;
            UpdatePartPath(k);
            //std::cout<<"r"<<std::endl;
            C = C_;
            R = R_;
            P = P_;
            for(Link link : G1->GetLink())
            {
                if((P[lamda_link.ter_id].find(link.source_id) != P[lamda_link.ter_id].end()) ||
                    P[lamda_link.ter_id].find(link.ter_id) != P[lamda_link.ter_id].end())
                {
                    double d1 = C[link.source_id] + link.cost - C[link.ter_id];
                    double d2 = R[link.source_id] + link.delay - R[link.ter_id];
                    if(d2 < 0)
                    {
                        heap.push(LinkEtaPair(link,-d1/d2));
                    }else{
                        heap.push(LinkEtaPair(link,INF));
                    }
                }
            }
            lamda = heap.top().eta;
            lamda_link = heap.top().link;
            heap.pop();
        }else{
            //is not a tree
            break;
        }
    }
    //std::cout<<2<<std::endl;
    lamda = 0;
    k = 0;
    InitialTree(flow);
    //std::cout<<1<<std::endl;
    std::priority_queue<LinkNegetaPair, std::vector<LinkNegetaPair>> negheap;
    for(Link link : G1->GetLink())
    {
        double d1 = C[link.source_id] + link.cost - C[link.ter_id];
        double d2 = R[link.source_id] + link.delay - R[link.ter_id];
        if(d2 > 0)
        {
            negheap.push(LinkNegetaPair(link,-d1/d2));
        }else{
            negheap.push(LinkNegetaPair(link,-INF));
        }
    }
    lamda = negheap.top().eta;
    lamda_link = negheap.top().link;
    negheap.pop();
    while(abs(lamda+INF) > 0.001)
    {
        //std::cout<<lamda<<std::endl;
        //if A after changed not a tree, i \in P(j)
        if(P[lamda_link.ter_id].find(lamda_link.source_id) == P[lamda_link.ter_id].end())
        {
            //is a tree.
            for(auto it = G1->GetNode().begin(); it != G1->GetNode().end(); it++)
            {
                if(P[lamda_link.ter_id].find(*it) == P[lamda_link.ter_id].end())
                {
                    C_[*it] = C[*it];
                    R_[*it] = R[*it];
                }else{
                    C_[*it] = C[*it] + 
                                C[lamda_link.source_id] + 
                                lamda_link.cost - 
                                C[lamda_link.ter_id];
                    R_[*it] = R[*it] + 
                                R[lamda_link.source_id] + 
                                lamda_link.delay - 
                                R[lamda_link.ter_id];
                }
                if(*it == lamda_link.source_id)
                {
                    //P_[i] = P[i] + P[j]
                    P_[lamda_link.source_id] = P[lamda_link.source_id];
                    P_[lamda_link.source_id].insert(P[lamda_link.ter_id].begin(),
                                                P[lamda_link.ter_id].end());
                }else if(*it == parent[lamda_link.ter_id]->source_id)
                {
                    //P_[l] = P[l] - P[j]
                    P_[*it].clear();
                    for(auto it_ = P[*it].begin(); it_ != P[*it].end(); it_++)
                    {
                        if(P[lamda_link.ter_id].find(*it_) == P[lamda_link.ter_id].end())
                        {
                            P_[*it].insert(*it_);
                        }
                    }

                }else{
                    P_[*it] = P[*it];
                }
            }
            //update the tree
            parent[lamda_link.ter_id] = new Link(lamda_link.link_id,lamda_link.source_id,lamda_link.ter_id, lamda_link.cost, lamda_link.bandwidth
                                                ,lamda_link.delay,lamda_link.srlg_num);
            k -= 1;
            UpdatePartPath(k);
            C = C_;
            R = R_;
            P = P_;
            for(Link link : G1->GetLink())
            {
                if((P[lamda_link.ter_id].find(link.source_id) != P[lamda_link.ter_id].end()) ||
                    P[lamda_link.ter_id].find(link.ter_id) != P[lamda_link.ter_id].end())
                {
                    double d1 = C[link.source_id] + link.cost - C[link.ter_id];
                    double d2 = R[link.source_id] + link.delay - R[link.ter_id];
                    if(d2 > 0)
                    {
                        negheap.push(LinkNegetaPair(link,-d1/d2));
                    }else{
                        negheap.push(LinkNegetaPair(link,-INF));
                    }
                }
            }
            lamda = negheap.top().eta;
            lamda_link = negheap.top().link;
            negheap.pop();
        }else{
            //is not a tree
            break;
        }
    }
}

std::vector<std::vector<Link*>> EffSol::GetRestPath(int f, const Flow &flow)
{
    std::vector<std::vector<Link*>> res;
    std::stack<LinkTagPair> s;
    std::vector<bool> visited(G2->GetMaxNodeId()+1, false);
    Link fake_link(-1, -1, f, 0, 0, 0, 0);
    s.push(LinkTagPair(&fake_link,0));
    std::vector<Link*> restpath;
    while(!s.empty())
    {
        LinkTagPair p = s.top();
        int u = p.link->ter_id;
        //std::cout<<u<<std::endl;
        s.pop();
        if(p.tag == 1)
        {
            restpath.pop_back();
            visited[u] = false;
        }
        else{
            if(u == flow.to)
            {
                restpath.push_back(p.link);
                std::vector<Link*> restpath_;
                restpath_.reserve(restpath.size()-1);
                for(int i = 1; i < restpath.size(); i++)
                {
                    restpath_.push_back(restpath[i]);
                }
                /*for(int i = 0; i < restpath_.size(); i++)
                {
                    std::cout<<"->"<<restpath_[i]->ter_id;
                }
                std::cout<<std::endl;*/
                res.push_back(restpath_);
                restpath.pop_back();
            }else{
                std::vector<Link*> egresslink = G2->GetEgressLinks(u);
                if(!egresslink.empty())
                {
                    restpath.push_back(p.link);
                    s.push(LinkTagPair(p.link,1));
                    visited[u] = true;
                    for(Link *link : egresslink)
                    {
                        if(visited[link->ter_id]==false)
                        {
                            s.push(LinkTagPair(link,0));
                        }
                    }
                }
            }
        }
    }
    return res;
}

Path EffSol::FindPath(const Flow &flow)
{
    Path result;
    double opt_cost = kMaxValue;
    clock_t start_time = clock();
    InitialEff();
    SplitGraph(flow);
    //std::cout<<"finish split graph"<<std::endl;
    InitialTree(flow);
    //std::cout<<"finish initial the tree"<<std::endl;
    CalPartPath(flow);
    //std::cout<<"finish cal part path"<<std::endl;
    std::unordered_set<int> N_ = Nf;
    while(!N_.empty())
    {
        auto it = N_.begin();
        NodeId f = *it;
        N_.erase(*it);
        std::vector<std::vector<Link*>> rest_paths;
        //std::cout<<"&&"<<std::endl;
        if(N2.empty())
        {
            if(f == flow.to)
            {
                rest_paths = std::vector<std::vector<Link*>> (1,std::vector<Link*>());
            }else{
                rest_paths = std::vector<std::vector<Link*>> (1,std::vector<Link*>(1,new Link(0,0,0,kMaxValue,0,kMaxValue,0)));
            }
        }else{
            rest_paths = GetRestPath(f,flow);
        }
        //std::cout<<"&&"<<std::endl;
        //std::cout<<f<<" "<<rest_paths.size()<<std::endl;
        while(!rest_paths.empty())
        {
            int k = 0;
            std::vector<Link*> rest_path = rest_paths.back();
            rest_paths.pop_back();
            while(eff_part_path[f].find(k) != eff_part_path[f].end())
            {
                ap_info_.iteration_num += 1;
                //std::cout<<k<<" "<<std::endl;
                double sum_delay = CalDelay(eff_part_path[f][k]) + CalDelay(rest_path);
                //std::cout<<"delay"<<sum_delay<<std::endl;
                if(flow.CheckDelayLb(sum_delay) && flow.CheckDelayUb(sum_delay))
                {
                    double sum_cost = CalCost(eff_part_path[f][k]) + CalCost(rest_path);
                    if(sum_cost < opt_cost)
                    {
                        std::vector<Link*> path;
                        path.reserve(eff_part_path[f][k].size() + rest_path.size());
                        for(Link *link : eff_part_path[f][k])
                        {
                            path.push_back(link);
                        }
                        for(Link *link : rest_path)
                        {
                            path.push_back(link);
                        }
                        result.path_link = path;
                        opt_cost = sum_cost;
                    }
                    break;
                }else if(!flow.CheckDelayLb(sum_delay)){
                    if(k > 0)
                    {
                        k = -1;
                    }else{
                        k--;
                    }
                }else if(!flow.CheckDelayUb(sum_delay)){
                    if(k < 0){
                        break;
                    }else{
                        k++;
                    }
                }
            }
        }
    }
    clock_t end_time = clock();
    ap_info_.total_time += (end_time-start_time);
    std::cout << "Heuristic 1985 takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    //std::cout<<"done"<<std::endl;
    if(opt_cost < kMaxDelay)
    {
        result.CompletePath();
    }
    return result;
}