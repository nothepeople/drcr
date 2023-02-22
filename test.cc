#include <iostream>
#include <fstream>
#include <unordered_set>

#include "src/ap_path.h"
#include "src/bp_path.h"
#include "src/graph.h"
#include "src/ksp_solver.h"
#include "src/shortest_path.h"
#include "src/pulse_solver.h"

void TestShortestPathSolver(std::string file_path) {
    Graph graph(file_path);
    std::cout << "\n--------------------------\n";
    std::cout << file_path << std::endl;
    std::cout << "Number of Nodes: " << graph.NodeSize()
              << "\nNumber of links: " << graph.LinkSize() << std::endl;
    std::vector<Link>& all_links = graph.GetMutableLinks();
    NodeId dst = 0;
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    Dijkstra dijkstra(&graph);
    dijkstra.InitWithDst(dst);
    std::vector<double> cost1(graph.NodeSize());
    for (NodeId src = 1; src < graph.NodeSize(); ++src) {
        all_links[src].status = Conflict;
        std::vector<Link *> links;
        cost1[src] = dijkstra.FindPathFromSrc(src, &links);
    }
    end_time = clock();
    std::cout << "Dijsktra takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    for (Link& link : all_links) {
        link.status = Available;
    }
    start_time = clock();
    AStar a_star(&graph);
    a_star.InitWithDst(dst);
    std::vector<double> cost2(graph.NodeSize());
    for (NodeId src = 1; src < graph.NodeSize(); ++src) {
        all_links[src].status = Conflict;
        std::vector<Link *> links;
        cost2[src] = a_star.FindPathFromSrc(src, &links);
    }
    end_time = clock();
    std::cout << "AStar takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    for (NodeId src = 1; src < graph.NodeSize(); ++src) {
        assert(cost1[src] == cost2[src]);
    }
    std::cout << "AStar and Dijkstra attain the same cost.\n";
}

void TestApPathSolver() {
    std::string file_path = "data/DelayDiff/large_case_39/topo.csv";
    Graph graph(file_path);
    std::cout << "\n--------------------------\n";
    std::cout << file_path << std::endl;
    std::cout << "Number of Nodes: " << graph.NodeSize()
              << "\nNumber of links: " << graph.LinkSize() << std::endl;
    Flow flow;
    flow.type = LinkDisjoint;
    flow.from = 1786;
    flow.to = 1474;
    flow.delay_lb = 363;
    flow.delay_ub = 370;
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    AStar a_star1(&graph, LinkDelay);
    a_star1.InitWithDst(flow.to);
    AStar a_star2(&graph, LinkCost);
    a_star2.InitWithDst(flow.to);
    GenericAp ap;
    DisjointBp bp(&graph);
    ap.Init(&graph, &bp, a_star2.GetCostVector(),
            a_star1.GetCostVector());
    LogInfo ap_info, bp_info;
    double min_cost = ap.FindOptPath(flow, kMaxValue, ap_info, bp_info);
    end_time = clock();
    std::cout << "AP search takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    if (min_cost < kMaxValue) {
        std::cout << "min cost is " << min_cost << "\nAP path: ";
        std::vector<Link*> result = ap.GetApPath();
        for (Link* link : result) {
            std::cout << link->link_id << " ";
        }
        std::cout << "\nBP path: ";
        std::vector<Link*> result2 = ap.GetBpPath();
        for (Link* link : result2) {
            std::cout << link->link_id << " ";
        }
        std::cout << "\n";
    } else {
        std::cout << "No feasible solution.\n";
    }
}

void TestBpPathSolver() {
    std::string file_path = "data/DelayDiff/large_case_39/topo.csv";
    Graph graph(file_path);
    std::cout << "\n--------------------------\n";
    std::cout << file_path << std::endl;
    std::cout << "Number of Nodes: " << graph.NodeSize()
              << "\nNumber of links: " << graph.LinkSize() << std::endl;

    // Initialize ap_path
    std::vector<Link*> ap_path;
    std::vector<int> ap_link_ids = {
        3660, 3658, 3656, 3654, 3652, 3650, 3648, 3647, 4501,
        4508, 4457, 4622, 4620, 4294, 4645, 4649, 4652, 4656,
        4662, 4664, 4590, 4633, 4189, 4608, 4603, 4600, 4598,
        4267, 4643, 4484, 4492, 3014, 3017};
    for (int link_id : ap_link_ids) {
        for (Link& link : graph.GetMutableLinks()) {
            if (link.link_id == link_id) {
                ap_path.push_back(&link);
                break;
            }
        }
    }
    Flow flow;
    flow.type = LinkDisjoint;
    flow.from = 1786;
    flow.to = 1474;
    flow.delay_lb = 363;
    flow.delay_ub = 370;
    // Test Bp path solver
    clock_t start_time;
    clock_t end_time;
    start_time = clock();
    SrlgDisjointBp bp_path_solver(&graph);
    int iteration_num = 0;
    double delay = bp_path_solver.FindBpPath(ap_path, flow, iteration_num);
    end_time = clock();
    std::cout << "BP search takes: "
              << double(end_time - start_time) / CLOCKS_PER_SEC * 1000
              << "(ms).\n";
    if (delay < kMaxValue) {
        std::vector<Link*> result = bp_path_solver.GetBpPath();
        for (Link* link : result) {
            std::cout << link->link_id << " ";
        }
        std::cout << "\n";
    } else {
        ConflictSet conflit_set = bp_path_solver.GetConflictSet();
        std::cout << "No feasible solution.\n";
        conflit_set.Print();
    }
}

void Test(std::string file_path) {
    Graph graph(file_path + "topo.csv");
    std::cout << "\n--------------------------\n";
    std::cout << file_path << std::endl;
    std::cout << "Number of Nodes: " << graph.NodeSize()
              << "\nNumber of links: " << graph.LinkSize() << std::endl;

    Demand demand(file_path + "tunnel.csv");
    std::cout << "Number of Flows: " << demand.NumFlows() << "\n";
    Algorithm* algorithm;
    if (demand.GetFlow(0).type == 2) {
        // algorithm = new SrlgDisjointPulse();
        algorithm = new CosePulse();
    } else {
        algorithm = new BidirectionalPulse();
        // algorithm = new Pulse();
    }
    algorithm->SetupTopology(&graph);
    for (int i = 0; i < demand.NumFlows(); ++i) {
        Flow flow = demand.GetFlow(i);
        demand.GetFlow(i).Print();
        if (flow.is_diff) {
            if (flow.type == 0) {
                std::cout << "**Link Separate**\n";
                PathPair path = algorithm->FindPathPair(flow);
                path.Print();
                if (path.ap_path.cost != flow.opt_cost) {
                    std::cout << "Error!!!!!" << std::endl;
                    return;
                }
            }
            if (flow.type == 1) {
                continue;
                std::cout << "**Node Separate**\n";
                PathPair path = algorithm->FindPathPair(flow);
                path.Print();
                if (path.ap_path.cost != flow.opt_cost) {
                    std::cout << "Error!!!!!" << std::endl;
                    return;
                }
            }
            if (flow.type == 2) {
                std::cout << "**Srlg Separate**\n";
                PathPair path = algorithm->FindPathPair(flow);
                path.Print();
                if (path.ap_path.cost != flow.opt_cost) {
                    std::cout << "Error!!!!!" << std::endl;
                    return;
                }
            }
        } else {
            std::cout << "**Delay Range**\n";
            Path path = algorithm->FindPath(flow);
            path.Print();
            if (path.cost != flow.opt_cost) {
                std::cout << "Error!!!!!" << std::endl;
                return;
            }
        }
    }
}

int main() {
    // TestShortestPathSolver("data/DelayDiff/dc5/");
    // TestShortestPathSolver("data/DelayDiff/large/case_39/");
    // TestApPathSolver();
    // TestBpPathSolver();
    // Test("data/DelayDiff/large_case_39/");
    // Test("data/DelayDiff/large_case_31/");
    // Test("data/DelayDiff/case2/");  // srlg disjoint
    Test("data/DelayRange/large_case_30/");
    return 0;
}
