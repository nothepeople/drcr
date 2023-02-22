#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include "src/ap_path.h"
#include "src/bp_path.h"
#include "src/heuristic.h"
#include "src/graph.h"
#include "src/ksp_solver.h"
#include "src/shortest_path.h"
#include "src/pulse_solver.h"

void test(std::string topo_path, std::string tunnel_path, int type_id, int flow_id = -1)
{
    Graph graph(topo_path);
    // std::cout << "\n--------------------------\n";
    // std::cout << topo_path << std::endl;
    // std::cout << "Number of Nodes: " << graph.NodeSize()
            //   << "\nNumber of links: " << graph.LinkSize() << std::endl;
    Demand demand(tunnel_path);
    // std::cout << "Number of Flows: " << demand.NumFlows() << "\n";
    // std::cout << demand.NumFlows() << std::endl;
    std::vector<int> flow_ids;
    if (flow_id == -1) {
        flow_ids.reserve(demand.NumFlows());
        for (int i = 0; i < demand.NumFlows(); ++i) {
            flow_ids.push_back(i);
        }
    } else {
        flow_ids.push_back(flow_id);
    }

    for (int i : flow_ids)
    {
        if (demand.GetFlow(i).is_diff)
        {
            switch(type_id){
                case 1:{
                    Pulse solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                case 2:{
                    DelayKsp solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                case 3:{
                    CostKsp solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                case 4:{
                    CostKspPulse solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                case 5:{
                    LagrangianKsp solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                case 6:{
                    CosePulse solver;
                    solver.SetupTopology(&graph);
                    PathPair path = solver.FindPathPair(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    break;
                }
                default:{
                    std::cout<<"The type id entered is meaningless, as for SrlgDisjoint DRCR, "
                                <<"you can choose 1 for Pulse+, "
                                <<"2 for DelayKsp, "
                                <<"3 for CostKsp, "
                                <<"4 for CostKspPulse, "
                                <<"5 for LagrangianKsp, "
                                <<"6 for CosePulse+."
                                <<std::endl;
                    return;
                }
            }
        }
        else
        {
            switch(type_id){
                case 1:{
                    Pulse solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 2:{
                    DelayKsp solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 3:{
                    CostKsp solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 4:{
                    CostKspPulse solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 5:{
                    LagrangianKsp solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 6:{
                    BidirectionalPulse solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 7:{
                    EffSol solver;
                    solver.SetupTopology(&graph);
                    Path path = solver.FindPath(demand.GetFlow(i));
                    path.Print();
                    demand.GetFlow(i).PrintToCsv();
                    solver.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                default:{
                    std::cout<<"\nThe type id entered is meaningless, as for DRCR, "
                                <<"you can choose 1 for Pulse+, "
                                <<"2 for DelayKsp, "
                                <<"3 for CostKsp, "
                                <<"4 for CostKspPulse, "
                                <<"5 for LagrangianKsp, "
                                <<"6 for BidirectionalPulse (Pulse+ with joint pruning), "
                                <<"7 for Heuristic Solution Developed in 1985."
                                <<std::endl;
                    return;
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    double optimality = 0;
    int cnt = 0;
    // std::cout << "hello" << std::endl;
    int type_id = atoi(argv[3]);
    int flow_id = atoi(argv[4]);
    clock_t start_time = clock();
    test(argv[1], argv[2],type_id, flow_id);
    clock_t end_time = clock();
    return 0;
}
