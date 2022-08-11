#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include "src/ap_path.h"
#include "src/bp_path.h"
#include "src/graph.h"
#include "src/ksp_solver.h"
#include "src/shortest_path.h"
#include "src/pulse_solver.h"

void test(std::string topo_path, std::string tunnel_path,int type_id)
{
    // std::cout << 1 << std::endl;
    // std::cout << part_path << std::endl;
    // std::string file_path = "/mnt/d/Users/Documents/GitHub/routing/" + part_path;
    // std::string file_path = topo_path;
    Graph graph(topo_path);
    // std::cout << "\n--------------------------\n";
    // std::cout << topo_path << std::endl;
    // std::cout << "Number of Nodes: " << graph.NodeSize()
            //   << "\nNumber of links: " << graph.LinkSize() << std::endl;
    Demand demand(tunnel_path);
    bool flag = true;
    // std::cout << "Number of Flows: " << demand.NumFlows() << "\n";
    // std::cout << demand.NumFlows() << std::endl;
    for (int i = 0; i < demand.NumFlows() && flag; ++i)
    {
        std::fstream Outfile("cost_ksp", std::ofstream::app);
        Outfile << demand.GetFlow(i).id << ",";
        demand.GetFlow(i).PrintToCsv();
        if (demand.GetFlow(i).is_diff)
        {
            switch(type_id){
                case 1:{
                    Pulse pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    break;
                }
                case 2:{
                    DelayKsp pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    break;
                }
                case 3:{
                    CostKsp pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    break;
                }
                case 4:{
                    CostKspPulse pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    break;
                }
                case 5:{
                    LagrangianKsp pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    break;
                }
                case 6:{
                    CosePulse pulse;
                    pulse.SetupTopology(&graph);
                    PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                    pulse.PrintToCsv();
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
                    flag = false;
                }
            }
        }
        else
        {
            switch(type_id){
                case 1:{
                    Pulse pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 2:{
                    DelayKsp pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 3:{
                    CostKsp pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 4:{
                    CostKspPulse pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 5:{
                    LagrangianKsp pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
                    if (path.cost != demand.GetFlow(i).opt_cost)
                        std::cout << "Error!!!!!" << std::endl;
                    break;
                }
                case 7:{
                    BidirectionalPulse pulse;
                    pulse.SetupTopology(&graph);
                    Path path = pulse.FindPath(demand.GetFlow(i));
                    pulse.PrintToCsv();
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
                                <<"7 for BidirectionalPulse (Pulse+ with joint pruning)."
                                <<std::endl;
                    flag =  false;
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
    clock_t start_time = clock();
    test(argv[1], argv[2],type_id);
    clock_t end_time = clock();
    return 0;
}
