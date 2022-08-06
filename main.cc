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

void test(std::string topo_path, std::string tunnel_path)
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
    // std::cout << "Number of Flows: " << demand.NumFlows() << "\n";
    // std::cout << demand.NumFlows() << std::endl;
    for (int i = 0; i < demand.NumFlows(); ++i)
    {
        std::fstream Outfile("cost_ksp", std::ofstream::app);
        Outfile << demand.GetFlow(i).id << ",";
        demand.GetFlow(i).PrintToCsv();
        if (demand.GetFlow(i).is_diff)
        {
            if (demand.GetFlow(i).type == 0)
            {
                // std::cout << "**Link Separate**\n";
                CosePulse pulse;
                pulse.SetupTopology(&graph);
                PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                pulse.PrintToCsv();
            }
            if (demand.GetFlow(i).type == 1)
            {
                // std::cout << "**Node Separate**\n";
                CosePulse pulse;
                pulse.SetupTopology(&graph);
                PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                pulse.PrintToCsv();
            }
            if (demand.GetFlow(i).type == 2)
            {
                // std::cout << "**Srlg Separate**\n";
                CosePulse pulse;
                pulse.SetupTopology(&graph);
                PathPair path = pulse.FindPathPair(demand.GetFlow(i));
                pulse.PrintToCsv();
            }
        }
        else
        {
            // std::cout << "**Delay Range**\n";
            LagrangianKsp pulse;
            pulse.SetupTopology(&graph);
            Path path = pulse.FindPath(demand.GetFlow(i));
            pulse.PrintToCsv();
            if (path.cost != demand.GetFlow(i).opt_cost)
                std::cout << "Error!!!!!" << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    double optimality = 0;
    int cnt = 0;
    // std::cout << "hello" << std::endl;
    clock_t start_time = clock();
    test(argv[1], argv[2]);
    clock_t end_time = clock();
    return 0;
}
