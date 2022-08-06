#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>

std::vector<std::vector<std::string>> read_in(std::string path)
{
    std::vector<std::vector<std::string>> data;
    std::ifstream inFile(path, std::ios::in);
    std::string lineStr;
    while (getline(inFile, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<std::string> lineArray;
        // 按照逗号分隔
        while (getline(ss, str, ','))
        {
            lineArray.push_back(str);
        }
        data.push_back(lineArray);
    }
    return data;
}

void check_res(std::vector<std::vector<std::string>> &data)
{
    double ap_time = 0;
    double optimality = 0;
    double bp_time = 0;
    double total_time = 0;
    for (int i = 0; i < data.size(); ++i)
    {
        ap_time += atof(data[i][1].c_str());
        bp_time += atof(data[i][2].c_str());
        total_time += atof(data[i][3].c_str());
        double tmp = atof(data[i][4].c_str()) > 0 ? atof(data[i][4].c_str()) : 0;
        optimality += tmp;
    }
    ap_time /= double(data.size());
    bp_time /= double(data.size());
    total_time /= double(data.size());
    optimality /= double(data.size());
    std::fstream Outfile("Result_Analysis", std::ofstream::app);
    Outfile << "Average AP Time:    " << ap_time << std::endl;
    Outfile << "Average BP Time:    " << bp_time << std::endl;
    Outfile << "Average Total Time: " << total_time << std::endl;
    Outfile << "Average Optimality  " << optimality << std::endl;
}

int main(int argc, char *argv[])
{
    std::vector<std::vector<std::string>> data;
    std::fstream Outfile("Result_Analysis", std::ofstream::app);
    for (int i = 1; i < argc; ++i)
    {
        data = read_in(argv[i]);
        Outfile << "-------------------------------" << std::endl;
        Outfile << argv[i] << std::endl;
        Outfile << "-------------------------------" << std::endl;
        check_res(data);
    }
    return 0;
}