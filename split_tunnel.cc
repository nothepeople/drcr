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

int main(int argc, char *argv[])
{
    std::vector<std::vector<std::string>> data;
    data = read_in(argv[1]);
    // std::string folderpath = "D:\\Users\\Documents\\GitHub/routing/tunnels";
    // // std::string command;
    // command = "mkdir -p " + folderpath;
    // system(command.c_str());
    std::string name = argv[1];
    std::string tunnel_name;
    for (int i = 0; i < name.size(); ++i)
    {
        if (name[i] != '.')
        {
            tunnel_name += name[i];
        }
        else
            break;
    }

    int p = 0;
    std::string title;
    for (p = 0; p < data[0].size() - 1; ++p)
    {
        title.append(data[0][p]);
        title.append(",");
    }
    title.append(data[0][p]);
    for (int i = 1; i < data.size(); ++i)
    {
        int j = 0;
        std::string filename = "./tunnels/";
        filename.append(tunnel_name);
        filename.append("_");
        filename.append(data[i][0]);
        std::fstream Outfile(filename, std::ofstream::app);
        Outfile << title << std::endl;
        for (j = 0; j < data[i].size() - 1; ++j)
        {
            Outfile << data[i][j] << ",";
        }
        Outfile << data[i][j];
    }
}
