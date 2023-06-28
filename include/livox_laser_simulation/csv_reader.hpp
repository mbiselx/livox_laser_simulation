//
// Created by lfc on 2021/3/1.
//

#ifndef SRC_GAZEBO_CSV_READER_HPP
#define SRC_GAZEBO_CSV_READER_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvReader {
 public:
    static bool ReadCsvFile(std::string file_name, std::vector<std::vector<double>>& datas) {
        std::fstream file_stream;
        std::string header, line_str, value;
        std::vector<double> data;

        // open file
        file_stream.open(file_name, std::ios::in);
        if (!file_stream.is_open()){
            std::cerr << "cannot read csv file!" << file_name << "\n";
            return false;
        }
    
        // discard header
        std::getline(file_stream, header, '\n');

        // get values by line
        while (std::getline(file_stream, line_str, '\n')) {
            std::stringstream line_stream(line_str);
            data.clear();
            try {
                while (std::getline(line_stream, value, ',')) 
                    data.push_back(std::stod(value));
                
            } catch (...) {
                std::cerr << "cannot convert str:" << line_str << "\n";
                continue;
            }
            datas.push_back(data);
        }

        std::cout << "data size:" << datas.size() << "\n";
        return true;

    }
};

#endif  // SRC_GAZEBO_CSV_READER_HPP
