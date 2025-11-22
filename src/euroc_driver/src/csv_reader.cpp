#include "euroc_driver/csv_reader.hpp"
#include <iostream>

namespace euroc_driver {

CSVReader::CSVReader(const std::string& file_path, bool skip_header) 
    : file_path_(file_path) {
    file_.open(file_path);
    if (!file_.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return;
    }
    
    if (skip_header) {
        std::string header;
        std::getline(file_, header);
    }
}

CSVReader::~CSVReader() {
    if (file_.is_open()) {
        file_.close();
    }
}

bool CSVReader::readNextRow(std::vector<std::string>& row) {
    if (!file_.is_open()) {
        return false;
    }
    
    std::string line;
    if (std::getline(file_, line)) {
        parseLine(line, row);
        return true;
    }
    
    return false;
}

bool CSVReader::isOpen() const {
    return file_.is_open();
}

void CSVReader::reset(bool skip_header) {
    if (!file_.is_open()) {
        return;
    }
    
    file_.clear();
    file_.seekg(0, std::ios::beg);
    
    if (skip_header) {
        std::string header;
        std::getline(file_, header);
    }
}

void CSVReader::parseLine(const std::string& line, std::vector<std::string>& tokens) {
    tokens.clear();
    std::stringstream ss(line);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
}

} // namespace euroc_driver
