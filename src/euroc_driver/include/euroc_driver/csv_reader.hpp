#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

namespace euroc_driver {

/**
 * @brief Generic CSV reader utility class
 */
class CSVReader {
public:
    /**
     * @brief Constructor
     * @param file_path Path to the CSV file
     * @param skip_header Whether to skip the first line (header)
     */
    explicit CSVReader(const std::string& file_path, bool skip_header = true);
    
    /**
     * @brief Destructor
     */
    ~CSVReader();
    
    /**
     * @brief Read the next row from the CSV file
     * @param row Output vector to store the parsed row values
     * @return true if a row was successfully read, false if end of file
     */
    bool readNextRow(std::vector<std::string>& row);
    
    /**
     * @brief Check if the file is open and valid
     * @return true if file is open, false otherwise
     */
    bool isOpen() const;
    
    /**
     * @brief Reset file position to beginning
     * @param skip_header Whether to skip the first line after reset
     */
    void reset(bool skip_header = true);

private:
    std::ifstream file_;
    std::string file_path_;
    
    /**
     * @brief Parse a CSV line into tokens
     * @param line Input line to parse
     * @param tokens Output vector to store parsed tokens
     */
    void parseLine(const std::string& line, std::vector<std::string>& tokens);
};

} // namespace euroc_driver
