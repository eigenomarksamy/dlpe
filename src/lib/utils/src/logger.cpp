/**
 * @file logger.cpp
 * @author osamy
 * @brief this is a file used for logging data into txt, csv, etc.
 */

/* project-specific includes */
#include "utils.hpp"

/* defined MACROs */
#define DEFAULT_STR "default"

Logger_C::Logger_C(const std::string& extension,
                   const std::string& name,
                   const std::string& path)
{
    std::string fileName;

    if ('.' == extension.at(0))
    {
        fileName = path + name + extension;
    }
    else
    {
        fileName = path + name + '.' + extension;
    }

    setFileName(fileName);

    std::unique_ptr<std::ofstream> fileToWrite_(new std::ofstream);
}

Logger_C::~Logger_C()
{
    if (fileToWrite_->is_open())
        fileToWrite_->close();
}

bool Logger_C::writeDataToFile()
{

    bool retVal;

    const extension_E extension = extractExtension();

    switch (extension)
    {
    case EXTENSION_TXT:
        retVal = writeDataToTxt();
        break;
    case EXTENSION_CSV:
        retVal = writeDataToCsv();
        break;

    default:
        retVal = false;
        break;
    }

    return retVal;
}

Logger_C::extension_E Logger_C::extractExtension() const
{
    extension_E extExt = EXTENSION_NON;

    if ("txt" == fileName_.substr(fileName_.find_last_of(".") + 1))
    {
        extExt = EXTENSION_TXT;
    }
    else if ("csv" == fileName_.substr(fileName_.find_last_of(".") + 1))
    {
        extExt = EXTENSION_CSV;
    }

    return extExt;
}

bool Logger_C::writeDataToTxt()
{
    bool valid = true;

    return valid;
}

bool Logger_C::writeDataToCsv()
{
    bool valid = true;

    return valid;
}

void updateDataVector(std::vector<data_logger_S>& dataVec,
                      const uint64_t idx,
                      const std::vector<std::vector<int64_t>> grid,
                      const std::vector<Node_C> pathVec,
                      const std::vector<Node_C> pointVec,
                      const Node_C startNode,
                      const Node_C goalNode)
{
    data_logger_S tmpData {idx, grid, pathVec, pointVec, startNode, goalNode};
    dataVec.push_back(tmpData);
}

bool generateLogs(const std::vector<data_logger_S>& dataVec)
{
    return generateLogs(dataVec, DEFAULT_STR, DEFAULT_STR, DEFAULT_STR);
}

bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outPath)
{
    return generateLogs(dataVec, outPath, DEFAULT_STR, DEFAULT_STR);
}

bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outPath,
                  const std::string& outName)
{
    return generateLogs(dataVec, outPath, outName, DEFAULT_STR);
}

bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension,
                  const std::string& outName,
                  const std::string& outPath)
{

    if (DEFAULT_STR == outExtension)
    {
        Logger_C logObj;
        logObj.setDataVec(dataVec);
        logObj.writeDataToFile();
    }
    else if (DEFAULT_STR == outName)
    {
        Logger_C logObj(outPath);
        logObj.setDataVec(dataVec);
        logObj.writeDataToFile();
    }
    else if (DEFAULT_STR == outPath)
    {
        Logger_C logObj(outPath, outName);
        logObj.setDataVec(dataVec);
        logObj.writeDataToFile();
    }
    else
    {
        Logger_C logObj(outPath, outName, outExtension);
        logObj.setDataVec(dataVec);
        logObj.writeDataToFile();
    }

    return true;
}