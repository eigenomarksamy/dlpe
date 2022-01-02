/**
 * @file logger.cpp
 * @author osamy
 * @brief this is a file used for logging data into txt, csv, etc.
 */

/* project-specific includes */
#include "utils.hpp"

/* defined MACROs */

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

bool generateLogs(const uint8_t logBitMap,
                  const std::vector<data_logger_S>& dataVec)
{
    bool valid;
    Logger_C logObj;
    generateLogs(logBitMap, dataVec, logObj);
    return true;
}

bool generateLogs(const uint8_t logBitMap,
                  const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension)
{
    bool valid;
    Logger_C logObj(outExtension);
    generateLogs(logBitMap, dataVec, logObj);
    return valid;
}

bool generateLogs(const uint8_t logBitMap,
                  const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension,
                  const std::string& outName)
{
    bool valid;
    Logger_C logObj(outExtension, outName);
    generateLogs(logBitMap, dataVec, logObj);
    return valid;
}

bool generateLogs(const uint8_t logBitMap,
                  const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension,
                  const std::string& outName,
                  const std::string& outPath)
{
    bool valid;
    Logger_C logObj(outPath, outName, outExtension);
    generateLogs(logBitMap, dataVec, logObj);
    return valid;
}

bool generateLogs(const uint8_t logBitMap,
                  const std::vector<data_logger_S>& dataVec,
                  Logger_C& logObj)
{
    bool valid;
    logObj.setDataVec(dataVec);
    logObj.setLogBitMap(logBitMap);
    valid = logObj.writeDataToFile();
    return valid;
}

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

void Logger_C::setLogBitMap(const uint8_t bitMap)
{
    for (uint8_t i = DATA_LOGGER_CYCLE; i < DATA_LOGGER_LEN; i++)
    {
        a_bitMapEnableInVec_[i] = ((bitMap >> (i)) & 1);
    }
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
    bool valid;

    valid = handleDirectory(true);

    if (valid)
    {
        p_fileToWrite_ = std::make_shared<std::ofstream>(fileName_);

        *p_fileToWrite_ << "This is the first line of a generated file.\n";

        /* get each data element, iff it's set to true in bitmap */
        for (const auto& elm : dataVec_)
        {
            if (a_bitMapEnableInVec_[DATA_LOGGER_CYCLE])
            {
                *p_fileToWrite_ << "Cycle: " << elm.cycleNum << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_GRID])
            {
                *p_fileToWrite_ << "Grid: " << "Not Implemented!" << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_PATH])
            {
                *p_fileToWrite_ << "Path: " << "Not Implemented!" << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_POINT])
            {
                *p_fileToWrite_ << "Point: " << "Not Implemented!" << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_START])
            {
                *p_fileToWrite_ << "Start: " << "Not Implemented!" << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_GOAL])
            {
                *p_fileToWrite_ << "Goal: " << "Not Implemented!" << "\n";
            }
        }
    }

    return valid;
}

bool Logger_C::handleDirectory(const bool forceDir) const
{
    bool exists;
    struct stat buf;

    const std::string tmpFileDir = fileName_.substr(0, fileName_.find_last_of('/') + 1);

    exists  = (stat (tmpFileDir.c_str(), &buf) == 0);

    if (!exists)
    {
        if(forceDir)
        {
            std::error_code ec;
            if (!std::filesystem::create_directories(tmpFileDir, ec))
            {
                exists = false;
            }
            else
            {
                exists = true;
            }
        }
    }
    return exists;
}

bool Logger_C::writeDataToCsv()
{
    bool valid = true;

    return valid;
}

void Logger_C::logNodeStatus(const Node_C& node)
{

}

void Logger_C::logPath(const std::vector<Node_C>& pathVec,
                       const Node_C& start_,
                       const Node_C& goal_,
                       std::vector<std::vector<int64_t>>& grid)
{

}

void Logger_C::logCost(const std::vector<std::vector<int64_t>>& grid,
                       const std::vector<Node_C>& pointVec)
{

}

void Logger_C::logPathInOrder(const std::vector<Node_C>& pathVector, const Node_C& start,
                              const Node_C& goal, std::vector<std::vector<int64_t>>& grid)
{

}