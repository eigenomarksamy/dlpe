/**
 * @file logger.cpp
 * @author osamy
 * @brief this is a file used for logging data into txt, csv, etc.
 */

/* project-specific includes */
#include "utils.hpp"

/* defined MACROs */

/* local functions */
/**
 * @brief log the node status
 * @param p_fileToWrite - shared pointer to file
 * @param node - node to be printed
 * @return void
 */
static void logNodeStatus(std::shared_ptr<std::ostream> p_fileToWrite,
                          const Node_C& node);

/**
 * @brief log the grid passed
 * @param p_fileToWrite - shared pointer to file
 * @param pathVec - path vector
 * @param start_ - start node
 * @param goal_ - goal node
 * @param grid - grid to work with
 * @return void
 */
static void logPath(std::shared_ptr<std::ostream> p_fileToWrite,
                    const std::vector<Node_C>& pathVec,
                    const Node_C& start,
                    const Node_C& goal,
                    std::vector<std::vector<int64_t>>& grid);

/**
 * @brief logs the cost for reaching points on the grid in the grid shape
 * @param p_fileToWrite - shared pointer to file
 * @param grid - grid on which algorithm is running
 * @param pointVec - vector of all points that have been considered. nodes in vector contain cost.
 * @return void
 */
static void logCost(std::shared_ptr<std::ostream> p_fileToWrite,
                    const std::vector<std::vector<int64_t>>& grid,
                    const std::vector<Node_C>& pointVec);

/**
 * @brief logs the grid passed, when the vector
 * is the path taken in order
 * @param p_fileToWrite - shared pointer to file
 * @param pathVec - the path vector
 * @param start - start node
 * @param goal - goal node
 * @param grid - reference to grid
 * @return void
 */
static void logPathInOrder(std::shared_ptr<std::ostream> p_fileToWrite,
                           const std::vector<Node_C>& pathVec, const Node_C& start,
                           const Node_C& goal, std::vector<std::vector<int64_t>>& grid);


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

    std::string newFileName = fileName_;

    valid = handleDirectory(newFileName, true);

    if (valid)
    {
        const std::string booleanArr[] = {"false", "true"};

        setFileName(newFileName);

        p_fileToWrite_ = std::make_shared<std::ofstream>(fileName_);

        *p_fileToWrite_ << "This is the first line of a generated file.\n";
        *p_fileToWrite_ << "Options: "
                        << "[ Cycle: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_CYCLE]]
                            + " | Grid: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_GRID]]
                            + " | Path: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_PATH]]
                            + " | Point: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_POINT]]
                            + " | Start: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_START]]
                            + " | Goal: " + booleanArr[a_bitMapEnableInVec_[DATA_LOGGER_GOAL]]
                            + " ]\n";

        /* get each data element, iff it's set to true in bitmap */
        for (auto& elm : dataVec_)
        {
            if (a_bitMapEnableInVec_[DATA_LOGGER_CYCLE])
            {
                *p_fileToWrite_ << "\nCycle: " << elm.cycleNum << "\n";
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_GRID])
            {
                *p_fileToWrite_ << "\nInitial Grid:\n";
                logGrid(p_fileToWrite_, elm.grid);
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_PATH]
                && a_bitMapEnableInVec_[DATA_LOGGER_START]
                && a_bitMapEnableInVec_[DATA_LOGGER_GOAL]
                && a_bitMapEnableInVec_[DATA_LOGGER_GRID])
            {
                *p_fileToWrite_ << "\nPath:\n";
                logPath(p_fileToWrite_, elm.pathVec, elm.startNode, elm.goalNode, elm.grid);
            }
            if (a_bitMapEnableInVec_[DATA_LOGGER_POINT]
                && a_bitMapEnableInVec_[DATA_LOGGER_GRID])
            {
                *p_fileToWrite_ << "\nPoint:\n";
                logCost(p_fileToWrite_, elm.grid, elm.pointVec);
            }
            *p_fileToWrite_ << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << std::setw(spacing_for_grid) << "----"
                            << "\n";
        }
    }

    return valid;
}

bool Logger_C::writeDataToCsv()
{
    bool valid = true;

    return valid;
}

static void logNodeStatus(std::shared_ptr<std::ostream> p_fileToWrite,
                          const Node_C& node)
{
    *p_fileToWrite << "-----------------" << '\n'
                   << "Node_C          : " << '\n'
                   << "x             : " << node.x_ << '\n'
                   << "y             : " << node.y_ << '\n'
                   << "Cost          : " << node.cost_ << '\n'
                   << "Heuristic cost: " << node.hCost_ << '\n'
                   << "Id            : " << node.id_ << '\n'
                   << "Parent id     : " << node.pId_ << '\n'
                   << "----------------" << '\n';
}

static void logPath(std::shared_ptr<std::ostream> p_fileToWrite,
                    const std::vector<Node_C>& pathVec,
                    const Node_C& start,
                    const Node_C& goal,
                    std::vector<std::vector<int64_t>>& grid)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    if (pathVec.empty())
    {
        *p_fileToWrite << "No path exists" << '\n';
        logGrid(p_fileToWrite, grid);
        return;
    }
    *p_fileToWrite << "Path (goal to start):" << '\n';
    for (size_t i = 0; i < pathVec.size(); i++)
    {
        if (compareCoordinates(goal, pathVec[i]))
        {
            logNodeStatus(p_fileToWrite, pathVec[i]);
            grid[pathVec[i].x_][pathVec[i].y_] = 3;
            while (pathVec[i].id_ != start.id_)
            {
                if (pathVec[i].id_ == pathVec[i].pId_)
                {
                    break;
                }
                for (size_t j = 0; j < pathVec.size(); j++)
                {
                    if (pathVec[i].pId_ == pathVec[j].id_)
                    {
                        i = j;
                        logNodeStatus(p_fileToWrite, pathVec[j]);
                        grid[pathVec[j].x_][pathVec[j].y_] = 3;
                    }
                }
            }
            break;
        }
    }
    grid[goal.x_][goal.y_] = 5;
    grid[start.x_][start.y_] = 4;
    logGrid(p_fileToWrite, grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

static void logCost(std::shared_ptr<std::ostream> p_fileToWrite,
                    const std::vector<std::vector<int64_t>>& grid,
                    const std::vector<Node_C>& pointVec)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    int64_t n = grid.size();
    std::vector<Node_C>::const_iterator it_v;
    for (int64_t i = 0; i < n; i++)
    {
        for (int64_t j = 0; j < n; j++)
        {
            for (it_v = pointVec.begin(); it_v != pointVec.end(); ++it_v)
            {
                if (i == it_v->x_ && j == it_v->y_)
                {
                    *p_fileToWrite << std::setw(spacing_for_grid) << it_v->cost_ << " , ";
                    break;
                }
            }
            if (it_v == pointVec.end())
            {
                *p_fileToWrite << std::setw(spacing_for_grid) << "  , ";
            }
        }
        *p_fileToWrite << '\n' << '\n';
    }
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

static void logPathInOrder(std::shared_ptr<std::ostream> p_fileToWrite,
                           const std::vector<Node_C>& pathVec, const Node_C& start,
                           const Node_C& goal, std::vector<std::vector<int64_t>>& grid)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    if (pathVec.empty())
    {
        *p_fileToWrite << "Path not found" << '\n';
        printGrid(grid);
        return;
    }
    *p_fileToWrite << "Path (goal to start):" << '\n';
    size_t i = 0;
    while (!compareCoordinates(pathVec[i], goal))
    {
        i++;
    }
    for (; i > 0; i = i - 1)
    {
        logNodeStatus(p_fileToWrite, pathVec[i]);
        grid[pathVec[i].x_][pathVec[i].y_] = 3;
    }
    logNodeStatus(p_fileToWrite, pathVec[0]);
    grid[pathVec[0].x_][pathVec[0].y_] = 3;
    logGrid(p_fileToWrite, grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

bool handleDirectory(std::string& fileName, const bool forceDir)
{
    bool exists;
    struct stat buf;

    char result[PATH_MAX];
    ssize_t count = readlink(LINUX_DIR_PROC, result, PATH_MAX);
    std::string path;
    if (count != -1) {
        path = static_cast<std::string>(dirname(result));
    }

    const std::string tmpFileDir = path + "/" + fileName.substr(0, fileName.find_last_of('/') + 1);

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
                fileName = path + "/" + fileName;
            }
        }
    }
    else
    {
        fileName = path + "/" + fileName;
    }

    return exists;
}