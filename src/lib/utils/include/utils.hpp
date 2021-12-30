/**
 * @file utils.hpp
 * @author osamy
 * @brief common functionalities and classes
 * for some references see:
 * https://github.com/vss2sn/path_planning/blob/master/lib/utils/include/utils/utils.hpp
 */

#ifndef UTILS_H_
#define UTILS_H_

/**
 * <TODO: remove iostream include and use files logging>
 */
#include <iostream>
#include <stdint.h>
#include <queue>
#include <unordered_set>
#include <vector>
#include <limits>
#include <string>
#include <memory>
#include <fstream>
#include <iomanip>

/* define colors */
#define RESET           "\x1b[0m"
#define BLACK           "\x1b[1;30m"
#define RED             "\x1b[1;31m"
#define GREEN           "\x1b[1;32m"
#define YELLOW          "\x1b[1;33m"
#define BLUE            "\x1b[1;34m"
#define MAGENTA         "\x1b[1;35m"
#define CYAN            "\x1b[1;36m"
#define WHITE           "\x1b[1;37m"
#define BLACK_BOLD      "\x1b[0;30m"
#define RED_BOLD        "\x1b[0;31m"
#define GREEN_BOLD      "\x1b[0;32m"
#define YELLOW_BOLD     "\x1b[0;33m"
#define BLUE_BOLD       "\x1b[0;34m"
#define MAGENTA_BOLD    "\x1b[0;35m"
#define CYAN_BOLD       "\x1b[0;36m"
#define WHITE_BOLD      "\x1b[0;37m"
#define BLACK_HL        "\x1b[1;30;40m"
#define RED_HL          "\x1b[1;30;41m"
#define GREEN_HL        "\x1b[1;30;42m"
#define YELLOW_HL       "\x1b[1;30;43m"
#define BLUE_HL         "\x1b[1;30;44m"
#define MAGENTA_HL      "\x1b[1;30;45m"
#define CYAN_HL         "\x1b[1;30;46m"
#define WHITE_HL        "\x1b[1;30;47m"

/**
 * @brief node class
 * <TODO: move all variables to private scope>
 */
class Node_C
{
public:
    /** \brief x coordinate */
    int64_t x_;
    /** \brief y coordinate */
    int64_t y_;
    /** \brief cost to reach this node */
    double cost_;
    /** \brief heuristic cost to reach the goal */
    double hCost_;
    /** \brief node id */
    int64_t id_;
    /** \brief node's parent's id */
    int64_t pId_;

    /**
     * @brief constructor for node class
     * @param x - x value
     * @param y - y value
     * @param cost - cost to get to node
     * @param hCost - heuristic cost
     * @param id - node id
     * @param pId - parent id
     */
    Node_C(const int64_t x = 0, const int64_t y = 0, const double cost = 0,
           const double hCost = 0, const int64_t id = 0, const int64_t pId = 0) :
           x_(x), y_(y), cost_(cost), hCost_(hCost), id_(id), pId_(pId) {}

    /**
     * @brief prints the values of the variables in the node
     * @return void
     */
    void printStatus() const;

    /**
     * @brief overloading operator + for class
     * @param p - node
     * @return node with current node's and input node p's values added
     */
    Node_C operator+(const Node_C& p) const;

    /**
     * @brief overloading operator - for class
     * @param p - node
     * @return node with current node's and input node p's values subtracted
     */
    Node_C operator-(const Node_C& p) const;

    /**
     * @brief overloading operator == for class
     * @param p - node
     * @return bool whether current node equals input node
     */
    bool operator==(const Node_C& p) const;
};

/**
 * @brief hash for node struct
 */
template<>
class std::hash<Node_C>
{
public:
    /**
     * @brief overload () operator to calculate the hash of a Node_C
     * @param n node for which the hash is to be calculated
     * @return hash value
     */
    size_t operator () (const Node_C& n) const {
        return std::hash<int64_t>()(n.x_) ^ std::hash<int64_t>()(n.y_);
    }
};

/**
 * @brief hash for node that returns node ID
 */
class NodeIdHash_C
{
public:
    /**
     * @brief overload () operator to calculate the hash of a node
     * @param n - node for which the hash to be calculated
     * @return hash value
     * @details the hash returned is the node id
     */
    size_t operator()(const Node_C& n) const {
        return n.id_;
    }
};

/**
 * @brief struct that encapsulates the function that compares cost between two nodes.
 */
struct compare_cost_S
{
    /**
     * @brief compare cost between 2 nodes
     * @param p1 - node 1
     * @param p2 - node 2
     * @return returns whether cost to get to node is greated than cost to get to node 2
     */
    bool operator()(const Node_C& p1, const Node_C& p2) const;
};

/**
 * @brief struct that encapsulates the function that compares cost between two nodes.
 */
struct compare_coord_S
{
    /**
     * @brief compare cost between 2 nodes
     * @param p1 - node 1
     * @param p2 - node 2
     * @return returns whether cost to get to node is greated than cost to get to node 2
     */
    bool operator()(const Node_C& p1, const Node_C& p2) const;
};

/**
 * @brief get permissible motion primatives
 * @return vector of permissible motions
 */
std::vector<Node_C> getPermissibleMotion();

/**
* @brief creates a random grid of a given size
* @param grid - referenct to grid
* @return void
*/
void makeGrid(std::vector<std::vector<int64_t>>& grid);

/**
 * @brief compare coordinates between 2 nodes
 * @param p1 - node 1
 * @param p2 - node 2
 * @return whether the two nodes are for the same coordinates
 */
bool compareCoordinates(const Node_C& p1, const Node_C& p2);

/**
 * @brief checks whether the node is outside the boundary of the grid
 * @param node - node whose coordinates are to be checked
 * @param n - size of the grid
 * @return whether the node is outside the boundary of the grid
 */
bool checkOutsideBoundary(const Node_C& node, const int64_t n);

/**
 * @brief struct to generate a hash for std::pair
 * @details this allows the use of pairs in data structures that use a hash,
 * such as unordered_map/set
 */
struct pair_hash_S {
    /**
     * @brief function used to generate hash for keys
     * @param pair - pair of values
     * @return generated hash value
     */
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

/**
 * @brief struct to hold key values for nodes used in D* Lite
 */
struct key_S {
  double first;
  double second;

    /**
     * @brief overload < operator for comparison
     * @param k - key to be compared
     * @return result of comparison
     */
    bool operator<(const key_S& k) const {
        return first < k.first || (first == k.first && second < k.second);
    }

    /**
     * @brief overload < operator for comparison
     * @param k - key to be compared
     * @return result of comparison
     */
    bool operator>(const key_S& k) const {
        return first > k.first || (first == k.first && second > k.second);
    }

    /**
     * @brief overload == operator for comparison
     * @param k - key to be compared
     * @return result of comparison
     */
    bool operator==(const key_S& k) const {
        return first == k.first && second == k.second;
    }

    /**
     * @brief overload != operator for comparison
     * @param k - key to be compared
     * @return result of comparison
     */
    bool operator!=(const key_S& k) const {
        return !(first == k.first && second == k.second);
    }
};

/**
 * @brief struct to contain the Node Key pairs used by the priority queue in
 * the D* Lite algorithm
 */
struct node_key_pair_S {
    Node_C node;
    key_S key;
};

struct compare_node_key_pair_keys_S {
    bool operator()(const node_key_pair_S& nkp1, const node_key_pair_S& nkp2) const {
        return nkp1.key == nkp2.key;
    }
};

struct compare_node_key_pair_coords_S {
    bool operator()(const node_key_pair_S& nkp1, const node_key_pair_S& nkp2) const {
        return compareCoordinates(nkp1.node, nkp2.node);
    }
};

struct compare_node_key_pair_coords_keys_S {
    bool operator()(const node_key_pair_S& nkp1, const node_key_pair_S& nkp2) const {
        return compareCoordinates(nkp1.node, nkp2.node) && nkp1.key == nkp2.key;
    }
};

template <>
class std::greater<node_key_pair_S> {
 public:
    /**
        * @brief overload () operator for std::greater to use for comparison by
        *        priority queue
        * @param nk1 - node key pair 1
        * @param nk2 - node key pair 2
        * @return result of comparison
        * @details compares the key values
        */
    bool operator()(const node_key_pair_S& nk1, const node_key_pair_S& nk2) const {
        return nk1.key > nk2.key;
    }
};

template <>
class std::hash<node_key_pair_S> {
 public:
    /**
     * @brief hash function for the node key pair
     * @param nkp - node key pair who's jas is to be calculated
     * @return hash value
     */
    size_t operator()(const node_key_pair_S& nkp) const {
        return std::hash<Node_C>()(nkp.node);
    }
};

/**
 * @brief the idea behind this class is to create a structure similar to a
 * priority queue that allows elements to be removed from the middle of the
 * queue as well, rather than just the top
 */
class PrioQ_C
{
public:

    /**
     * @brief clear the q
     * @return void
     */
    void clear();

    /**
     * @brief insert into the q
     * @return void
     */
    void insert(const node_key_pair_S& t);

    /**
     * @brief pop the top element from the q
     * @return void
     */
    void pop();

    /**
     * @brief returns the top element of the q
     * @return reference to the top value in the q
     */
    const node_key_pair_S& top() const;

    /**
     * @brief number of elements in the q
     * @return void
     */
    size_t size() const;

    /**
     * @brief checks whether the q is empty
     * @return bool whether the q is empty
     */
    bool empty() const;

    /**
     * @brief checks whether the element is in the q
     * @return bool whether the element is in the q
     */
    bool isElementInStruct(const node_key_pair_S& t) const;

    /**
     * @brief remove an element from the q if it exists
     * @return void
     */
    void remove(const node_key_pair_S& t);

private:
  std::priority_queue<node_key_pair_S, std::vector<node_key_pair_S>, std::greater<node_key_pair_S>> pq;
  // needs to just compare the coordinates and
  std::unordered_set<node_key_pair_S, std::hash<node_key_pair_S>, compare_node_key_pair_coords_S> s;
};

template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
void printGrid(const std::vector<std::vector<T>>& grid)
{
    int64_t n = grid.size();
    std::cout << "Grid: " << '\n'
              << "-Points not considered ---> 0" << '\n'
              << "-Obstacles             ---> 1" << '\n'
              << "-Points considered     ---> 2" << '\n'
              << "-Points in final path  ---> 3" << '\n'
              << "-Start point           ---> 4" << '\n'
              << "-Goal point            ---> 5" << '\n'
              << "-Invalid point         ---> I" << '\n'
              << "-Size                  ---> " << n << '\n';

    for (int64_t j = 0; j < n; j++)
    {
        std::cout << "---";
    }
    std::cout << '\n';

    for (const auto& row : grid)
    {
        for (const auto& ele : row)
        {
            if (ele == 1)
            {
                std::cout << RED << ele << RESET << " , ";
            }
            else if (ele == 2)
            {
                std::cout << MAGENTA << ele << RESET << " , ";
            }
            else if (ele == 3)
            {
                std::cout << BLUE << ele << RESET << " , ";
            }
            else if (ele == 4)
            {
                std::cout << YELLOW << ele << RESET << " , ";
            }
            else if (ele == 5)
            {
                std::cout << GREEN << ele << RESET << " , ";
            }
            else if (ele == std::numeric_limits<double>::max())
            {
                std::cout << CYAN << "I" << RESET << " , ";
            }
            else
            {
                std::cout << ele << " , ";
            }
        }
        std::cout << '\n' << '\n';
    }

    for (int64_t j = 0; j < n; j++) {
        std::cout << "---";
    }
    std::cout << '\n';
}

/* functions from printer utilities */
/**
 * @brief print the node status
 * @param node - node to be printed
 * @return void
 */
void printNodeStatus(const Node_C& node);

/**
 * @brief print the grid passed, highlighting the path taken
 * @param pathVec - path vector
 * @param start_ - start node
 * @param goal_ - goal node
 * @param grid - grid to work with
 * @return void
 */
void printPath(const std::vector<Node_C>& pathVec,
               const Node_C& start_,
               const Node_C& goal_,
               std::vector<std::vector<int64_t>>& grid);

/**
 * @brief prints the cost for reaching points on the grid in the grid shape
 * @param grid - grid on which algorithm is running
 * @param pointVec - vector of all points that have been considered. nodes in vector contain cost.
 * @return void
 */
void printCost(const std::vector<std::vector<int64_t>>& grid,
               const std::vector<Node_C>& pointVec);

/**
 * @brief prints the grid passed, highlighting the path taken, when the vector
 * is the path taken in order
 * @param pathVec - the path vector
 * @param start - start node
 * @param goal - goal node
 * @param grid - reference to grid
 * @return void
 */
void printPathInOrder(const std::vector<Node_C>& pathVector, const Node_C& start,
                      const Node_C& goal, std::vector<std::vector<int64_t>>& grid);

/* functions from logger utilities */

struct data_logger_S
{
    /** @brief cycle count */
    uint64_t cycleNum;
    /** @brief grid */
    std::vector<std::vector<int64_t>> grid;
    /** @brief path vector */
    std::vector<Node_C> pathVec;
    /** @brief point vector */
    std::vector<Node_C> pointVec;
    /** @brief start node */
    Node_C startNode;
    /** @brief goal node */
    Node_C goalNode;
};

/**
 * @brief function to update vector for logger data
 * @param dataVec - vector to be updated
 * @param idx - index
 * @param grid - grid map
 * @param pathVec - vector of path
 * @param pointVec - vector of point
 * @param startNode - start node
 * @param goalNode - goal node
 */
void updateDataVector(std::vector<data_logger_S>& dataVec,
                      const uint64_t idx,
                      const std::vector<std::vector<int64_t>> grid,
                      const std::vector<Node_C> pathVec,
                      const std::vector<Node_C> pointVec,
                      const Node_C startNode,
                      const Node_C goalNode);

/**
 * @brief function to encapsulate logger class
 * @param dataVec - vector to be written
 * @returns validity flag
 */
bool generateLogs(const std::vector<data_logger_S>& dataVec);

/**
 * @brief overload function to encapsulate logger class
 * @param dataVec - vector to be written
 * @param outExtension - output file extension
 * @returns validity flag
 */
bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension);

/**
 * @brief overload function to encapsulate logger class
 * @param dataVec - vector to be written
 * @param outExtension - output file extension
 * @param outName - output file name
 * @returns validity flag
 */
bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension,
                  const std::string& outName);

/**
 * @brief function to encapsulate logger class
 * @param dataVec - vector to be written
 * @param outExtension - output file extension
 * @param outName - output file name
 * @param outPath - output path
 * @returns validity flag
 */
bool generateLogs(const std::vector<data_logger_S>& dataVec,
                  const std::string& outExtension,
                  const std::string& outName,
                  const std::string& outPath);

/**
 * @brief logger class
 */
class Logger_C
{

public:

    enum extension_E
    {
        EXTENSION_NON = 0,
        EXTENSION_TXT,
        EXTENSION_CSV,
        EXTENSION_NUM
    };

    /**
     * @brief default constructor for logger class
     * @details in case of this constructor is called:
     *      extension is set to default (.txt),
     *      file name is generated automatically, and
     *      path is set to default path (<project_home>/gen/logger/)
     */
    Logger_C() : Logger_C("txt",
                          "outFileTxt",
                          "../gen/logger/") {}

    /**
     * @brief constructor for logger class
     * @details in case of this constructor is called:
     *      extension is explicitly provided,
     *      file name is generated automatically, and
     *      path is set to default path (<project_home>/gen/logger/)
     * @param extension - path in which file shall be generated
     */
    Logger_C(std::string extension) : Logger_C(extension,
                                               "outFile",
                                               "../gen/logger/") {}

    /**
     * @brief constructor for logger class
     * @details in case of this constructor is called:
     *      extension is explicitly provided,
     *      file name is explicitly provided, and
     *      path is set to default path (<project_home>/gen/logger/)
     * @param extension - path in which file shall be generated
     * @param name - file name required
     */
    Logger_C(std::string extension, std::string name) : Logger_C(extension, name, "../gen/logger/") {}

    /**
     * @brief constructor for logger class
     * @details in case of this constructor is called:
     *      extension is explicitly provided,
     *      file name is explicitly provided, and
     *      path is explicitly provided
     * @param extension - file extension
     * @param name - file name required
     * @param path - path in which file shall be generated
     */
    Logger_C(const std::string& extension,
             const std::string& name,
             const std::string& path);

    /**
     * @brief default destructor for logger class,
     * @details checks if the file is open and closes it
     */
    ~Logger_C();

    /**
     * @brief sets the data vector
     * @param dataVec - data vector to be set
     */
    void setDataVec(const std::vector<data_logger_S>& dataVec) { dataVec_ = dataVec; }

    /**
     * @brief writes out the data to file
     * @returns validity flag
     */
    bool writeDataToFile();

private:

    /** \brief final file object */
    std::unique_ptr<std::ofstream> fileToWrite_;

    /** \brief final vector of data */
    std::vector<data_logger_S> dataVec_;

    /** \brief file name */
    std::string fileName_;

    /**
     * @brief sets the file name member
     */
    void setFileName(std::string fileName) { fileName_ = fileName; }

    /**
     * @brief extracts the extension of the file
     * @returns extension
     */
    extension_E extractExtension() const;

    /**
     * @brief writes data to text file
     * @returns validity flag
     */
    bool writeDataToTxt();

    /**
     * @brief writes data to comma seperated value file
     * @returns validity flag
     */
    bool writeDataToCsv();
};

#endif /* UTILS_H_ */