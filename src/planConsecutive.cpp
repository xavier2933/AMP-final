/* Author: Xavier O'Keefe */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <cmath>
#include <ctime> 
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;


struct SphereObstacle {
    double x, y, z, radius;
};

struct StationTask {
    public:
        int taskComplete = 0;
        int serviced = 0;
        int index;
        Eigen::VectorXd goal = Eigen::VectorXd(3);
};

std::vector<SphereObstacle> obstacles = {
    {0.0, 0.0, 0.0, 0.25}, // x, y, z, radius
    {0.5, 0.5, 0.5, 0.15},
    {1.0, 1.0, 0.0, 5.2},
    {5.0, 5.0, 6.0, 3.2},
    {-5.0, -6.0, -4.0, 3.2}
};

// GPT sphere collision checker
bool isStateValid(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    for (const auto &obstacle : obstacles) {
        double dist = std::sqrt(std::pow(x - obstacle.x, 2) +
                                std::pow(y - obstacle.y, 2) +
                                std::pow(z - obstacle.z, 2));
        if (dist <= obstacle.radius) {
            return false; // State is in collision
            std::cout << "invalid state found ==========" << std::endl;
        }
    }

    return true; // State is valid
}


double planWithSimpleSetupSeq(const std::string &output_file, Eigen::VectorXd startVec, Eigen::VectorXd goalVec)
{

    std::ofstream out_file(output_file, std::ios::app);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return 0.0;
    }

    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });



    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);

    for(int i = 0; i < 7; i++)
    {
        start[i] = startVec[i];
        goal[i] = goalVec[i];
    }

    ss.setStartAndGoalStates(start, goal);
    ss.setup();

    ob::PlannerStatus solved = ss.solve(1.0);
    double len = 0.0;
    if (solved)
    {
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        const og::PathGeometric &path = ss.getSolutionPath();
        len = path.length();

        // Get state count and iterate through each state
        size_t num_states = path.getStateCount();
        for (size_t i = 0; i < num_states; ++i)
        {
            const ob::State *state = path.getState(i);
            const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // Extract position
            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];

            // Write position
            out_file << std::fixed << std::setprecision(5) << x << " " << y << " " << z << std::endl;

        }
    }
    else
    {
        out_file << "No solution found" << std::endl;
    }
    out_file.close();
    return len;

}

int getNearest(std::vector<StationTask> stations, Eigen::VectorXd curr)
{
    double minDist = std::numeric_limits<double>::max();
    int index = -1;

    Eigen::VectorXd currNew(3);
    if (curr.size() >= 3) {
        currNew << curr[0], curr[1], curr[2];
    } else {
        std::cerr << "Error: curr must have at least 3 elements!" << std::endl;
        return index;
    }

    for (int i = 0; i < stations.size(); i++)
    {
        if (stations[i].goal.size() != currNew.size()) {
            std::cerr << "Skipping station " << i 
                      << " due to size mismatch: goal size is " 
                      << stations[i].goal.size() << std::endl;
            continue;
        }

        if ((currNew - stations[i].goal).norm() < minDist &&
            stations[i].serviced != 1 && 
            !currNew.isApprox(stations[i].goal))
        {
            index = i;
            minDist = (currNew - stations[i].goal).norm();
        }
    }
    return index;
}

double planSeqTask()
{
    double pathLength = 0;
    std::string output_file = "SampleOut.txt";
    if (std::remove(output_file.c_str()) == 0) {
        std::cout << "Deleted existing file: " << output_file << std::endl;
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::time_t now = std::time(nullptr);

    std::vector<StationTask> stations;
    std::vector<StationTask> serviced;

    Eigen::VectorXd startVec(7);
    Eigen::VectorXd goalVec(7);

    StationTask s1;
    s1.goal << 9.0, 9.0, 9.0;
    s1.index = 0;
    stations.push_back(s1);

    StationTask s2;
    s2.goal << -9.0, -9.0, -9.0;
    s2.index = 1;
    stations.push_back(s2);

    StationTask s3;

    s3.goal << -9.0, 9.0, -9.0;
    s3.index = 2;
    stations.push_back(s3);

    std::string visited = "init ";
    Eigen::VectorXd init(7);
    Eigen::VectorXd curr(7);
    Eigen::VectorXd finalGoal(7);

    init << 3.0, -5.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z
    finalGoal << 0, -9.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z
    goalVec << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z

    curr = init;

    std::cout << "Closest is " << getNearest(stations, curr) << std::endl;

    // Randomly select a station to start
    int firstIndex = now % stations.size();

    while(serviced.size() != stations.size())
    {
        std::time_t loopTime = std::time(nullptr);
        firstIndex = getNearest(stations, curr);
        if(firstIndex == -1)
        {
            std::cout << "going to goal" << std::endl;
            break;
        }
        if(stations[firstIndex].serviced == 1)
        {
            continue;
        }
        std::cout << "Planning station " << firstIndex << std::endl;
        goalVec[0] = stations[firstIndex].goal[0];
        goalVec[1] = stations[firstIndex].goal[1];
        goalVec[2] = stations[firstIndex].goal[2];

        pathLength += planWithSimpleSetupSeq(output_file, curr, goalVec);
        visited = visited + " -> " + std::to_string(firstIndex);
        curr[0] = stations[firstIndex].goal[0];
        curr[1] = stations[firstIndex].goal[1];
        curr[2] = stations[firstIndex].goal[2];
        if(loopTime % 3 != 1)
        {
            serviced.push_back(stations[firstIndex]);
            stations[firstIndex].serviced = 1;
            std::cout << "SERVICED " << firstIndex << std::endl;
        } else {
            std::cout << "DID NOT service " << firstIndex << std::endl;
            std::cout << "Current location " << curr.transpose() << std::endl;
        }
    }
    pathLength += planWithSimpleSetupSeq(output_file, curr, finalGoal);

    std::cout << std::endl << std::endl;
    return pathLength;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::vector<double> lens;
    double len = 0.0;
    double localLen = 0.0;
    double avg = 0.0;
    len = planSeqTask();

    //////////////////////////////////////////////////////////////////////////
    /*
    Uncomment below for benchmarking
    */

    // for(int j = 0; j < 10; j++)
    // {
    //     len = 0;
    //     for(int i = 0; i < 100; i++)
    //     {
    //         localLen = planSeqTask();
    //         len+=localLen;
    //         lens.push_back(localLen);
    //     }
    //     avg += len/100.0;
    // }

    // std::ofstream outFile("lensSeq.csv");
    // if (outFile.is_open()) {
    //     outFile << "Index,Value\n"; // Optional: Header row for CSV
    //     for (size_t i = 0; i < lens.size(); i++) {
    //         outFile << i << "," << lens[i] << "\n";
    //     }
    //     outFile.close();
    //     std::cout << "Data written to lens.csv successfully." << std::endl;
    // } else {
    //     std::cerr << "Failed to open file for writing." << std::endl;
    // }

    //////////////////////////////////////////////////////////////////////////


    return 0;
}