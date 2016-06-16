#include "ClosestDistance.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <algorithm>
#include <iomanip>

using namespace std;
using namespace argos;

void ClosestDistance::Init(TConfigurationNode& t_tree) {
    parseLogConfig(t_tree);
    parseTargetConfig(t_tree);
}

void ClosestDistance::parseLogConfig(TConfigurationNode& t_tree) {
    log.name = "pso.log";
    try {
        TConfigurationNode& conf = GetNode(t_tree, "log");
        GetNodeAttribute(conf, "path", log.name);
        LOG << "Log file: " << log.name << endl;
    }
    catch (CARGoSException& e) {
        LOGERR << "Error parsing log config! " <<  e.what() << endl;
    }
}

void ClosestDistance::parseTargetConfig(TConfigurationNode& t_tree) {
    try {
        double x,y;
        TConfigurationNode& conf = GetNode(t_tree, "target");
        GetNodeAttribute(conf, "position_x", x);
        GetNodeAttribute(conf, "position_y", y);
        position.Set(x, y);
    }
    catch (CARGoSException& e) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", e);
    }
}

void ClosestDistance::Destroy() {
    saveLog();
}

void ClosestDistance::addRobotPosition(CVector3 pos) {
    CVector2 robotsPosition;
    pos.ProjectOntoXY(robotsPosition);
    Real distance = (robotsPosition - position).Length();

    std::lock_guard<std::mutex> guard(robotDistanceUpdateMutex);
    if (distance < bestObtainedDistance) {
        bestObtainedDistance = distance;
        log.closestDistances[GetSpace().GetSimulationClock()] = bestObtainedDistance;
    }
}

void ClosestDistance::addTargetPosition(int id, const CVector3& position) {
    std::lock_guard<std::mutex> guard(tagetPositionUpdateMutex);
    if (log.targets.find(id) == log.targets.end()) {
        log.targets[id] = { GetSpace().GetSimulationClock(), position };
        LOG << "Target was found!" << endl;
    }
}

void ClosestDistance::saveLog() {
    log.file.open(log.name);
    log.file << "{\n";

    log.file << "\"closest\" : [\n";
    for (auto& distance : log.closestDistances)
        log.file << "\t" "{ "
        << "\"step\" : " << distance.first << ", "
        << "\"distance\" : " << distance.second
        << " },\n";
    if (log.closestDistances.size() != 0)
        log.file.seekp(-2, ios_base::end);
    log.file << "\n],\n";

    log.file << "\"targets\" : [\n";
    for (auto& target : log.targets) {
        log.file << "\t" "{ "
        << "\"id\" : " << target.first << ", "
        << "\"step\" : " << target.second.step << ", "
        << "\"position\" : [" << target.second.position << "]"
        << " },\n";
    }
    if (log.targets.size() != 0)
        log.file.seekp(-2, ios_base::end);
    log.file << "\n]\n";

    log.file << "}";
    log.file.close();
}

REGISTER_LOOP_FUNCTIONS(ClosestDistance, "closest_distance")

