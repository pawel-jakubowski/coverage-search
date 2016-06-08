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

void ClosestDistance::PostStep() {
    auto& entities = GetSpace().GetEntitiesByType("e-puck");

    Real closest = 10000;
    for (const auto& entity : entities) {
        auto& epuck = *(any_cast<CEPuckEntity*>(entity.second));
        CVector2 epuckPosition;
        epuck.GetEmbodiedEntity().GetOriginAnchor().Position.ProjectOntoXY(epuckPosition);
        Real distance = (epuckPosition - position).Length();
        if (distance < closest)
            closest = distance;
    }

    if (bestObtainedDistance > closest) {
        log.closestDistances[GetSpace().GetSimulationClock()] = closest;
        bestObtainedDistance = closest;
    }
}

void ClosestDistance::Destroy() {
    PostStep();
    saveLog();
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
    log.file.seekp(-2, ios_base::end);
    log.file << "\n]\n";

    log.file << "}";
    log.file.close();
}

REGISTER_LOOP_FUNCTIONS(ClosestDistance, "closest_distance")

