#include "VoronoiCalculator.h"

using namespace argos;
using namespace boost::polygon;


void VoronoiCalculator::Init(TConfigurationNode& t_tree) {
    voronoi.setArenaLimits(GetSpace().GetArenaLimits());
    update();
}

void VoronoiCalculator::update() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
    voronoi.calculate(robotsPositions);
}

void VoronoiCalculator::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    robotsPositions.clear();
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        robotsPositions[footbot.GetId()] = position;
    }
}

std::vector<argos::CVector3> VoronoiCalculator::getVertices() {
    return voronoi.getVertices();
}

std::vector<CRay3> VoronoiCalculator::getEdges() {
    return voronoi.getEdges();
}

REGISTER_LOOP_FUNCTIONS(VoronoiCalculator, "calculate_voronoi")
