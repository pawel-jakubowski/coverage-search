#include "MbfoLoopFunction.h"

using namespace argos;
using namespace boost::polygon;


void MbfoLoopFunction::Init(TConfigurationNode& t_tree) {
    voronoi.setArenaLimits(GetSpace().GetArenaLimits());
    coverage.initGrid(GetSpace().GetArenaLimits());
    update();
}

void MbfoLoopFunction::PreStep() {
    update();
}

void MbfoLoopFunction::update() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
    for (auto& position : robotsPositions)
        coverage.getCell(position).isCovered = true;
    voronoi.calculate(robotsPositions);
}

void MbfoLoopFunction::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    robotsPositions.clear();
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        robotsPositions.push_back(position);
    }
}

std::vector<std::vector<CoverageGrid::Cell>> MbfoLoopFunction::getCoverageGrid() {
    return coverage.getGrid();
}

std::vector<argos::CVector3> MbfoLoopFunction::getVoronoiVertices() {
    return voronoi.getVertices();
}

std::vector<CRay3> MbfoLoopFunction::getVoronoiEdges() {
    return voronoi.getEdges();
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
