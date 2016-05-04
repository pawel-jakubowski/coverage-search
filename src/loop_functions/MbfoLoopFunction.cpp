#include "MbfoLoopFunction.h"

using namespace argos;
using namespace boost::polygon;


void MbfoLoopFunction::Init(TConfigurationNode& t_tree) {
    voronoi.setArenaLimits(GetSpace().GetArenaLimits());
    coverage.initGrid(GetSpace().GetArenaLimits());
    PreStep();
    update();
}

void MbfoLoopFunction::PreStep() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
}

void MbfoLoopFunction::PostStep() {
    for (auto& position : robotsPositions)
        coverage.getCell(position).concentration /= 2;
}

void MbfoLoopFunction::update() {
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

const std::vector<std::vector<CoverageGrid::Cell>>& MbfoLoopFunction::getCoverageGrid() {
    return coverage.getGrid();
}

const std::vector<VoronoiDiagram::Cell>& MbfoLoopFunction::getVoronoiCells() {
    return voronoi.getCells();
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
