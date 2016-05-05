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
        coverage.getCell(position.second).concentration /= 2;
}

void MbfoLoopFunction::update() {
    voronoi.calculate(robotsPositions, coverage.getGrid());

    int gridCounter = 0;
    for (auto& voronoiCell : voronoi.getCells()) {
        robotsCells[voronoiCell.seedId] = &voronoiCell;
        gridCounter += voronoiCell.coverageCells.size();
    }
    auto gridCellsCount = coverage.getGrid().size();
    gridCellsCount *= gridCellsCount;
    assert(gridCounter == gridCellsCount);
}

void MbfoLoopFunction::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        robotsPositions[footbot.GetId()] = position;
    }
}

const CoverageGrid& MbfoLoopFunction::getCoverageGrid() {
    return coverage;
}

const std::vector<VoronoiDiagram::Cell>& MbfoLoopFunction::getVoronoiCells() {
    return voronoi.getCells();
}

const VoronoiDiagram::Cell* MbfoLoopFunction::getVoronoiCell(std::string id) {
    return robotsCells.at(id);
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
