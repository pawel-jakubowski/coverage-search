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
    voronoi.calculate(robotsPositions, coverage.getGrid());
    for (auto& id : robotsIds)
        cellsPerId[id] = nullptr;

    int gridCounter = 0;
    auto idCellPair = cellsPerId.begin();
    for (auto& voronoiCell : voronoi.getCells()) {
        // Assume that voronoi cells are in the same order as robots
        assert(idCellPair != cellsPerId.end());
        idCellPair->second = &voronoiCell;
        idCellPair++;

        gridCounter += voronoiCell.coverageCells.size();
    }
    assert(idCellPair == cellsPerId.end());
    auto gridCellsCount = coverage.getGrid().size();
    gridCellsCount *= gridCellsCount;
    assert(gridCounter == gridCellsCount);
}

void MbfoLoopFunction::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    robotsPositions.clear();
    robotsIds.clear();
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        robotsPositions.push_back(position);
        robotsIds.push_back(footbot.GetId());
    }
}

const std::vector<std::vector<CoverageGrid::Cell>>& MbfoLoopFunction::getCoverageGrid() {
    return coverage.getGrid();
}

const std::vector<VoronoiDiagram::Cell>& MbfoLoopFunction::getVoronoiCells() {
    return voronoi.getCells();
}

const VoronoiDiagram::Cell* MbfoLoopFunction::getVoronoiCell(std::string id) {
    return cellsPerId.at(id);
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
