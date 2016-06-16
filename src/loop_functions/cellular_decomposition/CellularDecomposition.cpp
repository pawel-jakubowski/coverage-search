#include "CellularDecomposition.h"
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

using namespace argos;

static const Real FOOTBOT_BODY_RADIUS = 0.085036758f;

CellularDecomposition::CellularDecomposition()
    : coverage(maxCellConcentration, 0.1f)
{}

void CellularDecomposition::Init(TConfigurationNode& t_tree) {
    CRange<CVector2> limits;

    limits.SetMin(
        CVector2(
            GetSpace().GetArenaLimits().GetMin().GetX() + FOOTBOT_BODY_RADIUS,
            GetSpace().GetArenaLimits().GetMin().GetY() + FOOTBOT_BODY_RADIUS
        )
    );

    limits.SetMax(
        CVector2(
            GetSpace().GetArenaLimits().GetMax().GetX() - FOOTBOT_BODY_RADIUS,
            GetSpace().GetArenaLimits().GetMax().GetY() - FOOTBOT_BODY_RADIUS
        )
    );

    taskManager.addNewCell(limits);
    Reset();
}

void CellularDecomposition::PreStep() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
    taskManager.assignTasks();
}

void CellularDecomposition::PostStep() {
    std::vector<CoverageGrid::CellIndex> affectedCells = getCellsCoveredByRobots();
    removeDuplicates(affectedCells);
    try {
        updateCoverageCells(affectedCells);
    }
    catch(std::exception& e) {
        THROW_ARGOSEXCEPTION_NESTED("Error during concentration update!", e)
    }
}

std::vector<CoverageGrid::CellIndex> CellularDecomposition::getCellsCoveredByRobots() const {
    std::vector<CoverageGrid::CellIndex> affectedCells;
    auto delta = coverage.getCellSize() / 2;
    for (auto ray : rays) {
        auto rayLength = ray.GetLength();
        while (rayLength > 0) {
            try {
                affectedCells.emplace_back(coverage.getCellIndex(ray.GetEnd()));
            }
            catch(std::exception& e) {
                std::stringstream s;
                s << "Error during concentration update for ray end (" << ray.GetEnd() << ")";
                THROW_ARGOSEXCEPTION_NESTED(s.str(), e);
            }
            rayLength = ray.GetLength() - delta;
            ray.SetLength(rayLength);
        }
    }
    return affectedCells;
}

void CellularDecomposition::removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const {
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
}

void CellularDecomposition::updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells) {
    for (auto &cell : affectedCells) {
        unsigned x = cell.first;
        unsigned y = cell.second;
        coverage.getGrid()[x][y].concentration /= 2;
    }
}

void CellularDecomposition::Reset() {
    coverage.initGrid(GetSpace().GetArenaLimits());
    PreStep();
}

void CellularDecomposition::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    rays.clear();
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        addRobotsRays(footbot);
        robotsPositions[footbot.GetId()] = position;
    }
}

void CellularDecomposition::addRobotsRays(CFootBotEntity& footbot) {
    auto& proximityEntity = footbot.GetProximitySensorEquippedEntity();
    for(UInt32 i = 0; i < proximityEntity.GetNumSensors(); ++i) {
        CVector3 cRayStart, cRayEnd;
        cRayStart = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        cRayEnd = proximityEntity.GetSensor(i).Offset;
        cRayEnd += proximityEntity.GetSensor(i).Direction;
        cRayEnd.Rotate(proximityEntity.GetSensor(i).Anchor.Orientation);
        cRayEnd += proximityEntity.GetSensor(i).Anchor.Position;
        wrapPointToArenaLimits(cRayEnd);
        rays.emplace_back(cRayStart, cRayEnd);
    }
}

void CellularDecomposition::wrapPointToArenaLimits(CVector3 &point) {
    auto limits = GetSpace().GetArenaLimits();
    if(point.GetX() > limits.GetMax().GetX())
        point.SetX(limits.GetMax().GetX());
    else if (point.GetX() < limits.GetMin().GetX())
        point.SetX(limits.GetMin().GetX());
    if(point.GetY() > limits.GetMax().GetY())
        point.SetY(limits.GetMax().GetY());
    else if (point.GetY() < limits.GetMin().GetY())
        point.SetY(limits.GetMin().GetY());
}

const CoverageGrid& CellularDecomposition::getCoverageGrid() {
    return coverage;
}

const std::map<std::string, CVector3> CellularDecomposition::getRobotsPositions() {
    return robotsPositions;
}

const std::vector<argos::CRay3> CellularDecomposition::getRays() {
    return rays;
}

REGISTER_LOOP_FUNCTIONS(CellularDecomposition, "cellular_loop_fcn")
