#include "MbfoLoopFunction.h"
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <mutex>

using namespace std;
using namespace argos;
using namespace boost::polygon;

std::mutex tagetPositionUpdateMutex;

void MbfoLoopFunction::Init(TConfigurationNode& t_tree) {
    parseLogConfig(t_tree);
    parseVoronoiConfig(t_tree);
    Reset();
    logFile << "{\n";
}

void MbfoLoopFunction::parseVoronoiConfig(TConfigurationNode& t_tree) {
    try {
        TConfigurationNode& conf = GetNode(t_tree, "voronoi");
        GetNodeAttributeOrDefault(conf, "assertion", voronoiAssertion, false);
    }
    catch (CARGoSException& e) {
        LOGERR << "Error parsing voronoi config! " <<  e.what();
    }
}

void MbfoLoopFunction::parseLogConfig(TConfigurationNode& t_tree) {
    string logFileName = "mbfo.log";
    try {
        TConfigurationNode& conf = GetNode(t_tree, "log");
        GetNodeAttribute(conf, "path", logFileName);
        LOG << "Log file: " << logFileName << endl;
        TConfigurationNodeIterator it("threshold");
        it = it.begin(&conf);
        double threshold = 0;
        while (it != NULL) {
            GetNodeAttribute(*it, "value", threshold);
            LOG << "Threshold: " << threshold << endl;
            thresholdsToLog.push_back(threshold);
            it++;
        }
    }
    catch (CARGoSException& e) {
        LOGERR << "Error parsing log config! " <<  e.what();
    }
    logFile.open(logFileName);
}

void MbfoLoopFunction::PreStep() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
}

void MbfoLoopFunction::PostStep() {
    std::vector<CoverageGrid::CellIndex> affectedCells = getCellsCoveredByRobots();
    removeDuplicates(affectedCells);
    try {
        updateCoverageCells(affectedCells);
    }
    catch(std::exception& e) {
        THROW_ARGOSEXCEPTION_NESTED("Error during concentration update!", e)
    }

    if (thresholdsToLog.size() > 0) {
        const auto percentageCoverage = coverage.getCoverageValue();
        if (percentageCoverage > thresholdsToLog.front()) {
            logFile << "\t" "\"threshold\" : {\n"
                    << "\t\t" "\"step\" : " << GetSpace().GetSimulationClock() << ",\n"
                    << "\t\t" "\"coverage\" : "
                    << std::fixed << std::setprecision(2) << percentageCoverage << ",\n"
                    << "\t\t" "\"target\" : [" << targetsPosition << "]\n"
                    << "\t}";
            thresholdsToLog.pop_front();
            if (thresholdsToLog.size() > 0)
                logFile << ",";
            logFile << "\n";
        }
    }
}

std::vector<CoverageGrid::CellIndex> MbfoLoopFunction::getCellsCoveredByRobots() const {
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

void MbfoLoopFunction::removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const {
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
}

void MbfoLoopFunction::updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells) {
    for (auto &cell : affectedCells) {
        unsigned x = cell.first;
        unsigned y = cell.second;
        coverage.getGrid()[x][y].concentration /= 2;
    }
}

void MbfoLoopFunction::Reset() {
    voronoi.setArenaLimits(GetSpace().GetArenaLimits());
    coverage.initGrid(GetSpace().GetArenaLimits());
    PreStep();
    update();
}

void MbfoLoopFunction::Destroy() {
    logFile << "}";
    logFile.close();
}

void MbfoLoopFunction::update() {
    voronoi.calculate(robotsPositions, coverage.getGrid());

    int gridCounter = 0;
    for (auto& voronoiCell : voronoi.getCells()) {
        robotsCells[voronoiCell.seed.id] = &voronoiCell;
        gridCounter += voronoiCell.coverageCells.size();
    }
    auto gridCellsCount = coverage.getGrid().size();
    gridCellsCount *= gridCellsCount;
    if (voronoiAssertion && gridCounter != gridCellsCount) {
        std::stringstream s;
        s << "There is " << gridCellsCount - gridCounter
            << " cells unassigned to voronoi cells!";
        THROW_ARGOSEXCEPTION(s.str());
    }
}

void MbfoLoopFunction::addSingleTargetPosition(double certainty, const argos::CVector3& position) {
    if (certainty > targetsPositionCertainty) {
        std::lock_guard<std::mutex> guard(tagetPositionUpdateMutex);
        targetsPosition = position;
        targetsPositionCertainty = certainty;
        LOG << "New target position estimation: " << targetsPosition << "\n";
    }
}

void MbfoLoopFunction::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    rays.clear();
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
        addRobotsRays(footbot);
        robotsPositions[footbot.GetId()] = position;
    }
}

void MbfoLoopFunction::addRobotsRays(CFootBotEntity& footbot) {
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

void MbfoLoopFunction::wrapPointToArenaLimits(CVector3 &point) {
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

const CoverageGrid& MbfoLoopFunction::getCoverageGrid() {
    return coverage;
}

const std::vector<VoronoiDiagram::Cell>& MbfoLoopFunction::getVoronoiCells() {
    return voronoi.getCells();
}

const VoronoiDiagram::Cell* MbfoLoopFunction::getVoronoiCell(std::string id) {
    return robotsCells.at(id);
}

const std::vector<const VoronoiDiagram::Cell*> MbfoLoopFunction::getNeighbouringVoronoiCells(std::string id) {
    std::vector<const VoronoiDiagram::Cell*> robotsNeighbours;
    for (auto& cell : robotsCells)
        if (cell.first != id)
            robotsNeighbours.emplace_back(cell.second);
    return robotsNeighbours;
}

const std::map<std::string, CVector3> MbfoLoopFunction::getRobotsPositions() {
    return robotsPositions;
}

const std::vector<argos::CRay3> MbfoLoopFunction::getRays() {
    return rays;
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
