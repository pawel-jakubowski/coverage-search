#include "MbfoLoopFunction.h"
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>


using namespace std;
using namespace argos;
using namespace boost::polygon;

void MbfoLoopFunction::Init(TConfigurationNode& t_tree) {
    parseLogConfig(t_tree);
    parseVoronoiConfig(t_tree);
    Reset();
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
    log.name = "mbfo.log";
    try {
        TConfigurationNode& conf = GetNode(t_tree, "log");
        GetNodeAttribute(conf, "path", log.name);
        LOG << "Log file: " << log.name << endl;
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
}

void MbfoLoopFunction::PreStep() {
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    assert(entities.size() >= 3);
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
    checkPercentageCoverage();
}

void MbfoLoopFunction::checkPercentageCoverage() {
    if (thresholdsToLog.size() > 0) {
        const auto percentageCoverage = coverage.getCoverageValue();
        if (percentageCoverage > thresholdsToLog.front()) {
            LOG << "Threshold " << thresholdsToLog.front() << "% achieved!" << endl;
            log.thresholds[GetSpace().GetSimulationClock()] = percentageCoverage;
            thresholdsToLog.pop_front();
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
    saveLog();
}

void MbfoLoopFunction::saveLog() {
    log.file.open(log.name);
    log.file << "{\n";
    log.file << "\"thresholds\" : [\n";
    for (auto& threshold : log.thresholds)
        log.file << "\t" "{ "
        << "\"step\" : " << threshold.first << ", "
        << "\"coverage\" : "
        << fixed << setprecision(2) << threshold.second
        << " },\n";
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
    log.file.seekp(-2, ios_base::end);
    log.file << "\n]\n";

    log.file << "}";
    log.file.close();
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
            << " cells unassigned to voronoi cells!\n";
        for (size_t i = 0; i < coverage.getGrid().size(); i++)
            for (size_t j = 0; j < coverage.getGrid().size(); j++) {
                s << "[" << i << "," << j << "] "
                    << "(" << coverage.getGrid().at(i).at(j).center << ") ";
                for (auto& voronoiCell : voronoi.getCells()) {
                    auto it = find_if(voronoiCell.coverageCells.begin(), voronoiCell.coverageCells.end(), [i, j]
                        (const VoronoiCell::CoverageCell& a) { return a.x == i && a.y == j; });
                    if (it != voronoiCell.coverageCells.end())
                        s << voronoiCell.seed.id;
                }
                s << "\n";
            }
        THROW_ARGOSEXCEPTION(s.str());
    }
}

void MbfoLoopFunction::addTargetPosition(int id, const CVector3& position) {
    std::lock_guard<std::mutex> guard(tagetPositionUpdateMutex);
    if (log.targets.find(id) == log.targets.end()) {
        log.targets[id] = {GetSpace().GetSimulationClock(), position};
        LOG << "Target was found!" << endl;
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
        if (cell.first != id) {
            assert(cell.second != nullptr);
            robotsNeighbours.emplace_back(cell.second);
        }
    return robotsNeighbours;
}

const std::map<std::string, CVector3> MbfoLoopFunction::getRobotsPositions() {
    return robotsPositions;
}

const std::vector<argos::CRay3> MbfoLoopFunction::getRays() {
    return rays;
}

REGISTER_LOOP_FUNCTIONS(MbfoLoopFunction, "mbfo_loop_fcn")
