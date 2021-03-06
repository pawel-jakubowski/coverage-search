#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utils/voronoi/VoronoiDiagram.h>
#include <utils/coverage/CoverageGrid.h>
#include <iostream>
#include <mutex>


class MbfoLoopFunction : public argos::CLoopFunctions {
public:
    static constexpr int maxCellConcentration = std::numeric_limits<int>::max();

    MbfoLoopFunction() : coverage(maxCellConcentration, 0.1f) {}
    virtual ~MbfoLoopFunction() = default;
    virtual void Init(argos::TConfigurationNode& t_tree) override;
    virtual bool IsExperimentFinished() override;
    virtual void PreStep() override;
    virtual void PostStep() override;
    virtual void Reset() override;
    virtual void Destroy() override;

    void update();
    void addTargetPosition(int id, const argos::CVector3& position);
    const CoverageGrid& getCoverageGrid();
    const std::vector<VoronoiDiagram::Cell>& getVoronoiCells();
    const VoronoiDiagram::Cell* getVoronoiCell(std::string id);
    const std::vector<const VoronoiDiagram::Cell*> getNeighbouringVoronoiCells(std::string id);
    const std::map<std::string, argos::CVector3> getRobotsPositions();
    const std::vector<argos::CRay3> getRays();

private:
    struct Target {
        using Id = int;
        argos::UInt32 step;
        argos::CVector3 position;
    };
    struct MbfoLog {
        std::string name;
        std::ofstream file;
        std::map<Target::Id, Target> targets;
        std::map<argos::UInt32, double> thresholds;
    };

    std::mutex tagetPositionUpdateMutex;

    MbfoLog log;
    std::list<double> thresholdsToLog;
    unsigned targetsNumber;
    CoverageGrid coverage;
    VoronoiDiagram voronoi;
    bool voronoiAssertion = false;
    std::map<std::string, argos::CVector3> robotsPositions;
    std::map<std::string, const VoronoiDiagram::Cell*> robotsCells;
    std::vector<argos::CRay3> rays;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void addRobotsRays(argos::CFootBotEntity& footbot);
    void wrapPointToArenaLimits(argos::CVector3 &point);
    std::vector<CoverageGrid::CellIndex> getCellsCoveredByRobots() const;
    void removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const;
    void updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells);

    void parseLogConfig(argos::TConfigurationNode& t_tree);
    void parseVoronoiConfig(argos::TConfigurationNode& t_tree);

    void checkPercentageCoverage();

    void saveLog() ;
};
