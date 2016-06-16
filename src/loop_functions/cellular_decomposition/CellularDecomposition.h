#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utils/coverage/CoverageGrid.h>
#include <utils/task/Task.h>

class CellularDecomposition : public argos::CLoopFunctions {
public:
    static constexpr int maxCellConcentration = std::numeric_limits<int>::max();

    CellularDecomposition() : coverage(maxCellConcentration, 0.1f) {}
    virtual ~CellularDecomposition() = default;
    virtual void Init(argos::TConfigurationNode& t_tree) override;
    virtual void PreStep() override;
    virtual void PostStep() override;
    virtual void Reset() override;

    Task getNewTask(Task old);
    const CoverageGrid& getCoverageGrid();
    const std::map<std::string, argos::CVector3> getRobotsPositions();
    const std::vector<argos::CRay3> getRays();

private:
    CoverageGrid coverage;
    std::map<std::string, argos::CVector3> robotsPositions;
    std::vector<argos::CRay3> rays;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void addRobotsRays(argos::CFootBotEntity& footbot);
    void wrapPointToArenaLimits(argos::CVector3 &point);
    std::vector<CoverageGrid::CellIndex> getCellsCoveredByRobots() const;
    void removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const;
    void updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells);
};


