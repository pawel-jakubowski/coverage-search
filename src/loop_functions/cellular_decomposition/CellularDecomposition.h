#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <robots/custom-foot-bot/simulator/footbot_entity.h>
#include <utils/coverage/CoverageGrid.h>
#include <utils/task/TaskManager.h>

class CellularDecomposition : public argos::CLoopFunctions {
public:
    static constexpr int maxCellConcentration = std::numeric_limits<int>::max();

    CellularDecomposition();
    virtual ~CellularDecomposition() = default;
    virtual void Init(argos::TConfigurationNode& t_tree) override;
    virtual void PreStep() override;
    virtual void PostStep() override;
    virtual void Reset() override;

    void registerToTaskManager(TaskHandler& handler) { taskManager.registerHandler(handler); }
    void unregisterFromTaskManager(TaskHandler& handler) { taskManager.unregisterHandler(handler); }
    const auto getTaskCells() const { return taskManager.getCells(); }

    const CoverageGrid& getCoverageGrid();
    const std::map<std::string, argos::CVector3> getRobotsPositions();
    const std::vector<argos::CRay3> getRays();

private:
    TaskManager taskManager;
    CoverageGrid coverage;
    std::map<std::string, argos::CVector3> robotsPositions;
    std::vector<argos::CRay3> rays;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void addRobotsRays(argos::CCustomFootBotEntity& footbot);
    void wrapPointToArenaLimits(argos::CVector3 &point);
    std::vector<CoverageGrid::CellIndex> getCellsCoveredByRobots() const;
    void removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const;
    void updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells);
};


