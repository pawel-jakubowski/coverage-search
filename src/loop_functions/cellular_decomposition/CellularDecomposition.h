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
    virtual void Destroy() override;
    virtual void PreStep() override;
    virtual void PostStep() override;
    virtual void Reset() override;
    virtual bool IsExperimentFinished() override;

    void addTargetPosition(int id, const argos::CVector3& position);
    std::shared_ptr<TaskManager> getManager() { return taskManager; }
    const auto getTaskCells() const { return taskManager->getCells(); }

    const CoverageGrid& getCoverageGrid();
    const std::map<std::string, argos::CVector3> getRobotsPositions();
    const std::vector<argos::CRay3> getRays();

private:
    struct Target {
        using Id = int;
        argos::UInt32 step;
        argos::CVector3 position;
    };

    struct CellularLog {
        std::string name;
        std::ofstream file;
        std::map<Target::Id, Target> targets;
    };

    std::shared_ptr<TaskManager> taskManager;
    CoverageGrid coverage;
    std::map<std::string, argos::CVector3> robotsPositions;
    std::vector<argos::CRay3> rays;

    std::mutex tagetPositionUpdateMutex;
    unsigned targetsNumber;
    argos::CVector2 position;
    CellularLog log;

    void parseLogConfig(argos::TConfigurationNode& t_tree);
    void saveLog();

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void addRobotsRays(argos::CCustomFootBotEntity& footbot);
    void wrapPointToArenaLimits(argos::CVector3 &point);
    std::vector<CoverageGrid::CellIndex> getCellsCoveredByRobots() const;
    void removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const;
    void updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells);
};


