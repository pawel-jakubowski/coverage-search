#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utils/VoronoiDiagram.h>
#include <utils/CoverageGrid.h>

class MbfoLoopFunction : public argos::CLoopFunctions {
public:
    static constexpr int maxCellConcentration = std::numeric_limits<int>::max();

    MbfoLoopFunction() : coverage(maxCellConcentration) {}
    virtual ~MbfoLoopFunction() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;
    virtual void PreStep() override;
    virtual void PostStep() override;
    virtual void Reset() override;

    void update();
    const CoverageGrid& getCoverageGrid();
    const std::vector<VoronoiDiagram::Cell>& getVoronoiCells();
    const VoronoiDiagram::Cell* getVoronoiCell(std::string id);
    const std::vector<const VoronoiDiagram::Cell*> getNeighbouringVoronoiCells(std::string id);
    const std::map<std::string, argos::CVector3> getRobotsPositions();
    const std::vector<argos::CRay3> getRays();

private:
    CoverageGrid coverage;
    VoronoiDiagram voronoi;
    std::map<std::string, argos::CVector3> robotsPositions;
    std::map<std::string, const VoronoiDiagram::Cell*> robotsCells;
    std::vector<argos::CRay3> rays;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void addRobotsRays(argos::CFootBotEntity& footbot);

    void wrapPointToArenaLimits(argos::CVector3 &point);

    std::vector<CoverageGrid::CellIndex> getCellsCoveredByRobots() const;
    void removeDuplicates(std::vector<CoverageGrid::CellIndex>& cells) const;
    void updateCoverageCells(const std::vector<CoverageGrid::CellIndex>& affectedCells);
};
