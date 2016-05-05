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

    void update();
    const std::vector<std::vector<CoverageGrid::Cell>>& getCoverageGrid();
    const std::vector<VoronoiDiagram::Cell>& getVoronoiCells();
    const VoronoiDiagram::Cell* getVoronoiCell(std::string id);

private:
    CoverageGrid coverage;
    VoronoiDiagram voronoi;
    std::vector<argos::CVector3> robotsPositions;
    std::vector<std::string> robotsIds;
    std::map<std::string, const VoronoiDiagram::Cell*> cellsPerId;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
};
