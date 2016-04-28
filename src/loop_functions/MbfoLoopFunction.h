#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utils/VoronoiDiagram.h>
#include <utils/CoverageGrid.h>

class MbfoLoopFunction : public argos::CLoopFunctions {
public:
    MbfoLoopFunction() {}
    virtual ~MbfoLoopFunction() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    void PreStep() override;
    void update();
    std::vector<std::vector<CoverageGrid::Cell>> getCoverageGrid();
    std::vector<argos::CVector3> getVoronoiVertices();
    std::vector<argos::CRay3> getVoronoiEdges();

private:
    CoverageGrid coverage;
    VoronoiDiagram voronoi;
    std::vector<argos::CVector3> robotsPositions;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
};


