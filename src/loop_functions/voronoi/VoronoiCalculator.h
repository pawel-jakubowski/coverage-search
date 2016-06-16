#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utils/voronoi/VoronoiDiagram.h>


class VoronoiCalculator : public argos::CLoopFunctions {
public:
    VoronoiCalculator() {}
    virtual ~VoronoiCalculator() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    void update();
    std::vector<argos::CVector3> getVertices();
    std::vector<argos::CRay3> getEdges();

private:
    VoronoiDiagram voronoi;
    std::map<std::string, argos::CVector3> robotsPositions;

    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
};


