#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <iostream>

class ClosestDistance : public argos::CLoopFunctions {
public:
    ClosestDistance() = default;
    virtual ~ClosestDistance() = default;

    virtual void Init(argos::TConfigurationNode& t_tree);
    virtual void PostStep();
    virtual void Destroy();
private:
    struct PsoLog {
        std::string name;
        std::ofstream file;
        std::map<argos::UInt32, double> closestDistances;
    };
    double bestObtainedDistance = std::numeric_limits<double>::max();
    argos::CVector2 position;
    PsoLog log;

    void parseTargetConfig(argos::TConfigurationNode& t_tree);
    void parseLogConfig(argos::TConfigurationNode& t_tree);

    void saveLog();
};

