#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <iostream>
#include <mutex>


class ClosestDistance : public argos::CLoopFunctions {
public:
    ClosestDistance() = default;
    virtual ~ClosestDistance() = default;

    virtual void Init(argos::TConfigurationNode& t_tree);
    virtual bool IsExperimentFinished();
    virtual void Destroy();

    void addRobotPosition(argos::CVector3 pos);
    void addTargetPosition(int id, const argos::CVector3& position);

private:
    struct Target {
        using Id = int;
        argos::UInt32 step;
        argos::CVector3 position;
    };

    struct PsoLog {
        std::string name;
        std::ofstream file;
        std::map<Target::Id, Target> targets;
        std::map<argos::UInt32, double> closestDistances;
    };

    std::mutex robotDistanceUpdateMutex;
    std::mutex tagetPositionUpdateMutex;

    double bestObtainedDistance = std::numeric_limits<double>::max();
    unsigned targetsNumber;
    argos::CVector2 position;
    PsoLog log;

    void parseTargetConfig(argos::TConfigurationNode& t_tree);
    void parseLogConfig(argos::TConfigurationNode& t_tree);

    void saveLog();
};

