#pragma once

#include <argos3/core/simulator/loop_functions.h>

class CoverageCalculator : public argos::CLoopFunctions {
public:
    CoverageCalculator() {}
    virtual ~CoverageCalculator() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    std::vector<argos::CRay3> getGrid();
private:
    const argos::Real cellSizeInMeters = 0.1f;
    const argos::Real gridLiftOnZ = 0.05f;
    std::vector<argos::CRay3> grid;
};


