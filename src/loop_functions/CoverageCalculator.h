#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <utils/CoverageGrid.h>

class CoverageCalculator : public argos::CLoopFunctions {
public:
    CoverageCalculator() {}
    virtual ~CoverageCalculator() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    std::vector<argos::CRay3> getGrid();
private:
    CoverageGrid coverage;
};


