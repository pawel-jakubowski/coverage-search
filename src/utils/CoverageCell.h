#pragma once

#include <argos3/core/utility/math/ray3.h>
#include <list>

struct CoverageCell {
    std::list<argos::CRay3> edges;
    argos::CVector3 center;
    int concentration;
};
