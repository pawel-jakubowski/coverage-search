#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/ray3.h>

class VoronoiCell {
public:
    std::vector<argos::CRay3> edges;

    VoronoiCell(argos::Real diagramLiftOnZ = 0.02f);
    void fillMissingEdges(const argos::CRange<argos::CVector3>& limits);
    bool isInside(argos::CVector3 point) const;

private:
    const argos::Real diagramLiftOnZ;

    bool areVectorsEqual(const argos::CVector3 &a, const argos::CVector3 &b) const;
    bool areOnSameSide(const argos::CVector3 &a, const argos::CVector3 &b) const;
};


