#include "AvoidObstacleBehavior.h"
#include <limits>
#include <numeric>
#include <list>
#include <algorithm>

using namespace std;
using namespace argos;

/* Footbot dimensions */
static const Real BODY_RADIUS = 0.085036758f;

AvoidObstacleBehavior::AvoidObstacleBehavior(Sensors s, Actuators a)
    : ControllerBehavior(s, a)
    , histogramAlpha(5)
    , obstacleHistogram(static_cast<size_t>(CDegrees(360)/histogramAlpha), false)
    , histogramThresholdHysteresis(0.12f, 0.2f)
{}

CVector2 AvoidObstacleBehavior::proceed() {
//    LOG << "Avoid obstacle!" << endl;
    CDegrees closestFree(180);
    for (size_t i = 0; i < obstacleHistogram.size(); i++) {
        if (!obstacleHistogram.at(i)) {
            auto angleDiff = ((histogramAlpha * i) - CDegrees(0)).SignedNormalize();
            if (angleDiff.GetAbsoluteValue() < closestFree.GetAbsoluteValue())
                closestFree = angleDiff;
        }
    }
//    LOG << "Free angle " << closestFree << endl;
    return move(closestFree);
}

bool AvoidObstacleBehavior::isRoadClear(CVector2 desiredVelocity) {
    if (desiredVelocity.SquareLength() == 0)
        return true;

    updateObstacleHistogram();

    bool isObstacleAtDesiredAngle = false;
    CDegrees velocityAngle = ToDegrees(desiredVelocity.Angle()).UnsignedNormalize();

//    LOG << "[";
//    for (auto isObstacle : obstacleHistogram)
//        LOG << (isObstacle ? "|" : ".");
//    LOG << "] desired = " << velocityAngle << "\n";


    if (velocityAngle.GetValue() == 0)
        velocityAngle.SetValue(360);
    for (size_t i = 0; i < obstacleHistogram.size(); i++) {
        auto angle = histogramAlpha * i;
        auto nextAngle = histogramAlpha * (i+1);
        if (velocityAngle > angle && velocityAngle <= nextAngle) {
            isObstacleAtDesiredAngle |= obstacleHistogram.at(i);
            if (i + 1 < obstacleHistogram.size())
                isObstacleAtDesiredAngle |= obstacleHistogram.at(i + 1);
            else
                isObstacleAtDesiredAngle |= obstacleHistogram.at(0);
        }
    }
    return !isObstacleAtDesiredAngle;
}

void AvoidObstacleBehavior::updateObstacleHistogram() {
    vector<list<Real>> magnitudeHistogramVariables(obstacleHistogram.size());

    for (auto r : sensors.proximity.GetReadings()) {
        auto distance = getObstacleDistanceFromFootbotProximityReading(r.Value);
        if (isinf(distance))
            continue;

//        LOG << distance << ", ";

        const auto clearance = 0.01f;
        auto enlargementAngle = ASin((BODY_RADIUS + clearance) / distance);
        auto lowerAngleBoundary = (r.Angle - enlargementAngle).UnsignedNormalize();
        auto upperAngleBoundary = (r.Angle + enlargementAngle).UnsignedNormalize();
        auto isInBoundary = generateIsInBoundaryCheck(lowerAngleBoundary, upperAngleBoundary);

        for (size_t i = 0; i < obstacleHistogram.size(); i++) {
            auto angle = i * ToRadians(histogramAlpha);
            if (isInBoundary(angle))
                magnitudeHistogramVariables.at(i).push_back(r.Value * r.Value);
        }
    }
//    LOG << "\n";

    vector<Real> magnitudeHistogram(magnitudeHistogramVariables.size());
    for (size_t i = 0; i < magnitudeHistogramVariables.size(); i++) {
        auto& l = magnitudeHistogramVariables.at(i);
        auto maxEl = max_element(l.begin(), l.end());
        if (maxEl != l.end())
            magnitudeHistogram.at(i) = *maxEl;
//        LOG << magnitudeHistogram.at(i) << ", ";
    }
//    LOG << "\n";

    fill(obstacleHistogram.begin(), obstacleHistogram.end(), false);
    for (size_t i = 0; i < obstacleHistogram.size(); i++) {
        if (magnitudeHistogram.at(i) > histogramThresholdHysteresis.GetMax())
            obstacleHistogram.at(i) = true;
        else if (magnitudeHistogram.at(i) < histogramThresholdHysteresis.GetMin())
            obstacleHistogram.at(i) = false;
        else if (i > 0)
            obstacleHistogram.at(i) = obstacleHistogram.at(i - 1);
    }
}

function<bool(const CRadians&)> AvoidObstacleBehavior::generateIsInBoundaryCheck(const CRadians& lowerBoundary,
                                                      const CRadians& upperBoundary) const {
    if (lowerBoundary < upperBoundary)
        return [lowerBoundary, upperBoundary](const CRadians& a) { return a >= lowerBoundary && a <= upperBoundary; };
    else
        return [lowerBoundary, upperBoundary](const CRadians& a) { return a >= lowerBoundary || a <= upperBoundary; };
}
