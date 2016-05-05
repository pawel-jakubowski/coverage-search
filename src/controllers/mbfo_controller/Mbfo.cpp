#include "Mbfo.h"

#include <assert.h>

using namespace std;

namespace argos {

Mbfo::Mbfo()
    : wheelsEngine(nullptr)
    , proximitySensor(nullptr)
    , velocity(5.0f)
    , minDistanceFromObstacle(0.1f)
    , minAngleFromObstacle(CDegrees(-45.0f), CDegrees(45.0f))
    , loopFnc(dynamic_cast<MbfoLoopFunction&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void Mbfo::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");

    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
            minDistanceFromObstacle);

    coverage = &loopFnc.getCoverageGrid();

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(coverage != nullptr);
}

void Mbfo::ControlStep() {
    CVector2 obstacleProximity = getWeightedProximityReading();
    if (isRoadClear(obstacleProximity)) {
        const auto& cell = getVoronoiCell();
        auto nextBestPosition = findNextBestPositionInVoronoiCell(cell);

        LOG << "[" << GetId() << "] "
            << positioningSensor->GetReading().Position << " -> " << nextBestPosition
            << std::endl;

        // Determine new direction
        // Tumble or swim

        wheelsEngine->SetLinearVelocity(velocity, velocity);
    }
    else
        avoidObstacle(obstacleProximity);
}

CVector3 Mbfo::findNextBestPositionInVoronoiCell(const VoronoiDiagram::Cell &cell) const {
    const CVector2 positionCellIndex = getCurrentPosition();
    // Complete search
    CVector2 nextBestPosition;
    int bestConcentration = -1;
    Real bestDistance = numeric_limits<Real>::max();
    for (auto& i : cell.coverageCells) {
        CVector2 v(i.x, i.y);
        auto& gridCell = getCoverageCell(v);
        if (gridCell.concentration >= bestConcentration) {
            Real distance = calculateDistance(positionCellIndex, v);
            if (distance < bestDistance) {
                bestConcentration = gridCell.concentration;
                bestDistance = distance;
                nextBestPosition = v;
            }
        }
    }
    return coverage->getGrid().at(nextBestPosition.GetX()).at(nextBestPosition.GetY()).center;
}

const CoverageGrid::Cell& Mbfo::getCoverageCell(CVector2 index) const {
    assert(coverage != nullptr);
    return coverage->getGrid().at(index.GetX()).at(index.GetY());
}

const Real Mbfo::calculateDistance(const CVector2& a, const CVector2& b) const {
    return (a - b).SquareLength();
}

const CVector2 Mbfo::getCurrentPosition() const {
    return loopFnc.getCoverageGrid().getCellIndex(positioningSensor->GetReading().Position);
}

const VoronoiDiagram::Cell& Mbfo::getVoronoiCell() {
    const auto cell = loopFnc.getVoronoiCell(GetId());
    assert(cell != nullptr);
    return *cell;
}

CVector2 Mbfo::getWeightedProximityReading() {
    const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
    CVector2 accumulator;
    for (size_t i = 0; i < proximityReadings.size(); ++i) {
        accumulator += CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }
    accumulator /= proximityReadings.size();
    return accumulator;
}

bool Mbfo::isRoadClear(const CVector2& obstacleProximity) {
    CDegrees obstacleAngle(ToDegrees(obstacleProximity.Angle()));
    CDegrees safeAngle(150.0f);
    return (minAngleFromObstacle.WithinMinBoundIncludedMaxBoundIncluded(obstacleAngle)
            && obstacleProximity.Length() < minDistanceFromObstacle) ||
            (obstacleAngle < -safeAngle || obstacleAngle > safeAngle);
}

void Mbfo::avoidObstacle(const CVector2& obstacleProximity) {
    auto obstacleAngle = ToDegrees(obstacleProximity.Angle());
    auto rotationDirection = getRotationDirection(obstacleAngle);
    rotate(rotationDirection);
}

Mbfo::Direction Mbfo::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::right : Direction::left;
}

void Mbfo::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::right)
        wheelsEngine->SetLinearVelocity(velocity, -0.1*velocity);
    else if (rotationDirection == Direction::left)
        wheelsEngine->SetLinearVelocity(-0.1*velocity, velocity);
}

}

REGISTER_CONTROLLER(Mbfo, "mbfo_controller")
