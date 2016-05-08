#include "Mbfo.h"

#include <assert.h>

using namespace std;

namespace argos {

/* Footbot dimensions */
static const Real INTERWHEEL_DISTANCE        = 0.14f;
static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
static const Real HALF_INTERWHEEL_DISTANCE_IN_CM = HALF_INTERWHEEL_DISTANCE * 100;
static const Real TICKS_PER_SEC = 10;
static const unsigned long CHEMOTAXIS_LENGTH = 10;

Mbfo::Mbfo()
    : wheelsEngine(nullptr)
    , proximitySensor(nullptr)
    , velocity(5.0f)
    , minDistanceFromObstacle(0.1f)
    , minAngleFromObstacle(CDegrees(-45.0f), CDegrees(45.0f))
    , loopFnc(dynamic_cast<MbfoLoopFunction&>(CSimulator::GetInstance().GetLoopFunctions()))
    , step(0)
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
//    if (isRoadClear(obstacleProximity)) {
        CRadians angleX, angleY, angleZ;
        positioningSensor->GetReading().Orientation.ToEulerAngles(angleX, angleY, angleZ);
        CDegrees robotsOrientation = ToDegrees(angleX);

        Real angleEpsilon = 1;

        if (step % CHEMOTAXIS_LENGTH == 0) {
            const auto& cell = getVoronoiCell();
            auto nextBestDirections = findNextBestDirectionsInVoronoiCell(cell);

            // Tumble or swim
            LOG << "[" << GetId() << "] "
                << "(possible directions: " << nextBestDirections.size() << ") "
                << robotsOrientation << " -> "
                << desiredDirection
                << std::endl;

            // Determine new direction
            auto sameDirection = std::find_if(nextBestDirections.begin(), nextBestDirections.end(),
                [&](NextDirection d){ return (d.angle - this->desiredDirection).GetAbsoluteValue() < angleEpsilon; });
            if (sameDirection == nextBestDirections.end()) {
                auto randomIndex = rand() % nextBestDirections.size();
                desiredDirection = nextBestDirections.at(randomIndex).angle;
            }
        }

        if ((robotsOrientation - desiredDirection).GetAbsoluteValue() > angleEpsilon)
            tumble(robotsOrientation);
        else
            swim();
//    }
//    else
//        avoidObstacle(obstacleProximity);
    step++;
}

void Mbfo::tumble(const CDegrees &robotsOrientation) {
    auto angleDiff = (robotsOrientation - desiredDirection).SignedNormalize();
    auto rotationDirection = getRotationDirection(angleDiff);
    rotationSpeed = ToRadians(angleDiff).GetAbsoluteValue() * HALF_INTERWHEEL_DISTANCE_IN_CM * TICKS_PER_SEC;
    LOG << "[" << GetId() << "] "
        << "tumble "
        << "(start: "   << robotsOrientation
        << ", end: "    << desiredDirection
        << ", diff: "   << angleDiff
        << ", speed: "  << rotationSpeed
        << ")" << std::endl;
    rotate(rotationDirection);
}

void Mbfo::swim() const {
    LOG << "[" << GetId() << "] swim (speed: " << velocity << ")" << std::endl;
    wheelsEngine->SetLinearVelocity(this->velocity, this->velocity);
}

CDegrees Mbfo::getAngleBetweenPoints(const CVector3 &a, const CVector3 &b) const {
    return ToDegrees(ATan2(b.GetX() - a.GetX(), b.GetY() - a.GetY()));
}

std::vector<Mbfo::NextDirection> Mbfo::findNextBestDirectionsInVoronoiCell(const VoronoiDiagram::Cell &cell) const {
    const CVector3 realPosition = positioningSensor->GetReading().Position;
    const CVector2 positionCellIndex = loopFnc.getCoverageGrid().getCellIndex(realPosition);

    // Complete search
    std::vector<Mbfo::NextDirection> nextDirections;
    Real bestValue = 0;
    for (auto& i : cell.coverageCells) {
        CVector2 v(i.x, i.y);
        auto& gridCell = getCoverageCell(v);

        // Take only cells that are better or same as current bestValue
        if (bestValue <= gridCell.concentration) {
            bestValue = gridCell.concentration;
            Real distance = calculateDistance(v, positionCellIndex);
            CDegrees angle = getAngleBetweenPoints(realPosition, gridCell.center);
            nextDirections.push_back(NextDirection{gridCell.concentration, distance, angle});
        }
    }

    // remove elements lower than max
    auto elementsLowerThanBestValue = std::remove_if(nextDirections.begin(), nextDirections.end(),
        [bestValue](NextDirection d){ return d.value < bestValue; });
    nextDirections.erase(elementsLowerThanBestValue, nextDirections.end());

    // From vector with best values find closest distance
    Real closestDistance = std::numeric_limits<Real>::max();
    for (auto& d : nextDirections)
        closestDistance = (closestDistance > d.distance) ? d.distance : closestDistance;

    // remove elements with greater distance than closest one
    auto elementsWithSameDistance = std::remove_if(nextDirections.begin(), nextDirections.end(),
        [closestDistance](NextDirection d){ return d.distance > closestDistance; });
    nextDirections.erase(elementsWithSameDistance, nextDirections.end());

    return nextDirections;
}

const CoverageGrid::Cell& Mbfo::getCoverageCell(CVector2 index) const {
    assert(coverage != nullptr);
    return coverage->getGrid().at(index.GetX()).at(index.GetY());
}

const Real Mbfo::calculateDistance(const CVector2& a, const CVector2& b) const {
    return (a - b).SquareLength();
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
        wheelsEngine->SetLinearVelocity(rotationSpeed, -rotationSpeed);
    else if (rotationDirection == Direction::left)
        wheelsEngine->SetLinearVelocity(-rotationSpeed, rotationSpeed);
}

}

REGISTER_CONTROLLER(Mbfo, "mbfo_controller")
