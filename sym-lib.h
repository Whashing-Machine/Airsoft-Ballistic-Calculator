#ifndef SYM_LIB_H
#define SYM_LIB_H

#include <iosfwd>
#include <vector>

constexpr double kPi = 3.14159265358979323846;
constexpr double kBbDiameterMeters = 0.006;
constexpr double kBbRadiusMeters = kBbDiameterMeters / 2.0;
constexpr double kGravity = 9.81;
constexpr double kAirDensity = 1.225;
constexpr double kDefaultAirViscosity = 0.0000181;
constexpr double kCrossSectionArea = kPi * kBbRadiusMeters * kBbRadiusMeters;
constexpr double kDragCoefficient = 0.47;

class Vector3 {
public:
    double x;
    double y;
    double z;

    Vector3();
    Vector3(double xValue, double yValue, double zValue);

    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator-() const;
    Vector3 operator*(double scalar) const;
    Vector3 operator/(double scalar) const;
    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(double scalar);

    double magnitude() const;
    Vector3 normalized() const;
    double dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
};

Vector3 operator*(double scalar, const Vector3& vector);
std::ostream& operator<<(std::ostream& output, const Vector3& vector);

class SimStep {
public:
    double time;
    Vector3 position;
    Vector3 linearVelocity;
    Vector3 rotationalPosition;
    Vector3 rotationalVelocity;
    double dragCoefficient;
    double liftCoefficient;
    double spinRatio;
    double reynoldsNumber;
    double rotationalReynoldsNumber;

    SimStep();
    SimStep(double timeValue,
            const Vector3& positionValue,
            const Vector3& linearVelocityValue,
            const Vector3& rotationalPositionValue,
            const Vector3& rotationalVelocityValue);
};

struct SimulationSettings {
    double massKg = 0.00020;
    double angularVelocityRadPerSecond = 1500.0;
    double initialSpeedMetersPerSecond = 100.0;
    double launchAngleDegrees = 0.0;
    double startingHeightMeters = 1.5;
    double timeStepSeconds = 0.001;
    double maxSimulationSeconds = 5.0;
    double airViscosity = kDefaultAirViscosity;
};

double calculateReynoldsNumber(double speed, double airViscosity);
double calculateRotationalReynoldsNumber(double omega, double airViscosity);
double calculateSpinRatio(double speed, double omega);
double calculateLiftCoefficient(double spinRatio);
void updateDiagnostics(SimStep& step, double airViscosity);

class ForceModel {
public:
    virtual ~ForceModel() = default;
    virtual Vector3 calculateForce(const SimStep& step, double massKg) const = 0;
};

class ConstantForceModel : public ForceModel {
protected:
    Vector3 force;

public:
    explicit ConstantForceModel(const Vector3& forceValue);
    Vector3 calculateForce(const SimStep& step, double massKg) const override;
};

class GravityForce : public ConstantForceModel {
public:
    explicit GravityForce(double massKg);
};

class AerodynamicForce : public ForceModel {
protected:
    double airDensity;
    double area;

public:
    AerodynamicForce(double airDensityValue, double areaValue);
};

class DragForce : public AerodynamicForce {
public:
    DragForce(double airDensityValue, double areaValue);
    Vector3 calculateForce(const SimStep& step, double massKg) const override;
};

class MagnusForce : public AerodynamicForce {
public:
    MagnusForce(double airDensityValue, double areaValue);
    Vector3 calculateForce(const SimStep& step, double massKg) const override;
};

std::vector<SimStep> runSimulation(const SimulationSettings& settings);
bool writeTrajectoryCsv(const char* fileName, const std::vector<SimStep>& trajectory);

#endif
