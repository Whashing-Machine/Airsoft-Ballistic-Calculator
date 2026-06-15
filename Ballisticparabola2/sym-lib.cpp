#include "sym-lib.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <ostream>

Vector3::Vector3() : x(0.0), y(0.0), z(0.0) {}

Vector3::Vector3(double xValue, double yValue, double zValue)
    : x(xValue), y(yValue), z(zValue) {}

Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
}

Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator*(double scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
}

Vector3 Vector3::operator/(double scalar) const {
    return Vector3(x / scalar, y / scalar, z / scalar);
}

Vector3& Vector3::operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3& Vector3::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

double Vector3::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

Vector3 Vector3::normalized() const {
    const double mag = magnitude();
    if (mag <= std::numeric_limits<double>::epsilon()) {
        return Vector3();
    }
    return *this / mag;
}

double Vector3::dot(const Vector3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

Vector3 operator*(double scalar, const Vector3& vector) {
    return vector * scalar;
}

std::ostream& operator<<(std::ostream& output, const Vector3& vector) {
    output << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    return output;
}

SimStep::SimStep()
    : time(0.0),
      position(),
      linearVelocity(),
      rotationalPosition(),
      rotationalVelocity(),
      dragCoefficient(0.0),
      liftCoefficient(0.0),
      spinRatio(0.0),
      reynoldsNumber(0.0),
      rotationalReynoldsNumber(0.0) {}

SimStep::SimStep(double timeValue,
                 const Vector3& positionValue,
                 const Vector3& linearVelocityValue,
                 const Vector3& rotationalPositionValue,
                 const Vector3& rotationalVelocityValue)
    : time(timeValue),
      position(positionValue),
      linearVelocity(linearVelocityValue),
      rotationalPosition(rotationalPositionValue),
      rotationalVelocity(rotationalVelocityValue),
      dragCoefficient(0.0),
      liftCoefficient(0.0),
      spinRatio(0.0),
      reynoldsNumber(0.0),
      rotationalReynoldsNumber(0.0) {}

double calculateReynoldsNumber(double speed, double airViscosity) {
    if (speed <= std::numeric_limits<double>::epsilon()
        || airViscosity <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return kBbDiameterMeters * kAirDensity * speed / airViscosity;
}

double calculateRotationalReynoldsNumber(double omega, double airViscosity) {
    if (omega <= std::numeric_limits<double>::epsilon()
        || airViscosity <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return kAirDensity * kBbRadiusMeters * kBbRadiusMeters * omega / airViscosity;
}

double calculateSpinRatio(double speed, double omega) {
    if (speed <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return kBbRadiusMeters * omega / speed;
}

double calculateLiftCoefficient(double spinRatio) {
    if (spinRatio <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return std::min(0.5 * spinRatio, 0.5);
}

void updateDiagnostics(SimStep& step, double airViscosity) {
    const double speed = step.linearVelocity.magnitude();
    const double spinSpeed = step.rotationalVelocity.magnitude();

    step.spinRatio = calculateSpinRatio(speed, spinSpeed);
    step.reynoldsNumber = calculateReynoldsNumber(speed, airViscosity);
    step.rotationalReynoldsNumber = calculateRotationalReynoldsNumber(spinSpeed, airViscosity);
    step.dragCoefficient = kDragCoefficient;
    step.liftCoefficient = calculateLiftCoefficient(step.spinRatio);
}

ConstantForceModel::ConstantForceModel(const Vector3& forceValue)
    : force(forceValue) {}

Vector3 ConstantForceModel::calculateForce(const SimStep& step, double massKg) const {
    (void)step;
    (void)massKg;
    return force;
}

GravityForce::GravityForce(double massKg)
    : ConstantForceModel(Vector3(0.0, 0.0, -massKg * kGravity)) {}

AerodynamicForce::AerodynamicForce(double airDensityValue, double areaValue)
    : airDensity(airDensityValue), area(areaValue) {}

DragForce::DragForce(double airDensityValue, double areaValue)
    : AerodynamicForce(airDensityValue, areaValue) {}

Vector3 DragForce::calculateForce(const SimStep& step, double massKg) const {
    (void)massKg;
    const double speed = step.linearVelocity.magnitude();
    if (speed <= std::numeric_limits<double>::epsilon()) {
        return Vector3();
    }

    const double forceMagnitude = 0.5 * airDensity * kDragCoefficient * area * speed * speed;
    return -step.linearVelocity.normalized() * forceMagnitude;
}

MagnusForce::MagnusForce(double airDensityValue, double areaValue)
    : AerodynamicForce(airDensityValue, areaValue) {}

Vector3 MagnusForce::calculateForce(const SimStep& step, double massKg) const {
    (void)massKg;
    const double speed = step.linearVelocity.magnitude();
    const double spinSpeed = step.rotationalVelocity.magnitude();
    if (speed <= std::numeric_limits<double>::epsilon()
        || spinSpeed <= std::numeric_limits<double>::epsilon()) {
        return Vector3();
    }

    const Vector3 liftDirectionVector = step.rotationalVelocity.cross(step.linearVelocity);
    const double liftDirectionMagnitude = liftDirectionVector.magnitude();
    if (liftDirectionMagnitude <= std::numeric_limits<double>::epsilon()) {
        return Vector3();
    }

    const double spinRatio = calculateSpinRatio(speed, spinSpeed);
    const double liftCoefficient = calculateLiftCoefficient(spinRatio);
    const double forceMagnitude = 0.5 * airDensity * area * std::abs(liftCoefficient) * speed * speed;
    return -liftDirectionVector.normalized() * forceMagnitude;
}

std::vector<SimStep> runSimulation(const SimulationSettings& settings) {
    const double launchAngleRadians = settings.launchAngleDegrees * kPi / 180.0;
    const Vector3 initialVelocity(
        settings.initialSpeedMetersPerSecond * std::cos(launchAngleRadians),
        0.0,
        settings.initialSpeedMetersPerSecond * std::sin(launchAngleRadians)
    );

    SimStep current(
        0.0,
        Vector3(0.0, 0.0, settings.startingHeightMeters),
        initialVelocity,
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, settings.angularVelocityRadPerSecond, 0.0)
    );

    std::vector<std::unique_ptr<ForceModel>> forceModels;
    forceModels.push_back(std::make_unique<GravityForce>(settings.massKg));
    forceModels.push_back(std::make_unique<DragForce>(
        kAirDensity,
        kCrossSectionArea
    ));
    forceModels.push_back(std::make_unique<MagnusForce>(
        kAirDensity,
        kCrossSectionArea
    ));

    std::vector<SimStep> trajectory;
    updateDiagnostics(current, settings.airViscosity);
    trajectory.push_back(current);

    while (current.time < settings.maxSimulationSeconds && current.position.z > 0.0) {
        auto accelerationFor = [&](const Vector3& position, const Vector3& velocity) {
            SimStep sample(
                current.time,
                position,
                velocity,
                current.rotationalPosition,
                current.rotationalVelocity
            );

            Vector3 totalForce;
            for (const auto& forceModel : forceModels) {
                totalForce += forceModel->calculateForce(sample, settings.massKg);
            }
            return totalForce / settings.massKg;
        };

        const double dt = settings.timeStepSeconds;
        const Vector3 k1Position = current.linearVelocity;
        const Vector3 k1Velocity = accelerationFor(current.position, current.linearVelocity);

        const Vector3 k2Position = current.linearVelocity + 0.5 * dt * k1Velocity;
        const Vector3 k2Velocity = accelerationFor(
            current.position + 0.5 * dt * k1Position,
            current.linearVelocity + 0.5 * dt * k1Velocity
        );

        const Vector3 k3Position = current.linearVelocity + 0.5 * dt * k2Velocity;
        const Vector3 k3Velocity = accelerationFor(
            current.position + 0.5 * dt * k2Position,
            current.linearVelocity + 0.5 * dt * k2Velocity
        );

        const Vector3 k4Position = current.linearVelocity + dt * k3Velocity;
        const Vector3 k4Velocity = accelerationFor(
            current.position + dt * k3Position,
            current.linearVelocity + dt * k3Velocity
        );

        current.position += dt / 6.0 * (k1Position + 2.0 * k2Position + 2.0 * k3Position + k4Position);
        current.linearVelocity += dt / 6.0 * (k1Velocity + 2.0 * k2Velocity + 2.0 * k3Velocity + k4Velocity);
        current.rotationalPosition += current.rotationalVelocity * dt;
        current.time += dt;

        if (current.position.z < 0.0) {
            current.position.z = 0.0;
        }

        updateDiagnostics(current, settings.airViscosity);
        trajectory.push_back(current);
    }

    return trajectory;
}

bool writeTrajectoryCsv(const char* fileName, const std::vector<SimStep>& trajectory) {
    std::ofstream file(fileName);
    if (!file) {
        return false;
    }

    file << "time_s,"
         << "position_x_m,position_y_m,position_z_m,"
         << "velocity_x_mps,velocity_y_mps,velocity_z_mps,"
         << "rotation_x_rad,rotation_y_rad,rotation_z_rad,"
         << "angular_velocity_x_radps,angular_velocity_y_radps,angular_velocity_z_radps,"
         << "drag_coefficient,lift_coefficient,spin_ratio,"
         << "reynolds_number,rotational_reynolds_number\n";

    file << std::fixed << std::setprecision(8);
    for (const SimStep& step : trajectory) {
        file << step.time << ","
             << step.position.x << "," << step.position.y << "," << step.position.z << ","
             << step.linearVelocity.x << "," << step.linearVelocity.y << "," << step.linearVelocity.z << ","
             << step.rotationalPosition.x << "," << step.rotationalPosition.y << "," << step.rotationalPosition.z << ","
             << step.rotationalVelocity.x << "," << step.rotationalVelocity.y << "," << step.rotationalVelocity.z << ","
             << step.dragCoefficient << "," << step.liftCoefficient << "," << step.spinRatio << ","
             << step.reynoldsNumber << "," << step.rotationalReynoldsNumber << "\n";
    }

    return true;
}
