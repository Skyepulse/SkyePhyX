#include "camera.hpp"

#include <cmath>

#define Math_PI 3.14159265358979323846

//================================//
static void Mat4Perspective(float fovY, float aspect, float nearPlane, float farPlane, Eigen::Matrix4f& out)
{
    const float f = 1.0 / std::tan(fovY * 0.5);
    const float nf = 1.0 / (nearPlane - farPlane);

    out.setZero();
    out(0, 0) = f / aspect;
    out(1, 1) = f;
    out(2, 2) = farPlane * nf;
    out(2, 3) = -1.0f;
    out(3, 2) = (nearPlane * farPlane) * nf;
}

//================================//
void Camera::UpdateViewMatrix()
{
    Eigen::Vector3f target = position + forward;

    const Eigen::Vector3f zAxis = (position - target).normalized();
    const Eigen::Vector3f xAxis = up.cross(zAxis).normalized();
    const Eigen::Vector3f yAxis = zAxis.cross(xAxis);

    viewMatrix.setIdentity();
    viewMatrix.block<3, 1>(0, 0) = xAxis;
    viewMatrix.block<3, 1>(0, 1) = yAxis;
    viewMatrix.block<3, 1>(0, 2) = zAxis;
    viewMatrix(0, 3) = -xAxis.dot(position);
    viewMatrix(1, 3) = -yAxis.dot(position);
    viewMatrix(2, 3) = -zAxis.dot(position);
    viewMatrix(3, 3) = 1.0f;
}

//================================//
void Camera::UpdateCameraVectors()
{
    // Compute forward from orientation
    forward = (orientation * Eigen::Vector3d(0.0, 0.0, 1.0)).cast<float>();
    forward.normalize();

    right = forward.cross(worldUp);
    right.normalize();

    up = right.cross(forward);
    up.normalize();

    UpdateViewMatrix();
}

//================================//
Camera::Camera(float aspectRatio, float fovY = Math_PI / 4.0f, float nearPlane = 0.1f, float farPlane = 100.0f) : aspectRatio(aspectRatio), fovY(fovY), nearPlane(nearPlane), farPlane(farPlane)
{
    this->position = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
    this->worldUp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    this->forward = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    this->up = worldUp;
    this->right = forward.cross(up).normalized();

    const float yaw = Math_PI/2.0f;
    const float pitch = 0.0f;
    this->orientation = Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()));

    this->movementSpeed = 5.0f;
    this->rotationSpeed = 0.5f;

    // Initialize view and projection matrices
    this->viewMatrix.setIdentity();
    Mat4Perspective(this->fovY, this->aspectRatio, this->nearPlane, this->farPlane, this->projectionMatrix);

    this->UpdateCameraVectors();
}