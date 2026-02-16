#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

using Quaterniond = Eigen::Quaternion<double>;

//================================//
class Camera
{
public:
    Camera(float aspectRatio, float fovY, float nearPlane, float farPlane);

    //================================//
    void SetPosition(const Eigen::Vector3f& position)
    {
        this->position = position;
        UpdateViewMatrix();
    }

    //================================//
    void SetFOV(float fovY)
    {
        this->fovY = fovY;
        UpdateViewMatrix();
    }

    //================================//
    void SetAspectRatio(float aspectRatio)
    {
        this->aspectRatio = aspectRatio;
        UpdateViewMatrix();
    }

    //================================//
    void SetClippingPlanes(float nearPlane, float farPlane)
    {
        this->nearPlane = nearPlane;
        this->farPlane = farPlane;
        UpdateViewMatrix();
    }

    //================================//
    void SetOrientation(const Quaterniond& orientation)
    {
        this->orientation = orientation;
        UpdateCameraVectors();
    }

    //================================//
    void MoveBy(const Eigen::Vector3f& delta)
    {
        position += delta;
        UpdateViewMatrix();
    }

    //================================//
    void MoveForwardBy(float delta)
    {
        position += forward * delta;
        UpdateViewMatrix();
    }

    //================================//
    void MoveRightBy(float delta)
    {
        position += right * delta;
        UpdateViewMatrix();
    }

    //================================//
    void MoveUpBy(float delta)
    {
        position += up * delta;
        UpdateViewMatrix();
    }

    //================================//
    void MoveWorldUpBy(float delta)
    {
        position += worldUp * delta;
        UpdateViewMatrix();
    }

    //================================//
    void MoveLocal(const Eigen::Vector3f& delta)
    {
        position += forward * delta.z() + right * delta.x() + up * delta.y();
        UpdateViewMatrix();
    }

    //================================//
    void RotateBy(const Quaterniond& delta)
    {
        orientation = delta * orientation;
        UpdateCameraVectors();
    }

    //================================//
    void RotateByMouseMovement(float deltaX, float deltaY)
    {
        float yaw = deltaX * rotationSpeed;
        float pitch = deltaY * rotationSpeed;

        Quaterniond yawRotation(Eigen::AngleAxisd(yaw, worldUp.cast<double>()));
        Quaterniond pitchRotation(Eigen::AngleAxisd(pitch, right.cast<double>()));

        orientation = yawRotation * pitchRotation * orientation;
        UpdateCameraVectors();
    }

    //================================//
    void LookAtDirection(const Eigen::Vector3f& direction)
    {
        forward = direction.normalized();
        right = forward.cross(worldUp).normalized();
        up = right.cross(forward).normalized();

        UpdateCameraVectors();
    }

    //================================//
    void LookAtPoint(const Eigen::Vector3f& target)
    {
        Eigen::Vector3f direction = target - position;
        LookAtDirection(direction);
    }


private:
    Eigen::Vector3f position;

    Eigen::Vector3f forward;
    Eigen::Vector3f up;
    Eigen::Vector3f right;

    Eigen::Vector3f worldUp;

    float fovY;
    float aspectRatio;
    float nearPlane;
    float farPlane;

    Quaterniond orientation;

    float movementSpeed;
    float rotationSpeed;

    Eigen::Matrix4f viewMatrix;
    Eigen::Matrix4f projectionMatrix;

    void UpdateCameraVectors();
    void UpdateViewMatrix();
};

#endif // CAMERA_HPP