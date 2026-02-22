#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include "math.hpp"
#include <string>

using Vector6f    = Eigen::Matrix<float, 6, 1>;
using Matrix6f    = Eigen::Matrix<float, 6, 6>;

// Forward declaration of Solver to avoid circular dependency
class Solver;
struct Force;

//================================//
enum ModelType
{
    ModelType_Cube = 0,
    ModelType_Sphere = 1,
    ModelType_Pyramid = 2
};

//================================//
struct Vertex
{
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f uv;
};

//================================//
struct Triangle
{
    uint32_t vertexIndices[3];
};

//================================//
struct GPUMat3x3
{
    float col0[4];
    float col1[4];
    float col2[4];
};
static_assert(sizeof(GPUMat3x3) == 48, "Must match WGSL mat3x3 layout");

//================================//
struct GPULineData
{
    float start[3];
    float end[3];
    float color[4];
};

struct GPUDebugPointData
{
    float position[3];
    float _pad;
    float color[4];
};


//================================//
class Transform
{
public:
    Transform() : position(Eigen::Vector3f::Zero()), rotation(Quaternionf::Identity()), scale(Eigen::Vector3f::Ones()) {}

    //================================//
    Eigen::Vector3f TransformPoint(const Eigen::Vector3f& point) const
    {
        return rotation * (scale.asDiagonal() * point) + position;
    }

    //================================//
    void SetPosition(const Eigen::Vector3f& position)
    {
        this->position = position;
    }

    //================================//
    void SetRotation(const Quaternionf& rotation)
    {
        this->rotation = rotation;
    }

    //================================//
    void SetScale(const Eigen::Vector3f& scale)
    {
        this->scale = scale;
    }

    //================================//
    Eigen::Vector3f GetPosition() const
    {
        return position;
    }

    //================================//
    Quaternionf GetRotation() const
    {
        return rotation;
    }

    //================================//
    Eigen::Vector3f GetScale() const
    {
        return scale;
    }

    //================================//
    void GetModelMatrix(Eigen::Matrix4f& outModelMatrix) const
    {
        outModelMatrix = Eigen::Matrix4f::Identity();
        outModelMatrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();

        outModelMatrix.col(0) *= scale.x();
        outModelMatrix.col(1) *= scale.y();
        outModelMatrix.col(2) *= scale.z();

        outModelMatrix.block<3, 1>(0, 3) = position;
    }

    //================================//
    void GetNormalMatrix(GPUMat3x3& outNormalMatrix) const
    {
        Eigen::Matrix4f modelMatrix;
        GetModelMatrix(modelMatrix);
        Eigen::Matrix3f normal = modelMatrix.block<3, 3>(0, 0).inverse().transpose();

        std::memcpy(outNormalMatrix.col0, normal.col(0).data(), 3 * sizeof(float));
        outNormalMatrix.col0[3] = 0.0f;

        std::memcpy(outNormalMatrix.col1, normal.col(1).data(), 3 * sizeof(float));
        outNormalMatrix.col1[3] = 0.0f;

        std::memcpy(outNormalMatrix.col2, normal.col(2).data(), 3 * sizeof(float));
        outNormalMatrix.col2[3] = 0.0f;
    }

private:
    Eigen::Vector3f position;
    Quaternionf rotation;
    Eigen::Vector3f scale;
};

//================================//
struct Mesh
{
    Mesh(Solver* solver, ModelType modelType, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    ~Mesh();

    int solverIndex = -1;

    Transform transform;
    std::vector<Force*> forces;

    ModelType modelType;
    Eigen::Vector3f color;
    
    Solver* solver;

    std::string name = "Mesh";

    Eigen::Matrix3f cachedRotationMatrix;
    Matrix6f        cachedGeneralizedMass;

    // ---- Mass properties ----
    float mass      = 0.0f;
    float density   = 0.0f;
    float friction  = 0.0f;
    bool isStatic   = false;

    Eigen::Matrix3f inertiaTensorBody    = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f inertiaTensorBodyInv = Eigen::Matrix3f::Identity();

    // ---- Linear state ----
    Eigen::Vector3f velocity     = Eigen::Vector3f::Zero();
    Eigen::Vector3f prevVelocity = Eigen::Vector3f::Zero();

    // ---- Angular state ----
    Eigen::Vector3f angularVelocity     = Eigen::Vector3f::Zero();
    Eigen::Vector3f prevAngularVelocity = Eigen::Vector3f::Zero();

    Eigen::Vector3f lastPosition = Eigen::Vector3f::Zero();
    Quaternionf     lastRotation = Quaternionf::Identity();
    Eigen::Vector3f inertialPosition = Eigen::Vector3f::Zero();
    Quaternionf     inertialRotation = Quaternionf::Identity();

    float detectionRadius;

    bool isDragged = false;
    Eigen::Vector3f addedDragVelocity = Eigen::Vector3f::Zero();
    bool isParticle = false;

    // HELPERS //
    //================================//
    Eigen::Matrix3f GetWorldInverseInertiaTensor() const
    {
        if (isStatic) return Eigen::Matrix3f::Zero();

        Quaternionf R = transform.GetRotation();
        Eigen::Matrix3f Rmat = R.toRotationMatrix();
        return Rmat * inertiaTensorBodyInv * Rmat.transpose();
    }

    //================================//
    Vector6f GetGeneralizedVelocity() const
    {
        Vector6f v;
        v.head<3>() = velocity;
        v.tail<3>() = angularVelocity;
        return v;
    }

    //================================//
    void SetGeneralizedVelocity(const Vector6f& v)
    {
        velocity = v.head<3>();
        angularVelocity = v.tail<3>();
    }

    //================================//
    void IntegrateRotation(float dt)
    {
        if (isStatic || isParticle) return;

        Quaternionf w(0.f, angularVelocity.x(), angularVelocity.y(), angularVelocity.z());

        Quaternionf q = transform.GetRotation();

        Quaternionf qdot;
        qdot.w()   = 0.5f * (w * q).w();
        qdot.vec() = 0.5f * (w * q).vec();
        q.w()   += qdot.w()   * dt;
        q.vec() += qdot.vec() * dt;
        q.normalize();

        transform.SetRotation(q);
    }

    //================================//
    Vector6f GetDisplacementFromInertial() const
    {
        Vector6f dx;

        Eigen::Vector3f pos = transform.GetPosition();
        dx.head<3>() = pos - inertialPosition;

        Quaternionf qCurr = transform.GetRotation();
        dx.tail<3>() = RotationDifference(qCurr, inertialRotation);

        return dx;
    }
};

#endif // GEOMETRY_HPP