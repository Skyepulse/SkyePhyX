#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Quaternionf = Eigen::Quaternion<float>;

// Forward declaration of Solver to avoid circular dependency
class Solver;

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
class Transform
{
public:
    Transform() : position(Eigen::Vector3f::Zero()), rotation(Quaternionf::Identity()), scale(Eigen::Vector3f::Ones()) {}

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
    void GetPosition(Eigen::Vector3f& outPosition) const
    {
        outPosition = position;
    }

    //================================//
    void GetRotation(Quaternionf& outRotation) const
    {
        outRotation = rotation;
    }

    //================================//
    void GetScale(Eigen::Vector3f& outScale) const
    {
        outScale = scale;
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

    Transform transform;
    Mesh* next = nullptr;

    ModelType modelType;
    Eigen::Vector3f color;
    
    Solver* solver;

    // Physics values
    float mass;
    float density;

    float friction;

    Eigen::Vector3f velocity;
    Eigen::Vector3f prevVelocity;
    Eigen::Vector3f angularVelocity;
    Eigen::Vector3f prevAngularVelocity;

    bool isStatic;
};

#endif // GEOMETRY_HPP