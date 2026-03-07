#ifndef MATH_HPP
#define MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

using Quaternionf = Eigen::Quaternion<float>;

//================================//
inline Eigen::Vector3f RotationDifference(const Quaternionf& q, const Quaternionf& p)
{
    Quaternionf dq = q * p.conjugate();
    if (dq.w() < 0.f) dq.coeffs() = -dq.coeffs();

    Eigen::Vector3f out;
    float sinHalf = dq.vec().norm();
    if (sinHalf < 1e-6f)
    {
        out = 2.f * dq.vec();
    }
    else
    {
        float halfAngle = std::atan2(sinHalf, dq.w());
        out = (2.f * halfAngle / sinHalf) * dq.vec();
    }

    return out;
}

//================================//
inline Quaternionf QuaternionFromDifference(const Eigen::Vector3f& v, float sign = 1.f)
{
    float angle = v.norm();
    Quaternionf q;
    if (angle < 1e-6f)
    {
        q.w() = 1.f;
        q.vec() = sign * 0.5f * v;
    }
    else
    {
        float halfAngle = 0.5f * angle;
        q.w() = std::cos(halfAngle);
        q.vec() = sign * std::sin(halfAngle) * (v / angle);
    }
    q.normalize();
    return q;
}

//================================//
inline Eigen::Vector3f QuaternionToRotVec(const Quaternionf& q)
{
    Quaternionf qn = q.w() >= 0.0f ? q : Quaternionf(-q.w(), -q.x(), -q.y(), -q.z());

    Eigen::Vector3f v = qn.vec();
    float sinHalf = v.norm();

    if (sinHalf < 1e-6f)
        return Eigen::Vector3f::Zero();

    float halfAngle = std::atan2(sinHalf, qn.w());
    return v * (2.0f * halfAngle / sinHalf);
}

//================================//
inline Eigen::Matrix3f skew(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f S;
    S <<     0.f, -v.z(),  v.y(),
          v.z(),     0.f, -v.x(),
         -v.y(),  v.x(),     0.f;
    return S;
}

//================================//
inline float rand01()
{
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

//================================//
inline Eigen::Vector3f genRandomPos(float minBounds[3], float maxBounds[3])
{
    float sizeX = maxBounds[0] - minBounds[0];
    float X = minBounds[0] + rand01() * sizeX;

    float sizeY = maxBounds[1] - minBounds[1];
    float Y = minBounds[1] + rand01() * sizeX;

    float sizeZ = maxBounds[2] - minBounds[2];
    float Z = minBounds[2] + rand01() * sizeZ;

    return Eigen::Vector3f(X, Y, Z);
}

//================================//
// NEO HOOKEAN SPECIFIC MATHS     //
//================================//
enum class EigenProjectionMode
{
    CLAMP = 0,
    ABSOLUTE = 1,
    ADAPTIVE = 2
};

namespace NeoHookeanMath
{
    //================================//
    struct SVDDecomposition
    {
        Eigen::Matrix3f U;
        Eigen::Vector3f S;
        Eigen::Matrix3f V;
    };

    //================================//
    struct HessianDecomposition
    {
        float eigenValues[9];
        Eigen::Vector3f scalingVectors[3];
    };

    //================================//
    SVDDecomposition svd(const Eigen::Matrix3f& F);

    //================================//
    Eigen::Matrix3f DeformationGradient(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3, const Eigen::Matrix3f& DmInv);

    //================================//
    float ComputeEnergyDensity(const Eigen::Matrix3f& F, float J, float mu, float lambda, float alpha);

    //================================//
    Eigen::Matrix3f ComputeFirstPiolaKirchhoff(const Eigen::Matrix3f& F, float J, float mu, float lambda, float alpha);

    //================================//
    HessianDecomposition ComputeEnergyHessian(const Eigen::Vector3f& sigma, float J, float mu, float lambda, float alpha);

    //================================//
    void ProjectEigenvalues(float* eigenvalues, int count, EigenProjectionMode mode, float trustRegionRho, float threshold = 0.01f, float epsilon = 1e-6f);

    //================================//
    Eigen::Matrix3f ReconstructVertexHessian(const SVDDecomposition& svd, const HessianDecomposition& hessDecomp, const float* projectedEigenValues, const Eigen::Vector3f& gradN, float restVolume);
} // NEO HOOKEAN MATH NAMESPACE

#endif // MATH_HPP