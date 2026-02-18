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
inline Eigen::Matrix3f skew(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f S;
    S <<     0.f, -v.z(),  v.y(),
          v.z(),     0.f, -v.x(),
         -v.y(),  v.x(),     0.f;
    return S;
}

#endif // MATH_HPP