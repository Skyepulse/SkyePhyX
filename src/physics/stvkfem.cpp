#include "energy.hpp"
using namespace STVKMath;

//================================//
STVKFEM::STVKFEM(Solver* solver,
            Mesh* body0, Mesh* body1, Mesh* body2,
            float youngsModulus, float poissonRatio)
    : Energy(solver, {body0, body1, body2}), body0(body0), body1(body1), body2(body2), youngsModulus(youngsModulus), poissonRatio(poissonRatio)
{
    lameMu     = youngsModulus / (2.f * (1.f + poissonRatio));
    lameLambda = youngsModulus * poissonRatio / ((1.f + poissonRatio) * (1.f - 2.f * poissonRatio));

    Eigen::Vector3f p0 = body0->transform.GetPosition();
    Eigen::Vector3f p1 = body1->transform.GetPosition();
    Eigen::Vector3f p2 = body2->transform.GetPosition();

    Eigen::Vector3f e0 = p1 - p0;
    Eigen::Vector3f e1 = p2 - p0;

    float x1   = e0.norm();
    float x2   = e1.dot(e0) / x1;
    float y2   = std::sqrt(std::max(0.f, e1.squaredNorm() - x2 * x2));

    Eigen::Matrix2f Dm2D;
    Dm2D << x1, x2,
            0.f, y2;
    DmInv2D = Dm2D.inverse();

    restArea = 0.5f * x1 * y2;

    Eigen::Matrix2f DmInv2DT = DmInv2D.transpose();
    gradN1 = DmInv2DT.col(0);
    gradN2 = DmInv2DT.col(1);
    gradN0 = -gradN1 - gradN2;
}

//================================//
void STVKFEM::ComputeEnergyTerms(Mesh* mesh, EigenProjectionMode projectionMode, float trustRegionRho)
{
    Eigen::Vector3f p0 = body0->transform.GetPosition();
    Eigen::Vector3f p1 = body1->transform.GetPosition();
    Eigen::Vector3f p2 = body2->transform.GetPosition();

    F32 F = DeformationGradient(p0, p1, p2, DmInv2D);

    cachedEnergy = restArea * ComputeEnergyDensity(F, lameMu, lameLambda);

    Eigen::Vector2f gradN;
    if      (mesh == body0) gradN = gradN0;
    else if (mesh == body1) gradN = gradN1;
    else                    gradN = gradN2;

    if ((p1 - p0).cross(p2 - p0).norm() < 2.f * restArea * 0.01f)
    {
        HandleInvertedElement(mesh, F, gradN);
        return;
    }

    F32 P = ComputeFirstPiolaKirchhoff(F, lameMu, lameLambda);
    grad.setZero();
    grad.head<3>() = restArea * (P * gradN);

    SVDDecomposition SVD = svd(F);
    HessianDecomposition hessDecomp = ComputeEnergyHessian(SVD.S, lameMu, lameLambda);

    float projectedEigenValues[4];
    for (int i = 0; i < 4; ++i)
        projectedEigenValues[i] = hessDecomp.eigenValues[i];
    NeoHookeanMath::ProjectEigenvalues(projectedEigenValues, 4, projectionMode, trustRegionRho, trustRegionThreshold);

    hess.setZero();
    hess.block<3, 3>(0, 0) = ReconstructVertexHessian(SVD, hessDecomp, projectedEigenValues, gradN, restArea);
}

//================================//
void STVKFEM::HandleInvertedElement(Mesh* mesh, const F32& F, const Eigen::Vector2f& gradN)
{
    float stiffPenalty = lameMu + lameLambda;

    F32 P = stiffPenalty * F;
    grad.setZero();
    grad.head<3>() = restArea * (P * gradN);

    hess.setZero();
    hess.block<3,3>(0,0) = restArea * stiffPenalty * Eigen::Matrix3f::Identity();
}

//================================//
void STVKFEM::AddLineData(std::vector<GPULineData>& data) const
{
    std::vector<std::array<Mesh*, 2>> edges = {
        {body0, body1}, {body1, body2}, {body2, body0}
    };

    for (auto& e : edges)
    {
        Eigen::Vector3f pA = e[0]->transform.GetPosition();
        Eigen::Vector3f pB = e[1]->transform.GetPosition();

        GPULineData line;
        Eigen::Map<Eigen::Vector3f>(line.start) = pA;
        Eigen::Map<Eigen::Vector3f>(line.end)   = pB;
        Eigen::Vector3f color = (e[0]->color + e[1]->color) * 0.5f;
        Eigen::Map<Eigen::Vector3f>(line.color)  = color;

        data.push_back(line);
    }
}
