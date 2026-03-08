#include "energy.hpp"
using namespace NeoHookeanMath;

//================================//
NeoHookeanFEM::NeoHookeanFEM(Solver* solver,
                  Mesh* body0, Mesh* body1, Mesh* body2, Mesh* body3,
                  float youngsModulus, float poissonRatio)
    : Energy(solver, {body0, body1, body2, body3}), body0(body0), body1(body1), body2(body2), body3(body3), youngsModulus(youngsModulus), poissonRatio(poissonRatio)
{
    lameMu = youngsModulus / (2.f * (1.f + poissonRatio));
    lameLambda = youngsModulus * poissonRatio / ((1.f + poissonRatio) * (1.f - 2.f * poissonRatio));
    a = 1.0f + lameMu / lameLambda;

    Eigen::Vector3f p0 = body0->transform.GetPosition();
    Eigen::Vector3f p1 = body1->transform.GetPosition();
    Eigen::Vector3f p2 = body2->transform.GetPosition();
    Eigen::Vector3f p3 = body3->transform.GetPosition();

    Eigen::Matrix3f Dm;
    Dm.col(0) = p1 - p0;
    Dm.col(1) = p2 - p0;
    Dm.col(2) = p3 - p0;
    this->Dm = Dm;
    DmInv = Dm.inverse();

    restVolume = std::abs(Dm.determinant()) / 6.f; // Tetrahedron
    if (restVolume < 1e-8f)
    {
       // MMMm not sure how to handle this for now...
    }

    // gradN0 + gradN1 + gradN2 + gradN3 = 0
    Eigen::Matrix3f DmInvT = DmInv.transpose();
    gradN1 = DmInvT.col(0);
    gradN2 = DmInvT.col(1);
    gradN3 = DmInvT.col(2);
    gradN0 = -gradN1 - gradN2 - gradN3;
}

//================================//
void NeoHookeanFEM::ComputeEnergyTerms(Mesh* mesh, EigenProjectionMode projectionMode, float trustRegionRho)
{
    Eigen::Vector3f p0 = body0->transform.GetPosition();
    Eigen::Vector3f p1 = body1->transform.GetPosition();
    Eigen::Vector3f p2 = body2->transform.GetPosition();
    Eigen::Vector3f p3 = body3->transform.GetPosition();

    Eigen::Matrix3f F = DeformationGradient(p0, p1, p2, p3, DmInv);
    float J = F.determinant();

    cachedEnergy = restVolume * ComputeEnergyDensity(F, J, lameMu, lameLambda, a);

    Eigen::Vector3f gradN = Eigen::Vector3f::Zero();
    if (mesh == body0) gradN = gradN0;
    else if (mesh == body1) gradN = gradN1;
    else if (mesh == body2) gradN = gradN2;
    else if (mesh == body3) gradN = gradN3;

    if (J <= 1e-6f)
    {
        HandleInvertedElement(mesh, F, J, gradN);
        return;
    }

    // grad = P * V0 * gradN
    Eigen::Matrix3f P = ComputeFirstPiolaKirchhoff(F, J, lameMu, lameLambda, a);
    Eigen::Vector3f gradient = P * gradN * restVolume;

    grad.setZero(); // NO rotational component.
    grad.head<3>() = gradient;

    SVDDecomposition SVD = svd(F);
    HessianDecomposition hessDecomp = ComputeEnergyHessian(SVD.S, J, lameMu, lameLambda, a);

    float projectedEigenValues[9];
    for (int i = 0; i < 9; ++i)
        projectedEigenValues[i] = hessDecomp.eigenValues[i];
    ProjectEigenvalues(projectedEigenValues, 9, projectionMode, trustRegionRho, trustRegionThreshold);

    Eigen::Matrix3f HV = ReconstructVertexHessian(SVD, hessDecomp, projectedEigenValues, gradN, restVolume);

    hess.setZero();
    hess.block<3, 3>(0, 0) = HV;
}

//================================//
void NeoHookeanFEM::HandleInvertedElement(Mesh* mesh, const Eigen::Matrix3f& F, float J, const Eigen::Vector3f& gradNi)
{
    float stiffPenalty = lameMu + lameLambda;

    Eigen::Matrix3f cofF;
    cofF(0,0) =  (F(1,1)*F(2,2) - F(1,2)*F(2,1));
    cofF(0,1) = -(F(1,0)*F(2,2) - F(1,2)*F(2,0));
    cofF(0,2) =  (F(1,0)*F(2,1) - F(1,1)*F(2,0));
    cofF(1,0) = -(F(0,1)*F(2,2) - F(0,2)*F(2,1));
    cofF(1,1) =  (F(0,0)*F(2,2) - F(0,2)*F(2,0));
    cofF(1,2) = -(F(0,0)*F(2,1) - F(0,1)*F(2,0));
    cofF(2,0) =  (F(0,1)*F(1,2) - F(0,2)*F(1,1));
    cofF(2,1) = -(F(0,0)*F(1,2) - F(0,2)*F(1,0));
    cofF(2,2) =  (F(0,0)*F(1,1) - F(0,1)*F(1,0));

    Eigen::Vector3f dJdxi = cofF * gradNi;

    float penaltyMag = stiffPenalty * (1e-6f - J);

    grad.setZero();
    grad.head<3>() = -restVolume * penaltyMag * dJdxi;

    hess.setZero();
    hess.block<3,3>(0,0) = restVolume * stiffPenalty * Eigen::Matrix3f::Identity();
}

//================================//
void NeoHookeanFEM::AddLineData(std::vector<GPULineData>& data) const
{
    std::vector<std::array<Mesh*, 2>> edges = {
        {body0, body1}, {body0, body2}, {body0, body3},
        {body1, body2}, {body1, body3},
        {body2, body3}
    };

    for (auto& e : edges)
    {
        Eigen::Vector3f pA = e[0]->transform.GetPosition();
        Eigen::Vector3f pB = e[1]->transform.GetPosition();

        GPULineData line;
        Eigen::Map<Eigen::Vector3f>(line.start) = pA;
        Eigen::Map<Eigen::Vector3f>(line.end) = pB;
        Eigen::Vector3f color = (e[0]->color + e[1]->color) * 0.5f;
        Eigen::Map<Eigen::Vector3f>(line.color) = color;

        data.push_back(line);
    }
}