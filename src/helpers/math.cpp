#include "math.hpp"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

// PAPERS:
// Analytic Eigensystems for Isotropic Distortion Energies (Smith, de Goes, Kim 2018)
// Trust-Region Eigenvalue Filtering for Projected Newton (Chen, et al. 2024)

//================================//
namespace NeoHookeanMath
{
    //================================//
    SVDDecomposition svd(const Eigen::Matrix3f& F)
    {
        SVDDecomposition result{};

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        result.U = svd.matrixU();
        result.S = svd.singularValues();
        result.V = svd.matrixV();

        // Special cases
        if (result.U.determinant() < 0)
        {
            result.U.col(2) *= -1.f;
            result.S(2) *= -1.f;
        }
        if (result.V.determinant() < 0)
        {
            result.V.col(2) *= -1.f;
            result.S(2) *= -1.f;
        }

        return result;
    }

    //================================//
    Eigen::Matrix3f DeformationGradient(
            const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
            const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,
            const Eigen::Matrix3f& DmInv)
    {
        Eigen::Matrix3f Ds;
        Ds.col(0) = p1 - p0;
        Ds.col(1) = p2 - p0;
        Ds.col(2) = p3 - p0;

        return Ds * DmInv;
    }

    //================================//
    float ComputeEnergyDensity(const Eigen::Matrix3f& F, float J, float mu, float lambda, float alpha)
    {
        // I known = trace(F^T * F)
        float I1 = F.squaredNorm();

        // In 2D: Ψ = (μ/2)(I₁ − 2) + (λ/2)(J − α)²
        // In 3D: Ψ = (μ/2)(I₁ − 3) + (λ/2)(J − α)²
        return (mu * 0.5f) * (I1 - 3.f) + (lambda * 0.5f) * (J - alpha) * (J - alpha);
    }

    //================================//
    Eigen::Matrix3f ComputeFirstPiolaKirchhoff(const Eigen::Matrix3f& F, float J, float mu, float lambda, float alpha)
    {
        // dE/dx = dE/dF * dF/dx
        // Calculate ∂E/∂F (First Piola-Kirchhoff Stress Tensor, P)
        // P = ∂E/∂F = mu * F + lambda * (J - alpha) * J * F^-T
        // cof(F) = J * F^-T

        Eigen::Matrix3f cofF;
        cofF << (F(1,1)*F(2,2) - F(1,2)*F(2,1)),
                -(F(1,0)*F(2,2) - F(1,2)*F(2,0)),
                (F(1,0)*F(2,1) - F(1,1)*F(2,0)),

                -(F(0,1)*F(2,2) - F(0,2)*F(2,1)),
                (F(0,0)*F(2,2) - F(0,2)*F(2,0)),
                -(F(0,0)*F(2,1) - F(0,2)*F(2,0)),

                (F(0,1)*F(1,2) - F(0,2)*F(1,1)),
                -(F(0,0)*F(1,2) - F(0,2)*F(1,0)),
                (F(0,0)*F(1,1) - F(0,1)*F(1,0));

        return mu * F + lambda * (J - alpha) * cofF;
    }

    //================================//
    // Reference: Smith, de Goes, Kim 2018
    // "Analytic Eigensystems for Isotropic Distortion Energies"
    HessianDecomposition ComputeEnergyHessian(const Eigen::Vector3f& sigma, float J, float mu, float lambda, float alpha)
    {
        HessianDecomposition result{};

        const float s0 = sigma(0), s1 = sigma(1), s2 = sigma(2);
        float commonFactor = (J - alpha);

        // if f = vec(F), F in R^3x3 then ∂2Ψ/∂f2 in R^9x9. We have 9 eigenvalues.
        // We analyze the F space hessian NOT THE VERTEX-SPACE (12 eigenvalues).

        // The paper gives the eigenvalues decomposition, for ANY isotropic 3D energy.
        // We therefore have:
        // λ_twist_i = 2/(σⱼ+σₖ) · ∂Ψ/∂I₁  +  2 · ∂Ψ/∂I₂  +  σᵢ · ∂Ψ/∂I₃
        // λ_flip_i  =                        2 · ∂Ψ/∂I₂  −  σᵢ · ∂Ψ/∂I₃

        // In Neo Hookean (3D), we have:
        // Ψ = μ/2)(I₂ − 3) + (λ/2)(J − α)²
        // ∂Ψ/∂I₁ = 0
        // ∂Ψ/∂I₂ = μ/2
        // ∂Ψ/∂I₃ = λ(J − α)

        // The 6 twist and flip eigenvalues are closed form. The 3 scaling
        // require a small 3x3 eigenproblem solve.

        result.eigenValues[3] = mu + lambda * commonFactor * s2;
        result.eigenValues[6] = mu - lambda * commonFactor * s2;

        result.eigenValues[4] = mu + lambda * commonFactor * s1;
        result.eigenValues[7] = mu - lambda * commonFactor * s1;

        result.eigenValues[5] = mu + lambda * commonFactor * s0;
        result.eigenValues[8] = mu - lambda * commonFactor * s0;

        // Small Eigenproblem from appendix D "SCALING MODE MATRIX"
        // a_ij = σₖ(∂Ψ/∂I₃) + (∂²Ψ/∂I₁²) + 4σⱼσᵢ(∂²Ψ/∂I₂²) + σₖI₃(∂²Ψ/∂I₃²) + 2(I1 - σₖ)(∂²Ψ/∂I₁∂I₂) + 2σₖ(I2 - σₖ^2)(∂²Ψ/∂I₂∂I₃) + σₖ(I1 - σₖ)(∂²Ψ/∂I₁∂I₃)
        // a_ii = 2(∂Ψ/∂I₂) + (∂²Ψ/∂I₁²) + 4σᵢ^2(∂²Ψ/∂I₂²) + σⱼ^2σₖ^2(∂²Ψ/∂I₃²) + 4σᵢ(∂²Ψ/∂I₁∂I₂) + 4I₃(∂²Ψ/∂I₂∂I₃) + 2(I₃/σᵢ)(∂²Ψ/∂I₁∂I₃)
        // σᵢσⱼ = I₃/σₖ
        // ∂Ψ/∂I₁ = 0, ∂Ψ/∂I₂ = μ/2, ∂Ψ/∂I₃ = λ(J − α)
        // ∂²Ψ/∂I₁² = 0, ∂²Ψ/∂I₂² = 0, ∂²Ψ/∂I₃² = λ, ∂²Ψ/∂I₁∂I₂ = 0, ∂²Ψ/∂I₁∂I₃ = 0, ∂²Ψ/∂I₂∂I₃ = 0

        Eigen::Matrix3f a;
        a(0,0) = mu + lambda * (s1*s2) * (s1*s2);
        a(1,1) = mu + lambda * (s0*s2) * (s0*s2);
        a(2,2) = mu + lambda * (s0*s1) * (s0*s1);

        float twoJma = 2.f * J - alpha;
        a(0,1) = a(1,0) = lambda * s2 * twoJma;
        a(0,2) = a(2,0) = lambda * s1 * twoJma;
        a(1,2) = a(2,1) = lambda * s0 * twoJma;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(a);
        Eigen::Vector3f scalingEigenValues = es.eigenvalues();
        Eigen::Vector3f scalingEigenVectors = es.eigenvectors();

        result.eigenValues[0] = scalingEigenValues(0);
        result.eigenValues[1] = scalingEigenValues(1);
        result.eigenValues[2] = scalingEigenValues(2);

        result.scalingVectors[0] = scalingEigenVectors.col(0);
        result.scalingVectors[1] = scalingEigenVectors.col(1);
        result.scalingVectors[2] = scalingEigenVectors.col(2);

        return result;
    }

    //================================//
    void ProjectEigenvalues(float* eigenvalues, int count, EigenProjectionMode mode, float trustRegionRho, float threshold = 0.01f, float epsilon = 1e-6f)
    {
        bool useAbsolute = false;

        switch(mode)
        {
            case EigenProjectionMode::CLAMP:
                for (int i = 0; i < count; ++i)
                    eigenvalues[i] = std::max(epsilon, eigenvalues[i]);
                return;

            case EigenProjectionMode::ABSOLUTE:
                useAbsolute = true;
                break;

            case EigenProjectionMode::ADAPTIVE:
                useAbsolute = (std::abs(trustRegionRho - 1.f) > threshold);
                if (!useAbsolute)
                {
                    for (int i = 0; i < count; ++i)
                        eigenvalues[i] = std::max(epsilon, eigenvalues[i]);
                    return;
                }
                break;
        }

        if (useAbsolute)
        {
            for (int i = 0; i < count; ++i)
            {
                eigenvalues[i] = std::abs(eigenvalues[i]);
                if (eigenvalues[i] < epsilon) eigenvalues[i] = epsilon;
            }
        }
    }

    //================================//
    Eigen::Matrix3f ReconstructVertexHessian(const SVDDecomposition& svd, const HessianDecomposition& hessDecomp, const float* projectedEigenValues, const Eigen::Vector3f& gradN, float restVolume)
    {
        const Eigen::Matrix3f& U = svd.U;
        const Eigen::Matrix3f& V = svd.V;

        Eigen::Vector3f u[3], v[3];
        for (int i = 0; i < 3; ++i)
        {
            u[i] = U.col(i);
            v[i] = V.col(i);
        }

        float vdotg[3];
        for (int i = 0; i < 3; ++i)
            vdotg[i] = v[i].dot(gradN);

        Eigen::Matrix3f hessianVertex = Eigen::Matrix3f::Zero();
        auto addOuter = [&](float lam, const Eigen::Vector3f& w)
        {
            hessianVertex.noalias() += lam * (w * w.transpose());
        };

        // Scaling
        for (int k = 0; k < 3; k++)
        {
            const Eigen::Vector3f& c = hessDecomp.scalingVectors[k];
            Eigen::Vector3f w = c(0) * vdotg[0] * u[0] + c(1) * vdotg[1] * u[1] + c(2) * vdotg[2] * u[2];
            addOuter(projectedEigenValues[k], w);
        }

        // Twist, antisymmetric 
        // Pair (i,j): Q_twist = (uᵢvⱼᵀ − uⱼvᵢᵀ)/√2
        static const int pairs[3][2] = {{0,1}, {0,2}, {1,2}};
        static constexpr float INV_SQRT2 = 0.70710678118f;

        for (int p = 0; p < 3; p++)
        {
            int i = pairs[p][0];
            int j = pairs[p][1];

            Eigen::Vector3f w = INV_SQRT2 * (vdotg[j] * u[i] - vdotg[i] * u[j]);
            addOuter(projectedEigenValues[3 + p], w);
        }

        // Flip, symmetric
        // Pair (i,j): Q_flip = (uᵢvⱼᵀ + uⱼvᵢᵀ)/√2
        for (int p = 0; p < 3; p++)
        {
            int i = pairs[p][0];
            int j = pairs[p][1];

            Eigen::Vector3f w = INV_SQRT2 * (vdotg[j] * u[i] + vdotg[i] * u[j]);
            addOuter(projectedEigenValues[6 + p], w);
        }

        return hessianVertex * restVolume;
    }
};