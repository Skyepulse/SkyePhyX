#include "math.hpp"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

// PAPERS:
// Analytic Eigensystems for Isotropic Distortion Energies (Smith, de Goes, Kim 2018)
// Trust-Region Eigenvalue Filtering for Projected Newton (Chen, et al. 2024)
// FEM Simulation of 3D Deformable Solids: A practitioner’s guide to theory, discretization and model reduction (Sifakis, Barbic 2012)

//================================//
namespace NeoHookeanMath
{
    //================================//
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
    float ComputeEnergyDensity(const Eigen::Matrix3f& F, float J, float mu, float lambda)
    {
        // Ψ = (μ/2)(I₁ − 3) − μ·ln(J) + (λ/2)(ln J)²
        float I1 = F.squaredNorm();
        float logJ = std::log(J);
        return (mu * 0.5f) * (I1 - 3.f) - mu * logJ + (lambda * 0.5f) * logJ * logJ;
    }

    //================================//
    Eigen::Matrix3f ComputeFirstPiolaKirchhoff(const Eigen::Matrix3f& F, float J, float mu, float lambda)
    {
        // P = μ(F − F⁻ᵀ) + λ·ln(J)·F⁻ᵀ
        // P = μF + (λ·ln(J) − μ)F-ᵀ
        // P = μF + (λ·ln(J) − μ)/J · JF⁻ᵀ
        // P = μF + (λ·ln(J) − μ)/J · cof(F) with cof(F) = JF⁻ᵀ
        // Source:
        // https://viterbi-web.usc.edu/~jbarbic/femdefo/sifakis-courseNotes-TheoryAndDiscretization.pdf
        // 3.6
        float logJ = std::log(J);
        float scalar = (lambda * logJ - mu) / J;

        Eigen::Matrix3f cofF = Cof(F);
        return mu * F + scalar * cofF;
    }

    //================================//
    // Reference: Smith, de Goes, Kim 2018
    // "Analytic Eigensystems for Isotropic Distortion Energies"
    HessianDecomposition ComputeEnergyHessian(const Eigen::Vector3f& sigma, float J, float mu, float lambda)
    {
        HessianDecomposition result{};

        const float s0 = sigma(0), s1 = sigma(1), s2 = sigma(2);

        // Ψ = (μ/2)(I₁−3) − μ·ln(J) + (λ/2)(ln J)²
        // ∂Ψ/∂σᵢ = μσᵢ + (λ·ln(J)−μ)/σᵢ  →  λ_twist_{ab} = (aₐ+a_b)/(σₐ+σ_b) = μ + σₖ·dΨ/dJ
        //                                      λ_flip_{ab}  = (aₐ−a_b)/(σₐ−σ_b) = μ − σₖ·dΨ/dJ

        float logJ    = std::log(J);
        float dPhi_dJ = (lambda * logJ - mu) / J;

        result.eigenValues[3] = mu + dPhi_dJ * s2;
        result.eigenValues[4] = mu + dPhi_dJ * s1;
        result.eigenValues[5] = mu + dPhi_dJ * s0;

        result.eigenValues[6] = mu - dPhi_dJ * s2;
        result.eigenValues[7] = mu - dPhi_dJ * s1;
        result.eigenValues[8] = mu - dPhi_dJ * s0;

        // Scaling matrix: ∂²Ψ/∂σᵢ∂σⱼ (derived from ∂Ψ/∂σᵢ = μσᵢ + (λ·ln(J)−μ)/σᵢ)
        // a_ii = μ + (μ + λ(1 − ln J)) / σᵢ²
        // a_ij = λσₖ / J   (i≠j, k = remaining index)
        float diagExtra = mu + lambda * (1.0f - logJ);

        Eigen::Matrix3f a;
        a(0,0) = mu + diagExtra / (s0 * s0);
        a(1,1) = mu + diagExtra / (s1 * s1);
        a(2,2) = mu + diagExtra / (s2 * s2);

        a(0,1) = a(1,0) = lambda * s2 / J;
        a(0,2) = a(2,0) = lambda * s1 / J;
        a(1,2) = a(2,1) = lambda * s0 / J;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(a);
        Eigen::Vector3f scalingEigenValues = es.eigenvalues();
        Eigen::Matrix3f scalingEigenVectors = es.eigenvectors();

        result.eigenValues[0] = scalingEigenValues(0);
        result.eigenValues[1] = scalingEigenValues(1);
        result.eigenValues[2] = scalingEigenValues(2);

        result.scalingVectors[0] = scalingEigenVectors.col(0);
        result.scalingVectors[1] = scalingEigenVectors.col(1);
        result.scalingVectors[2] = scalingEigenVectors.col(2);

        return result;
    }

    //================================//
    void ProjectEigenvalues(float* eigenvalues, int count, EigenProjectionMode mode, float trustRegionRho, float threshold, float epsilon)
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

    //================================//
    bool ComputeExactVertexHessian(const Eigen::Matrix3f& F, float J, const Eigen::Vector3f& gradN,
                                    float mu, float lambda, float restVolume, Eigen::Matrix3f& out)
    {
        // H_v = V0 * [ mu*||gradN||^2 * I  +  alpha * (cofF*gradN)(cofF*gradN)^T ]
        // alpha = (lambda*(1 - ln J) + mu) / J^2
        // Singular iff: mu*||gradN||^2 + alpha*||cofF*gradN||^2 == 0

        Eigen::Matrix3f cofF = Cof(F);

        float logJ  = std::log(J);
        float alpha = (lambda * (1.0f - logJ) + mu) / (J * J);

        Eigen::Vector3f GN = cofF * gradN;


        float diag = mu * gradN.squaredNorm();
        float critEigenvalue = diag + alpha * GN.squaredNorm();

        if (diag < 1e-12f || std::abs(critEigenvalue) < 1e-8f)
            return false;

        out = restVolume * (diag * Eigen::Matrix3f::Identity() + alpha * GN * GN.transpose());
        return true;
    }
};

//================================//
namespace STVKMath
{
    //================================//
    SVDDecomposition svd(const F32& F)
    {
        SVDDecomposition result{};

        Eigen::JacobiSVD<F32> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        result.U = svd.matrixU();
        result.S = svd.singularValues();
        result.V = svd.matrixV();

        if (result.V.determinant() < 0.f)
        {
            result.V.col(1) *= -1.f;
            result.S(1)     *= -1.f;
        }

        return result;
    }

    //================================//
    F32 DeformationGradient(
        const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
        const Eigen::Matrix2f& DmInv2D)
    {
        F32 Ds;
        Ds.col(0) = p1 - p0;
        Ds.col(1) = p2 - p0;
        return Ds * DmInv2D;
    }

    //================================//
    float ComputeEnergyDensity(const F32& F, float mu, float lambda)
    {
        // Ψ = μ||E||² + (λ/2)(tr E)²
        Eigen::Matrix2f E = 0.5f * (F.transpose() * F - Eigen::Matrix2f::Identity());
        float trE = E.trace();
        return mu * E.squaredNorm() + 0.5f * lambda * trE * trE;
    }

    //================================//
    // S (second piola-kirchhoff) = 2μE + λ(tr E)I
    // E = (FᵀF − I)/2
    // P = F * S
    // Source:
    // https://viterbi-web.usc.edu/~jbarbic/femdefo/sifakis-courseNotes-TheoryAndDiscretization.pdf
    // 3.3
    F32 ComputeFirstPiolaKirchhoff(const F32& F, float mu, float lambda)
    {
        Eigen::Matrix2f E = 0.5f * (F.transpose() * F - Eigen::Matrix2f::Identity());
        float trE = E.trace();
        return F * (2.f * mu * E + lambda * trE * Eigen::Matrix2f::Identity());
    }

    //================================//
    // Reference: Smith, de Goes, Kim 2018
    // "Analytic Eigensystems for Isotropic Distortion Energies"
    HessianDecomposition ComputeEnergyHessian(const Eigen::Vector2f& sigma, float mu, float lambda)
    {
        HessianDecomposition result{};

        const float s0 = sigma(0), s1 = sigma(1);
        const float traceS = s0*s0 + s1*s1;

        const float dPsi_ds0 = s0 * (mu * (s0*s0 - 1.f) + 0.5f * lambda * (traceS - 2.f));
        const float dPsi_ds1 = s1 * (mu * (s1*s1 - 1.f) + 0.5f * lambda * (traceS - 2.f));

        // Twist (antisymmetric mode): eigenvalue = (dΨ/dσ0 + dΨ/dσ1) / (σ0 + σ1)
        if (s0 + s1 > 1e-8f)
            result.eigenValues[2] = (dPsi_ds0 + dPsi_ds1) / (s0 + s1);
        else
            result.eigenValues[2] = -(mu + lambda);

        // Flip (symmetric mode): eigenvalue = (dΨ/dσ0 - dΨ/dσ1) / (σ0 - σ1)
        if (std::abs(s0 - s1) > 1e-8f)
            result.eigenValues[3] = (dPsi_ds0 - dPsi_ds1) / (s0 - s1);
        else
            result.eigenValues[3] = (3.f*mu + 2.f*lambda) * s0*s0 - mu - lambda;

        // Note to myself: this is a 2x2 eigenProblem instead
        // of the 3x3 neo hookean case.
        Eigen::Matrix2f A;
        A(0,0) = 3.f*mu*s0*s0 + 0.5f*lambda*(3.f*s0*s0 + s1*s1) - mu - lambda;
        A(1,1) = 3.f*mu*s1*s1 + 0.5f*lambda*(3.f*s1*s1 + s0*s0) - mu - lambda;
        A(0,1) = A(1,0) = lambda * s0 * s1;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(A);
        result.eigenValues[0] = es.eigenvalues()(0);
        result.eigenValues[1] = es.eigenvalues()(1);
        result.scalingVectors[0] = es.eigenvectors().col(0);
        result.scalingVectors[1] = es.eigenvectors().col(1);

        return result;
    }

    //================================//
    Eigen::Matrix3f ReconstructVertexHessian(
        const SVDDecomposition& svd, const HessianDecomposition& hessDecomp,
        const float* projectedEigenValues,
        const Eigen::Vector2f& gradN_mat,
        float restArea)
    {
        Eigen::Vector3f u[2];
        float vdotg[2];
        for (int i = 0; i < 2; ++i)
        {
            u[i]    = svd.U.col(i);
            vdotg[i] = svd.V.col(i).dot(gradN_mat);
        }

        Eigen::Matrix3f hessianVertex = Eigen::Matrix3f::Zero();
        auto addOuter = [&](float lam, const Eigen::Vector3f& w)
        {
            hessianVertex.noalias() += lam * (w * w.transpose());
        };

        for (int k = 0; k < 2; k++)
        {
            const Eigen::Vector2f& c = hessDecomp.scalingVectors[k];
            Eigen::Vector3f w = c(0)*vdotg[0]*u[0] + c(1)*vdotg[1]*u[1];
            addOuter(projectedEigenValues[k], w);
        }

        static constexpr float INV_SQRT2 = 0.70710678118f;
        addOuter(projectedEigenValues[2], INV_SQRT2 * (vdotg[1]*u[0] - vdotg[0]*u[1]));
        addOuter(projectedEigenValues[3], INV_SQRT2 * (vdotg[1]*u[0] + vdotg[0]*u[1]));

        return hessianVertex * restArea;
    }

    //================================//
    bool ComputeExactVertexHessian(const F32& F, const Eigen::Vector2f& gradN,
                                    float mu, float lambda, float restArea, Eigen::Matrix3f& out)
    {
        // H_v = V0 * [ (g^T S g) I3  +  mu*||g||^2 * (FF^T)  +  (mu+lambda) * (Fg)(Fg)^T ]
        Eigen::Matrix2f E = 0.5f * (F.transpose() * F - Eigen::Matrix2f::Identity());
        float trE = E.trace();
        Eigen::Matrix2f S = 2.0f * mu * E + lambda * trE * Eigen::Matrix2f::Identity();

        float gSg    = gradN.dot(S * gradN);
        float gNorm2 = gradN.squaredNorm();
        Eigen::Vector3f Fg = F * gradN;

        out = restArea * (gSg * Eigen::Matrix3f::Identity()
                        + mu * gNorm2 * (F * F.transpose())
                        + (mu + lambda) * (Fg * Fg.transpose()));

        return std::abs(out.determinant()) > 1e-10f;
    }

} // STVK MATH NAMESPACE