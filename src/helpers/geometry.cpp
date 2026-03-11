#include "geometry.hpp"
#include "../physics/solver.hpp"

//================================//
Mesh::Mesh(Solver* solver, ModelType modelType, const Eigen::Vector3f& color) : modelType(modelType), color(color), solver(solver)
{
    return;
}

//================================//
Mesh::~Mesh()   
{
    auto forcesCopy = forces;
    for (Force* f : forcesCopy)
    {
        solver->RemoveForce(f);
    }

    auto energiesCopy = energies;
    for (Energy* e : energiesCopy)
    {
        solver->RemoveEnergy(e);
    }
};

//================================//
namespace GeometryHelpers
{
    //================================//
    void makeTetVolume(Solver* solver, float ox, float oy, float oz, int nx, int ny, int nz, float cellSize, float E, float nu, const Eigen::Vector3f& color, float particleMass, float friction, bool pinMinX)
    {
        std::vector<Mesh*> parts(nx * ny * nz);

        auto idx = [&](int i, int j, int k)
        {
            return i + j * nx + k * nx * ny;
        };

        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < ny; j++)
            {
                for (int i = 0; i < nx; i++)
                {
                    Eigen::Vector3f pos(ox + i * cellSize, oy + j * cellSize, oz + k * cellSize);
                    bool isStatic = pinMinX && (i == 0);
                    Mesh* p = solver->AddParticle(
                        particleMass, friction,
                        pos, Eigen::Vector3f::Zero(),
                        isStatic, color
                    );
                    p->isParticle = true;
                    p->name = "V_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
                    parts[idx(i, j, k)] = p;
                }
            }
        }

        for (int ci = 0; ci < nx - 1; ci++)
        {
            for (int cj = 0; cj < ny - 1; cj++)
            {
                for (int ck = 0; ck < nz - 1; ck++)
                {
                    Mesh* v[8] = {
                        parts[idx(ci,   cj,   ck  )],  // v0
                        parts[idx(ci+1, cj,   ck  )],  // v1
                        parts[idx(ci,   cj+1, ck  )],  // v2
                        parts[idx(ci+1, cj+1, ck  )],  // v3
                        parts[idx(ci,   cj,   ck+1)],  // v4
                        parts[idx(ci+1, cj,   ck+1)],  // v5
                        parts[idx(ci,   cj+1, ck+1)],  // v6
                        parts[idx(ci+1, cj+1, ck+1)],  // v7
                    };

                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[0], v[1], v[2], v[4], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[3], v[2], v[1], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[5], v[1], v[4], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[6], v[4], v[2], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[1], v[2], v[4], v[7], E, nu
                    ));
                }
            }
        }
    }

    //================================//
    // UV-sphere fan tessellation.
    // Surface: nLon=6N longitude sectors × nLat=3N latitude rings + 2 poles.
    // Innermost shell fans each surface triangle to the centre (like a 2D circle fan).
    // Each additional shell stacks a triangular prism (3 tets) between adjacent shells.
    //
    // Approximate counts:
    //   res=1 →  21 particles,   36 tets
    //   res=2 → 149 particles,  576 tets
    //   res=3 → 493 particles, 2268 tets
    void makeTetSphere(Solver* solver, float cx, float cy, float cz, float radius, float res, float E, float nu, const Eigen::Vector3f& color, float mass, float friction)
    {
        int N      = std::max(1, (int)std::roundf(res));
        int nLon   = 6 * N;   // longitude sectors  (6 gives hexagonal look at N=1)
        int nLat   = 3 * N;   // latitude body rings (poles are separate)
        int nShells = N;       // concentric radial shells

        // ── Vertex storage ──────────────────────────────────────────────────────
        // [0] = centre
        // Per shell s: sBase(s)+0       = south pole
        //              sBase(s)+1       = north pole
        //              sBase(s)+2+j*nLon+i = body ring j (j=0 near north), sector i
        int vps    = 2 + nLat * nLon;
        int nVerts = 1 + nShells * vps;
        std::vector<Mesh*> pts(nVerts, nullptr);

        auto sBase  = [&](int s)            { return 1 + s * vps; };
        auto sSouth = [&](int s)            { return sBase(s); };
        auto sNorth = [&](int s)            { return sBase(s) + 1; };
        auto sV     = [&](int s, int j, int i) {
            return sBase(s) + 2 + j * nLon + ((i % nLon + nLon) % nLon);
        };

        auto makeP = [&](float x, float y, float z) -> Mesh* {
            Mesh* p = solver->AddParticle(mass, friction,
                Eigen::Vector3f(x, y, z), Eigen::Vector3f::Zero(), false, color);
            p->isParticle = true;
            return p;
        };

        // Centre
        pts[0] = makeP(cx, cy, cz);

        // Shells: radius grows linearly from radius/nShells to radius
        for (int s = 0; s < nShells; s++)
        {
            float r = radius * (s + 1) / (float)nShells;
            pts[sSouth(s)] = makeP(cx, cy - r, cz);
            pts[sNorth(s)] = makeP(cx, cy + r, cz);
            for (int j = 0; j < nLat; j++)
            {
                // theta: polar angle from north pole (0 = north, π = south)
                float theta = (float)M_PI * (j + 1) / (float)(nLat + 1);
                float sinT  = std::sin(theta);
                float cosT  = std::cos(theta);
                for (int i = 0; i < nLon; i++)
                {
                    float phi = 2.f * (float)M_PI * i / (float)nLon;
                    pts[sV(s, j, i)] = makeP(
                        cx + r * sinT * std::cos(phi),
                        cy + r * cosT,
                        cz + r * sinT * std::sin(phi)
                    );
                }
            }
        }

        // ── Tet helpers ─────────────────────────────────────────────────────────
        // Compute det(Dm) at spawn time and swap c↔d to guarantee positive volume.
        auto addTet = [&](Mesh* a, Mesh* b, Mesh* c, Mesh* d)
        {
            Eigen::Matrix3f Dm;
            Dm.col(0) = b->transform.GetPosition() - a->transform.GetPosition();
            Dm.col(1) = c->transform.GetPosition() - a->transform.GetPosition();
            Dm.col(2) = d->transform.GetPosition() - a->transform.GetPosition();
            if (Dm.determinant() < 0.f) std::swap(c, d);
            solver->AddEnergy(std::make_unique<NeoHookeanFEM>(solver, a, b, c, d, E, nu));
        };

        // Triangular prism (inner face P0P1P2, outer face Q0Q1Q2) → 3 tets.
        // Schöberl decomposition; each tet gets orientation-checked by addTet.
        auto addPrism = [&](Mesh* P0, Mesh* P1, Mesh* P2,
                            Mesh* Q0, Mesh* Q1, Mesh* Q2)
        {
            addTet(P0, P1, P2, Q2);
            addTet(P0, Q1, P1, Q2);
            addTet(P0, Q0, Q1, Q2);
        };

        // ── Tet generation ───────────────────────────────────────────────────────
        for (int s = 0; s < nShells; s++)
        {
            bool fanToCenter = (s == 0);
            int  sIn         = s - 1;

            for (int i = 0; i < nLon; i++)
            {
                int i1 = (i + 1) % nLon;

                // North cap: north_pole + ring j=0
                if (fanToCenter)
                    addTet(pts[0], pts[sNorth(s)], pts[sV(s,0,i)], pts[sV(s,0,i1)]);
                else
                    addPrism(pts[sNorth(sIn)], pts[sV(sIn,0,i)], pts[sV(sIn,0,i1)],
                             pts[sNorth(s  )], pts[sV(s,  0,i)], pts[sV(s,  0,i1)]);

                // South cap: south_pole + ring j=nLat-1
                int jS = nLat - 1;
                if (fanToCenter)
                    addTet(pts[0], pts[sSouth(s)], pts[sV(s,jS,i)], pts[sV(s,jS,i1)]);
                else
                    addPrism(pts[sSouth(sIn)], pts[sV(sIn,jS,i)], pts[sV(sIn,jS,i1)],
                             pts[sSouth(s  )], pts[sV(s,  jS,i)], pts[sV(s,  jS,i1)]);
            }

            // Body quads: ring j → ring j+1, split into 2 triangles each
            for (int j = 0; j < nLat - 1; j++)
            {
                for (int i = 0; i < nLon; i++)
                {
                    int i1 = (i + 1) % nLon;
                    if (fanToCenter)
                    {
                        addTet(pts[0], pts[sV(s,j,i)],  pts[sV(s,j,i1)],  pts[sV(s,j+1,i1)]);
                        addTet(pts[0], pts[sV(s,j,i)],  pts[sV(s,j+1,i1)],pts[sV(s,j+1,i )]);
                    }
                    else
                    {
                        addPrism(pts[sV(sIn,j,  i )], pts[sV(sIn,j,  i1)], pts[sV(sIn,j+1,i1)],
                                 pts[sV(s,  j,  i )], pts[sV(s,  j,  i1)], pts[sV(s,  j+1,i1)]);
                        addPrism(pts[sV(sIn,j,  i )], pts[sV(sIn,j+1,i1)], pts[sV(sIn,j+1,i )],
                                 pts[sV(s,  j,  i )], pts[sV(s,  j+1,i1)], pts[sV(s,  j+1,i )]);
                    }
                }
            }
        }
    }

    //================================//
    void makeTet(Solver* solver, float cx, float cy, float cz, float edge, float E, float nu, const Eigen::Vector3f& color, float mass, float friction)
    {
        float a = edge;
        float inv_sqrt3  = 1.f / std::sqrt(3.f);
        float inv_sqrt6  = 1.f / std::sqrt(6.f);
        float sqrt_2_3   = std::sqrt(2.f / 3.f);

        Eigen::Vector3f pyramidVertices[4] = 
        {
            { 0.f,            0.f,              sqrt_2_3 * a },
            { -0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.f,           inv_sqrt3 * a,          -inv_sqrt6 * a },
        };

        Eigen::Vector3f centroid = (pyramidVertices[0] + pyramidVertices[1] + pyramidVertices[2] + pyramidVertices[3]) / 4.f;
        Eigen::Vector3f offset(cx, cy, cz);
        for (int i = 0; i < 4; ++i)
            pyramidVertices[i] += offset - centroid;

        Mesh* particles[4];
        for (int i = 0; i < 4; ++i)
        {
            particles[i] = solver->AddParticle(
                mass, friction,
                pyramidVertices[i], Eigen::Vector3f::Zero(),
                false, color
            );
            particles[i]->isParticle = true;
            particles[i]->name = "Tet_" + std::to_string((int)E) + "_" + std::to_string(i);
        }

        solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
            solver,
            particles[0], particles[1], particles[2], particles[3],
            E, nu
        ));
    };
}