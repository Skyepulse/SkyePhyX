#ifndef LEVELS_H
#define LEVELS_H

#include "physics/solver.hpp"
#include "helpers/camera.hpp"

using namespace GeometryHelpers;

//================================//
struct LevelParameters
{
    float E;
    float nu;
    float particleMass;
};

//================================//
static void DefaultScene(Solver* solver, Camera* camera, const LevelParameters& params)
{
    Mesh* ground = solver->AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Ground";

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void Pyramid(Solver* solver, Camera* camera, const LevelParameters& params)
{
    Mesh* ground = solver->AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Ground";

    Eigen::Vector3f commonScale = Eigen::Vector3f(2.0f, 1.f, 2.0f);

    int levels = 6;
    float blockWidth  = commonScale.x();
    float blockHeight = commonScale.y();
    float blockDepth  = commonScale.z();
    float spacing     = 0.3f;

    float groundTop = -9.5f;
    float dropOffset = 3.f;

    for (int row = 0; row < levels; row++)
    {
        int blocksPerSide = levels - row;
        float rowSpan = blocksPerSide * blockWidth + (blocksPerSide - 1) * spacing;
        float startX = -rowSpan * 0.5f + blockWidth * 0.5f;
        float startZ = -rowSpan * 0.5f + blockDepth * 0.5f;

        float y = groundTop + blockHeight * 0.5f + row * (blockHeight + spacing) + dropOffset;

        for (int ix = 0; ix < blocksPerSide; ix++)
        {
            for (int iz = 0; iz < blocksPerSide; iz++)
            {
                float x = startX + ix * (blockWidth + spacing);
                float z = startZ + iz * (blockDepth + spacing);

                Eigen::Vector3f pos(x, y, z);
                Eigen::Vector3f color(
                    0.3f + 0.7f * (float)row / (float)(levels - 1),
                    0.2f + 0.5f * (float)ix / (float)(blocksPerSide),
                    0.2f + 0.5f * (float)iz / (float)(blocksPerSide)
                );

                Mesh* block = solver->AddBody(
                    ModelType_Cube, 1.0f, 0.5f,
                    pos, commonScale,
                    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                    Quaternionf::Identity(),
                    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                    false, color
                );
                block->name = "Pyramid_l" + std::to_string(row) + "_" + std::to_string(ix) + "_" + std::to_string(iz);
            }
        }
    }

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void MassStack(Solver* solver, Camera* camera, const LevelParameters& params)
{
    Mesh* ground = solver->AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Ground";

    float groundTop = -9.5f;
    float dropOffset = 3.f;

    int numBlocks = 5;
    Eigen::Vector3f initialScale(1.f, 1.f, 1.f);

    float scaleMult = 1.5f;

    for (int i = 0; i < numBlocks; i++)
    {
        float y = groundTop + initialScale.y() * 0.5f + i * (initialScale.y() + 0.3f) + dropOffset;

        Mesh* block = solver->AddBody(
            ModelType_Cube, 1.0f, 1.0f,
            Eigen::Vector3f(0.0f, y, 0.0f), initialScale,
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Quaternionf::Identity(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            false,
            Eigen::Vector3f(1.f - (float)i / (float)numBlocks, 0.2f, (float)i / (float)numBlocks)
        );
        block->name = "MassStack_" + std::to_string(i);

        initialScale *= scaleMult;
    }

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void FrictionSlope(Solver* solver, Camera* camera, const LevelParameters& params)
{
    float slopeDegrees = 30.f;
    Eigen::Vector3f rotationPerAxis(slopeDegrees, 0.0f, 0.0f);
    Quaternionf slopeRotation = Eigen::AngleAxisf(rotationPerAxis.x() * M_PI / 180.0f, Eigen::Vector3f::UnitX()) *
                               Eigen::AngleAxisf(rotationPerAxis.y() * M_PI / 180.0f, Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(rotationPerAxis.z() * M_PI / 180.0f, Eigen::Vector3f::UnitZ());

    Mesh* ground = solver->AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(35.0f, 1.0f, 10.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), slopeRotation, Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Slope";

    // N blocks, from left to right inclined like the slope, with increasing friction
    int numBlocks = 10;
    float startFriction = 0.0f;
    float endFriction = 0.95f;

    float heightOffset = 1.f;

    for (int i = 0; i < numBlocks; i++)
    {
        float friction = startFriction + (endFriction - startFriction) * ((float)i / (float)(numBlocks - 1));

        Mesh* block = solver->AddBody(
            ModelType_Cube, 1.0f, friction,
            Eigen::Vector3f(-15.f + i * 3.f, heightOffset, 0.0f), Eigen::Vector3f(2.f, 1.f, 2.f),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            slopeRotation,
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            false,
            Eigen::Vector3f(1.f - (float)i / (float)numBlocks, 0.2f, (float)i / (float)numBlocks)
        );
        block->name = "FrictionSlope_" + std::to_string(i);
    }

    camera->SetPosition(Eigen::Vector3f(0.0f, -5.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void MassSprings(Solver* solver, Camera* camera, const LevelParameters& params)
{
    Eigen::Vector3f commonScale(1.f, 1.f, 1.f);

    float densityMultiplier = 2.5f;

    int N = 8;
    float spacing = 2.f;

    std::vector<Mesh*> blocks;

    //place them at 0y, 0z and line in x

    float startX = -((N - 1) * spacing) * 0.5f;

    for (int i = 0; i < N; i++)
    {
        Mesh* block = solver->AddBody(
            ModelType_Cube, 1.0f * powf(densityMultiplier, (float)i), 0.5f,
            Eigen::Vector3f(startX + i * spacing, 0.f, 0.0f), commonScale,
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Quaternionf::Identity(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            false,
            Eigen::Vector3f((float)i / (float)N, 0.2f, 1.f - (float)i / (float)N)
        );
        block->name = "MassSpring_" + std::to_string(i);
        blocks.push_back(block);
    }

    // Connect each block to a rope force, at its position
    for (auto block : blocks)
    {
        Eigen::Vector3f anchor = block->transform.GetPosition();
        Force* rope = new Spring(solver, nullptr, anchor, block, Eigen::Vector3f::Zero(), 1000.f, 3.f, true);
        solver->AddForce(std::unique_ptr<Force>(rope));
    }

    camera->SetPosition(Eigen::Vector3f(0.0f, 0.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void JointPlayground(Solver* solver, Camera* camera, const LevelParameters& params)
{
    // Pendulum
    Eigen::Vector3f commonScale(2.f, 0.7f, 0.7f);
    Vector6f pendulumAnchors = Vector6f(INFINITY, INFINITY, INFINITY, INFINITY, 0.f, 0.f);
    Eigen::Vector3f startAnchor(0.f, 5.f, 0.f);
    int numLinks = 5;
    
    // start them horizontally
    Mesh* lastBody = nullptr;
    Eigen::Vector3f relLeft(-0.5f, 0.f, 0.f);
    Eigen::Vector3f relRight(0.5f, 0.f, 0.f);
    Eigen::Vector3f spawnPos(startAnchor.x() + commonScale.x() / 2.f, startAnchor.y(), startAnchor.z());
    for (int i = 0; i < numLinks; i++)
    {
        Mesh* block = solver->AddBody(
            ModelType_Cube, 1.0f, 0.5f,
            spawnPos, commonScale,
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Quaternionf::Identity(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            false,
            Eigen::Vector3f((float)i / (float)numLinks, 1.f - (float)i / (float)numLinks, 0.2f)
        );
        block->name = "PendulumLink_" + std::to_string(i);

        Force* joint = new Joint(solver, lastBody, lastBody ? relRight : startAnchor, block, relLeft, pendulumAnchors);
        solver->AddForce(std::unique_ptr<Force>(joint));
        lastBody = block;
        spawnPos.x() += commonScale.x();
    }

    // bridge with two ropes per link
    startAnchor = Eigen::Vector3f(-10.f, 5.f, -5.f);
    commonScale = Eigen::Vector3f(2.f, 0.5f, 3.f);
    numLinks = 12;
    float bridgeSpacing = 1.0f;
    lastBody = nullptr;
    spawnPos = Eigen::Vector3f(startAnchor.x() + commonScale.x() / 2.f, startAnchor.y(), startAnchor.z());

    float zEdge      = 0.5f;
    float zEdgeWorld = 0.5f * commonScale.z();

    // left wall anchors: one bridgeSpacing to the left of block[0]'s left face
    Eigen::Vector3f leftAnchorWorld = startAnchor - Eigen::Vector3f(bridgeSpacing, 0.f, 0.f);

    std::vector<int> breakableRopes = {6};
    for (int i = 0; i < numLinks; i++)
    {
        Mesh* block = solver->AddBody(
            ModelType_Cube, 1.0f, 0.5f,
            spawnPos, commonScale,
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Quaternionf::Identity(),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            false,
            Eigen::Vector3f(1.f - (float)i / (float)numLinks, (float)i / (float)numLinks, 0.2f)
        );
        block->name = "BridgeLink_" + std::to_string(i);

        bool breakable = std::find(breakableRopes.begin(), breakableRopes.end(), i) != breakableRopes.end();
        float fracture = breakable ? 2000.f : INFINITY;

        for (int side = 0; side < 2; side++)
        {
            float zSign = (side == 0) ? 1.f : -1.f;
            Eigen::Vector3f rA_w, rB_w;
            Mesh* ropeBodyA = lastBody;
            if (lastBody)
            {
                rA_w = Eigen::Vector3f(0.5f, 0.f, zSign * zEdge);
                rB_w = Eigen::Vector3f(-0.5f, 0.f, zSign * zEdge);
            }
            else
            {
                rA_w = leftAnchorWorld + Eigen::Vector3f(0.f, 0.f, zSign * zEdgeWorld);
                rB_w = Eigen::Vector3f(-0.5f, 0.f, zSign * zEdge);
            }
            Spring* rope = new Spring(solver, ropeBodyA, rA_w, block, rB_w, 5000.0f, -1.f, false);
            rope->constraintPoints[0].fracture = fracture;
            solver->AddForce(std::unique_ptr<Force>(rope));
        }

        lastBody = block;
        spawnPos.x() += commonScale.x() + bridgeSpacing;
    }

    // right wall anchors: one bridgeSpacing to the right of block[11]'s right face
    Eigen::Vector3f endAnchor = startAnchor + Eigen::Vector3f(commonScale.x() * numLinks + (numLinks - 1) * bridgeSpacing, 0.f, 0.f);
    Eigen::Vector3f rightAnchorWorld = endAnchor + Eigen::Vector3f(bridgeSpacing, 0.f, 0.f);
    for (int side = 0; side < 2; side++)
    {
        float zSign = (side == 0) ? 1.f : -1.f;
        Spring* rope = new Spring(solver, nullptr, rightAnchorWorld + Eigen::Vector3f(0.f, 0.f, zSign * zEdgeWorld), lastBody, Eigen::Vector3f(0.5f, 0.f, zSign * zEdge), INFINITY, -1.f, false);
        solver->AddForce(std::unique_ptr<Force>(rope));
    }

    Eigen::Vector3f bridgeCenter = (startAnchor + endAnchor) * 0.5f;
    float bridgeZ = startAnchor.z();
    float bridgeCenterX = bridgeCenter.x();
    Eigen::Vector3f blockScale = Eigen::Vector3f(3.0f, 3.0f, 3.0f);

    // Falling weights to break bridge
    solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(bridgeCenterX - 0.5f, 10.f, bridgeZ), blockScale,
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        false, Eigen::Vector3f(1.f, 0.4f, 0.1f)
    )->name = "Weight_1";

    solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(bridgeCenterX + 0.5f, 16.f, bridgeZ), blockScale,
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        false, Eigen::Vector3f(1.f, 0.2f, 0.05f)
    )->name = "Weight_2";

    solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(bridgeCenterX, 23.f, bridgeZ), blockScale,
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        false, Eigen::Vector3f(0.8f, 0.1f, 0.05f)
    )->name = "Weight_3";

    camera->SetPosition(Eigen::Vector3f(5.0f, 2.0f, 35.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void NeoHookeanTetTest(Solver* solver, Camera* camera, const LevelParameters& params)
{
    Mesh* ground = solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(0.0f, -10.0f, 0.0f),
        Eigen::Vector3f(30.0f, 1.0f, 30.0f),
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        true
    );
    ground->name = "Ground";

    float groundTop = -9.5f;

    float particleMass = params.particleMass;
    float particleFriction = 0.5f;

    float tetEdge = 3.0f;
    float dropHeight = groundTop + 4.f;

    // ── Row 1: Varying stiffness E, fixed ν = 0.30 ──
    makeTet(solver, -12, dropHeight, 0, tetEdge,    50, 0.30f, Eigen::Vector3f(0.3f, 0.5f, 1.0f), params.particleMass);  
    makeTet(solver, -6, dropHeight, 0, tetEdge,   500, 0.30f, Eigen::Vector3f(0.4f, 0.6f, 1.0f), params.particleMass);
    makeTet(solver,  0, dropHeight, 0, tetEdge,  3000, 0.30f, Eigen::Vector3f(0.5f, 0.7f, 1.0f), params.particleMass);
    makeTet(solver,  6, dropHeight, 0, tetEdge,  8000, 0.30f, Eigen::Vector3f(0.6f, 0.8f, 1.0f), params.particleMass);
    makeTet(solver, 12, dropHeight, 0, tetEdge, 20000, 0.30f, Eigen::Vector3f(0.7f, 0.9f, 1.0f), params.particleMass); // nearly rigid

    // ── Row 2: Varying ν (incompressibility), fixed E = 500 ──
    float row2Z = -8.f;
    makeTet(solver, -12, dropHeight, row2Z, tetEdge, 500, 0.2f, Eigen::Vector3f(1.0f, 0.3f, 0.3f), params.particleMass);
    makeTet(solver, -6, dropHeight, row2Z, tetEdge, 500, 0.25f, Eigen::Vector3f(1.0f, 0.4f, 0.4f), params.particleMass);
    makeTet(solver,  0, dropHeight, row2Z, tetEdge, 500, 0.35f, Eigen::Vector3f(1.0f, 0.5f, 0.5f), params.particleMass);
    makeTet(solver,  6, dropHeight, row2Z, tetEdge, 500, 0.42f, Eigen::Vector3f(1.0f, 0.6f, 0.6f), params.particleMass);
    makeTet(solver, 12, dropHeight, row2Z, tetEdge, 500, 0.48f, Eigen::Vector3f(1.0f, 0.7f, 0.7f), params.particleMass);  // nearly incompressible

    // Front row 
    float row3Z = 8.f;
    makeTet(solver, 0, dropHeight, row3Z, tetEdge, params.E, params.nu, Eigen::Vector3f(0.3f, 0.0f, 1.3f), params.particleMass);

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 25.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void NeoHookeanMesh(Solver* solver, Camera* camera, const LevelParameters& params)
{
    /*
    Mesh* ground = solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(0.0f, -10.0f, 0.0f),
        Eigen::Vector3f(40.0f, 1.0f, 20.0f),
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        true
    );
    ground->name = "Ground";
    */

    const int   N    = 3;
    const float cs   = 0.9f;    // cell size (world units)
    const float E    = params.E;
    const float nu   = params.nu;
    const float mass = params.particleMass;

    const int nx = 3 * N;
    const int ny = N;
    const int nz = N;

    float groundTop = -3.5f;
    float beamX     = -3.0f;
    float beamY     = groundTop + 3.5f;
    float beamZ     = -(nz - 1) * cs * 0.5f;

    makeTetVolume( solver, beamX, beamY, beamZ, nx, ny, nz, cs, E, nu, Eigen::Vector3f(0.35f, 0.65f, 1.0f), mass, 0.5f, true);

    float beamCentreX = beamX + (nx - 1) * cs * 0.5f;
    float beamCentreY = beamY + (ny - 1) * cs * 0.5f;
    camera->SetPosition(Eigen::Vector3f(beamCentreX, beamCentreY + 5.0f, 22.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, -0.22f, -1.0f));
}

//================================//
static void SoftSpheres(Solver* solver, Camera* camera, const LevelParameters& params)
{
    float radius = 3.0;
    float res = 2;

    Mesh* ground = solver->AddBody(
        ModelType_Cube, 1.0f, 0.5f,
        Eigen::Vector3f(0.0f, -10.0f, 0.0f),
        Eigen::Vector3f(40.0f, 1.0f, 20.0f),
        Eigen::Vector3f::Zero(), Quaternionf::Identity(), Eigen::Vector3f::Zero(),
        true
    );
    ground->name = "Ground";

    float groundTop = -9.5f;
    float SpawnY = groundTop + radius + 5.0f;

    Eigen::Vector3f Spawn1 = Eigen::Vector3f(-2.f*radius, SpawnY, -2.f*radius);
    Eigen::Vector3f Spawn2 = Eigen::Vector3f(0.0f, SpawnY, 0.0f);
    Eigen::Vector3f Spawn3 = Eigen::Vector3f(2.f*radius, SpawnY, -2.f*radius);

    //makeTetSphere(solver, Spawn1.x(), Spawn1.y(), Spawn1.z(), radius, res, 500.f, 0.4f, Eigen::Vector3f(0.5f, 0.5f, 1.0f), params.particleMass);
    makeTetSphere(solver, Spawn2.x(), Spawn2.y(), Spawn2.z(), radius, res, params.E, params.nu, Eigen::Vector3f(1.0f, 0.5f, 0.5f), params.particleMass);
    //makeTetSphere(solver, Spawn3.x(), Spawn3.y(), Spawn3.z(), radius, res, 20000, 0.49, Eigen::Vector3f(0.5f, 1.0f, 0.5f), params.particleMass);

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 25.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void ClothSimulation(Solver* solver, Camera* camera, const LevelParameters& params)
{
    const int   N       = 10;
    const int   M       = 15;
    const float spacing = 1.5f;
    const float startX  = -(N - 1) * spacing * 0.5f;
    const float startY  = 5.f;

    const float startZ = -(M - 1) * spacing * 0.5f;

    std::vector<Mesh*> parts(N * M);
    for (int j = 0; j < M; j++)
    {
        for (int i = 0; i < N; i++)
        {
            Eigen::Vector3f pos(startX + i * spacing, startY, startZ + j * spacing);
            Mesh* p = solver->AddParticle(
                params.particleMass, 0.5f,
                pos, Eigen::Vector3f::Zero(),
                false,
                Eigen::Vector3f((float)i / (N - 1), 0.3f, 1.f - (float)j / (M - 1))
            );
            p->isParticle = true;
            p->name = "Cloth_" + std::to_string(i) + "_" + std::to_string(j);
            parts[i + j * N] = p;
        }
    }

    Mesh* topLeft  = parts[0];
    Mesh* topRight = parts[N - 1];
    topLeft->isStatic = true;
    topRight->isStatic = true;

    for (int j = 0; j < M - 1; j++)
    {
        for (int i = 0; i < N - 1; i++)
        {
            Mesh* v00 = parts[i     +  j      * N];
            Mesh* v10 = parts[(i+1) +  j      * N];
            Mesh* v01 = parts[i     + (j + 1) * N];
            Mesh* v11 = parts[(i+1) + (j + 1) * N];

            solver->AddEnergy(std::make_unique<STVKFEM>(solver, v00, v10, v11, params.E, params.nu));
            solver->AddEnergy(std::make_unique<STVKFEM>(solver, v00, v11, v01, params.E, params.nu));
        }
    }

    camera->SetPosition(Eigen::Vector3f(0.f, startY + 10.f, startZ + (M - 1) * spacing + 15.f));
    camera->LookAtDirection(Eigen::Vector3f(0.f, -0.5f, -1.f).normalized());
}

static void SafetyNet(Solver* solver, Camera* camera, const LevelParameters& params)
{
    int count = 15;
    const float spacing = 0.5f;

    const float startX  = -(count - 1) * spacing * 0.5f;
    const float startZ = -(count - 1) * spacing * 0.5f;
    const float startY  = -10.f;

    std::vector<Mesh*> parts(count * count);
    for (int j = 0; j < count; j++)
    {
        for (int i = 0; i < count; i++)
        {
            Eigen::Vector3f pos(startX + i * spacing, startY, startZ + j * spacing);
            Mesh* p = solver->AddParticle(
                params.particleMass, 0.5f,
                pos, Eigen::Vector3f::Zero(),
                false,
                Eigen::Vector3f((float)i / (count - 1), 0.3f, 1.f - (float)j / (count - 1))
            );
            p->isParticle = true;
            p->name = "Cloth_" + std::to_string(i) + "_" + std::to_string(j);
            parts[i + j * count] = p;
        }
    }

    Mesh* topLeft  = parts[0];
    Mesh* topRight = parts[count - 1];
    Mesh* bottomLeft = parts[(count - 1) * count];
    Mesh* bottomRight = parts[count * count - 1];

    topLeft->isStatic = true;
    topRight->isStatic = true;
    bottomLeft->isStatic = true;
    bottomRight->isStatic = true;

    for (int j = 0; j < count - 1; j++)
    {
        for (int i = 0; i < count - 1; i++)
        {
            Mesh* v00 = parts[i     +  j      * count];
            Mesh* v10 = parts[(i+1) +  j      * count];
            Mesh* v01 = parts[i     + (j + 1) * count];
            Mesh* v11 = parts[(i+1) + (j + 1) * count];

            solver->AddEnergy(std::make_unique<STVKFEM>(solver, v00, v10, v11, params.E, params.nu));
            solver->AddEnergy(std::make_unique<STVKFEM>(solver, v00, v11, v01, params.E, params.nu));
        }
    }


    camera->SetPosition(Eigen::Vector3f(0.0f, -3.0f, 25.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void (*levels[])(Solver*, Camera*, const LevelParameters&) =
{
    DefaultScene,
    Pyramid,
    MassStack,
    FrictionSlope,
    MassSprings,
    JointPlayground,
    NeoHookeanTetTest,
    NeoHookeanMesh,
    SoftSpheres,
    ClothSimulation,
    SafetyNet
};

//================================//
static const char* names[] = {
    "Default",
    "Pyramid",
    "MassStack",
    "FrictionSlope",
    "MassSprings",
    "JointPlayground",
    "NeoHookeanTetTest",
    "SoftBeam",
    "SoftSpheres",
    "ClothSimulation",
    "SafetyNet"
};

static const int numLevels = 11;

#endif // levels.h