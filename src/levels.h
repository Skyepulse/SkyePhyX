#ifndef LEVELS_H
#define LEVELS_H

#include "physics/solver.hpp"
#include "helpers/camera.hpp"

//================================//
static void DefaultScene(Solver* solver, Camera* camera)
{
    Mesh* ground = solver->AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Ground";

    camera->SetPosition(Eigen::Vector3f(0.0f, -2.0f, 30.0f));
    camera->LookAtDirection(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
}

//================================//
static void Pyramid(Solver* solver, Camera* camera)
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
static void MassStack(Solver* solver, Camera* camera)
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
static void FrictionSlope(Solver* solver, Camera* camera)
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
static void MassSprings(Solver* solver, Camera* camera)
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
static void JointPlayground(Solver* solver, Camera* camera)
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

    // bridge, same but last body has anchor into world
    startAnchor = Eigen::Vector3f(-10.f, 5.f, -5.f);
    commonScale = Eigen::Vector3f(2.f, 0.5f, 3.f);
    numLinks = 12;
    lastBody = nullptr;
    spawnPos = Eigen::Vector3f(startAnchor.x() + commonScale.x() / 2.f, startAnchor.y(), startAnchor.z());

    std::vector<int> breakeableJoints = {6};
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

        Vector6f fractures = Vector6f::Constant(INFINITY);
        if (std::find(breakeableJoints.begin(), breakeableJoints.end(), i) != breakeableJoints.end())
        {
            fractures.head<3>() = Eigen::Vector3f(5000.f, 5000.f, 5000.f);
            fractures.tail<3>() = Eigen::Vector3f(500.f, 500.f, 500.f);
        }

        Force* joint = new Joint(solver, lastBody, lastBody ? relRight : startAnchor, block, relLeft, pendulumAnchors, fractures);
        solver->AddForce(std::unique_ptr<Force>(joint));
        lastBody = block;
        spawnPos.x() += commonScale.x();
    }

    // anchor last block to world
    Eigen::Vector3f endAnchor = startAnchor + Eigen::Vector3f(commonScale.x() * numLinks, 0.f, 0.f);
    Force* lastJoint = new Joint(solver, nullptr, endAnchor, lastBody, relRight, pendulumAnchors);
    solver->AddForce(std::unique_ptr<Force>(lastJoint));

    Eigen::Vector3f bridgeCenter = (startAnchor + endAnchor) * 0.5f;
    float bridgeZ = startAnchor.z();
    float bridgeCenterX = bridgeCenter.x();
    Eigen::Vector3f blockScale = Eigen::Vector3f(1.5f, 1.5f, 1.5f);

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
static void (*levels[])(Solver*, Camera*) = 
{
    DefaultScene,
    Pyramid,
    MassStack,
    FrictionSlope,
    MassSprings,
    JointPlayground
};

//================================//
static const char* names[] = {
    "Default",
    "Pyramid",
    "MassStack",
    "FrictionSlope",
    "MassSprings",
    "JointPlayground"
};

static const int numLevels = 6;

#endif // levels.h