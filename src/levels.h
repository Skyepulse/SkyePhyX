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
static void (*levels[])(Solver*, Camera*) = 
{
    DefaultScene,
    Pyramid,
    MassStack
};

//================================//
static const char* names[] = {
    "Default",
    "Pyramid",
    "MassStack"
};

static const int numLevels = 3;

#endif // levels.h