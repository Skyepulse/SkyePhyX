#include "solver.hpp"
#include <algorithm>
#include "../helpers/time.hpp"
#include <cmath>
#include <iostream>
#include <map>
#include <functional>
#include <numeric>
#include "../constants.hpp"

const Eigen::Vector3f GRAVITY(0.0f, -9.81f, 0.0f);

const float PENALTY_MIN = 1;
const float PENALTY_MAX = 1000000000;
const float ENERGY_STIFFNESS_MIN = 0.01;

const float MAX_ROTATION_VELOCITY = 50.0f;

static constexpr float MAX_VELOCITY = 500.0f;
static constexpr float BROADPHASE_AABB_PADDING = 0.02f; // Prevents false negatives in sweep

//================================//
Solver::Solver()
{
    GeometryHelpers::computeModelGeometry(modelGeometry);
}

//================================//
Solver::~Solver()
{
    Clear();
}

//================================//
void Solver::Start()
{
    this->Clear();
}

//================================//
void Solver::Clear()
{
    solverForces.clear();
    solverBodies.clear();
    solverEnergies.clear();
    forcePtrs.clear();
    bodyPtrs.clear();
    energyPtrs.clear();

    prevTotalEnergy = 0.f;
    trustRegionRho  = 0.f;
    emergencyStop = false;
    surfaceDirty = true;
    surfaceFaces.clear();
    softBodySurfaceData.clear();
    primalColoring.Clear();
    broadPhaseEntries.clear();
    constrainedPairCounts.clear();
    broadPhaseEntriesDirty = true;
}

//================================//
void Solver::BuildSoftBodySurface()
{
    surfaceFaces.clear();

    std::vector<std::array<Mesh*, 3>> allFaces;
    allFaces.reserve(solverEnergies.size() * 4);
    for (auto& e : solverEnergies)
        e->AddFaces(allFaces);

    std::map<std::tuple<Mesh*, Mesh*, Mesh*>, int> faceCount;
    std::map<std::tuple<Mesh*, Mesh*, Mesh*>, std::array<Mesh*, 3>> firstSeen;

    for (const auto& f : allFaces)
    {
        std::array<Mesh*, 3> sorted = f;
        std::sort(sorted.begin(), sorted.end());
        auto key = std::make_tuple(sorted[0], sorted[1], sorted[2]);
        faceCount[key]++;
        if (faceCount[key] == 1)
            firstSeen[key] = f;
    }

    for (const auto& [key, count] : faceCount)
    {
        if (count == 1)
            surfaceFaces.push_back(firstSeen[key]);
    }
}

//================================//
void Solver::UpdateSoftBodySurfaceData()
{
    softBodySurfaceData.clear();
    softBodySurfaceData.reserve(surfaceFaces.size() * 3);

    for (const auto& face : surfaceFaces)
    {
        Eigen::Vector3f p0 = face[0]->transform.GetPosition();
        Eigen::Vector3f p1 = face[1]->transform.GetPosition();
        Eigen::Vector3f p2 = face[2]->transform.GetPosition();

        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0).normalized();
        Eigen::Vector3f color = (face[0]->color + face[1]->color + face[2]->color) / 3.0f;

        GPUSoftBodyVertex v0{};
        Eigen::Map<Eigen::Vector3f>(v0.pos) = p0;
        Eigen::Map<Eigen::Vector3f>(v0.norm) = n;
        Eigen::Map<Eigen::Vector3f>(v0.color) = color;
        v0.color[3] = 1.0f;

        GPUSoftBodyVertex v1{};
        Eigen::Map<Eigen::Vector3f>(v1.pos) = p1;
        Eigen::Map<Eigen::Vector3f>(v1.norm) = n;
        Eigen::Map<Eigen::Vector3f>(v1.color) = color;
        v1.color[3] = 1.0f;

        GPUSoftBodyVertex v2{};
        Eigen::Map<Eigen::Vector3f>(v2.pos) = p2;
        Eigen::Map<Eigen::Vector3f>(v2.norm) = n;
        Eigen::Map<Eigen::Vector3f>(v2.color) = color;
        v2.color[3] = 1.0f;

        softBodySurfaceData.push_back(v0);
        softBodySurfaceData.push_back(v1);
        softBodySurfaceData.push_back(v2);
    }
}

//================================//
Mesh* Solver::AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color)
{
    std::unique_ptr<Mesh> newMesh = std::make_unique<Mesh>(this, modelType, color);
    Mesh* raw = newMesh.get();

    newMesh->density            = density;
    newMesh->friction           = friction;
    newMesh->velocity           = velocity;
    newMesh->angularVelocity    = angularVelocity;
    newMesh->isStatic           = isStatic;

    newMesh->transform.SetPosition(position);
    newMesh->transform.SetRotation(rotation);
    newMesh->transform.SetScale(scale);

    newMesh->detectionRadius = scale.norm() * 0.5f;

    if (isStatic)
    {
        newMesh->inertiaTensorBody    = Eigen::Matrix3f::Zero();
        newMesh->inertiaTensorBodyInv = Eigen::Matrix3f::Zero();
    }
    else
    {
        float sx2 = scale.x() * scale.x(); float sy2 = scale.y() * scale.y(); float sz2 = scale.z() * scale.z();
        float r = scale.x() * 0.5f;

        Eigen::Vector3f I;
        switch(modelType)
        {
            case ModelType_Cube:
            {
                newMesh->mass = density * scale.x() * scale.y() * scale.z();
                I = (newMesh->mass / 12.0f) * Eigen::Vector3f(sy2 + sz2, sx2 + sz2, sx2 + sy2);
                break;
            }
            case ModelType_Sphere:
            {
                newMesh->mass = density * (4.0f / 3.0f) * M_PI * r * r * r;
                
                // ensure uniform scaling
                newMesh->transform.SetScale(Eigen::Vector3f(scale.x(), scale.x(), scale.x()));
                I = Eigen::Vector3f::Constant((2.0f / 5.0f) * newMesh->mass * r * r);
                break;
            }
            case ModelType_Capsule:
            {
                const float s  = scale.x();
                const float s2 = s * s;
                const float s3 = s2 * s;
                const float s5 = s3 * s2;
                const float pi = float(M_PI);

                newMesh->mass = density * pi * (5.0f / 96.0f) * s3;
                newMesh->transform.SetScale(Eigen::Vector3f(s, s, s));

                const float Iaxial = density * pi * (23.0f / 15360.0f) * s5;
                const float Itrans = density * pi * (121.0f / 30720.0f) * s5;

                I = Eigen::Vector3f(Itrans, Iaxial, Itrans);
                break;
            }
            default:
            {
                newMesh->mass = density * scale.x() * scale.y() * scale.z();
                I = (newMesh->mass / 12.0f) * Eigen::Vector3f(sy2 + sz2, sx2 + sz2, sx2 + sy2);
                break;
            }
        }

        newMesh->inertiaTensorBody = I.asDiagonal();
        newMesh->inertiaTensorBodyInv = I.cwiseInverse().asDiagonal();
    }

    raw->solverIndex = static_cast<int>(solverBodies.size());
    solverBodies.push_back(std::move(newMesh));
    broadPhaseEntriesDirty = true;

    this->RebuildPtrCaches();
    return raw;
}

//================================//
void Solver::RemoveBody(Mesh* body)
{
    // forces removed in destructor
    broadPhaseEntriesDirty = true;
    int idx = body->solverIndex;
    int last = static_cast<int>(solverBodies.size()) - 1;
    if (idx != last)
    {
        std::swap(solverBodies[idx], solverBodies[last]);
        solverBodies[idx]->solverIndex = idx;
    }
    solverBodies.pop_back();
    RebuildPtrCaches();
}

//================================//
Mesh* Solver::AddParticle(float mass, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, bool isStatic, const Eigen::Vector3f& color)
{
    Eigen::Vector3f particleScale(0.1f, 0.1f, 0.1f);
    Mesh* particle = AddBody(ModelType_Cube, mass / (particleScale.x() * particleScale.y() * particleScale.z()), friction, position, Eigen::Vector3f(0.1f, 0.1f, 0.1f), velocity, Quaternionf::Identity(), Eigen::Vector3f::Zero(), isStatic, color);
    particle->isParticle = true;

    // Zero out inertia and every rotation related properties since it's a particle
    particle->inertiaTensorBody = Eigen::Matrix3f::Zero();
        particle->inertiaTensorBodyInv = Eigen::Matrix3f::Zero();
        particle->angularVelocity = Eigen::Vector3f::Zero();
        
    return particle;
}

//================================//
Force* Solver::AddForce(std::unique_ptr<Force> force)
{
    Force* raw = force.get();
    raw->solverIndex = static_cast<int>(solverForces.size());
    RegisterForcePairs(raw, 1);
    solverForces.push_back(std::move(force));
    return raw;
}

//================================//
void Solver::RemoveForce(Force* force)
{
    int idx = force->solverIndex;
    if (idx < 0) return;

    RegisterForcePairs(force, -1);

    int last = static_cast<int>(solverForces.size()) - 1;
    if (idx != last)
    {
        std::swap(solverForces[idx], solverForces[last]);
        solverForces[idx]->solverIndex = idx;
    }
    solverForces.pop_back(); // Memo for me: this triggers ~Force()
}

//================================//
Energy* Solver::AddEnergy(std::unique_ptr<Energy> energy)
{
    Energy* raw = energy.get();
    raw->solverIndex = static_cast<int>(solverEnergies.size());
    solverEnergies.push_back(std::move(energy));
    surfaceDirty = true;
    return raw;
}

//================================//
void Solver::RemoveEnergy(Energy* energy)
{
    int idx = energy->solverIndex;
    if (idx < 0) return;

    int last = static_cast<int>(solverEnergies.size()) - 1;
    if (idx != last)
    {
        std::swap(solverEnergies[idx], solverEnergies[last]);
        solverEnergies[idx]->solverIndex = idx;
    }
    solverEnergies.pop_back();
    surfaceDirty = true;
}

//================================//
bool Solver::CheckExplosion()
{
    for (Mesh* mesh : bodyPtrs)
    {
        Eigen::Vector3f vel = mesh->velocity;
        if (vel.x() > MAX_VELOCITY || vel.y() > MAX_VELOCITY || vel.z() > MAX_VELOCITY ||
            vel.x() < -MAX_VELOCITY || vel.y() < -MAX_VELOCITY || vel.z() < -MAX_VELOCITY)
        {
            std::cout << "[ERROR][Solver] Velocity exploded on body '" << mesh->name 
                      << "' |vel| = " << vel.norm() << "\n";
            return true;
        }
    }
    return false;
}

//================================//
MeshPairKey Solver::MakeMeshPairKey(Mesh* meshA, Mesh* meshB)
{
    if (std::less<Mesh*>{}(meshB, meshA))
        std::swap(meshA, meshB);

    return { meshA, meshB };
}

//================================//
void Solver::RegisterForcePairs(Force* force, int delta)
{
    if (delta == 0) return;

    const int bodyCount = static_cast<int>(force->linkedBodies.size());
    for (int i = 0; i < bodyCount; ++i)
    {
        for (int j = i + 1; j < bodyCount; ++j)
        {
            Mesh* bodyA = force->linkedBodies[i];
            Mesh* bodyB = force->linkedBodies[j];
            if (bodyA == nullptr || bodyB == nullptr || bodyA == bodyB) continue;

            MeshPairKey key = MakeMeshPairKey(bodyA, bodyB);
            int& pairCount = constrainedPairCounts[key];
            pairCount += delta;

            if (pairCount <= 0)
                constrainedPairCounts.erase(key);
        }
    }
}

//================================//
bool Solver::HasConstraintPair(Mesh* meshA, Mesh* meshB) const
{
    if (meshA == nullptr || meshB == nullptr || meshA == meshB)
        return false;

    const MeshPairKey key = MakeMeshPairKey(meshA, meshB);
    return constrainedPairCounts.find(key) != constrainedPairCounts.end();
}

//================================//
void Solver::RefreshBroadPhaseEntries()
{
    if (broadPhaseEntriesDirty || broadPhaseEntries.size() != bodyPtrs.size())
    {
        broadPhaseEntries.clear();
        broadPhaseEntries.reserve(bodyPtrs.size());

        for (Mesh* mesh : bodyPtrs)
        {
            BroadPhaseSweepEntry entry;
            entry.mesh = mesh;
            broadPhaseEntries.push_back(entry);
        }

        broadPhaseEntriesDirty = false;
    }

    for (BroadPhaseSweepEntry& entry : broadPhaseEntries)
    {
        Mesh* mesh = entry.mesh;
        if (mesh == nullptr) continue;

        const Eigen::Vector3f position = mesh->transform.GetPosition();
        const Quaternionf rotation = mesh->transform.GetRotation();
        const Eigen::Vector3f scale = mesh->transform.GetScale();

        const bool transformChanged =
            !entry.cachedPosition.isApprox(position) ||
            !entry.cachedRotation.coeffs().isApprox(rotation.coeffs()) ||
            !entry.cachedScale.isApprox(scale);

        const bool shouldRecompute = !entry.hasCachedAABB || transformChanged;

        if (!shouldRecompute)
            continue;

        entry.aabb = mesh->GetWorldAABB();
        entry.aabb.min -= Eigen::Vector3f::Constant(BROADPHASE_AABB_PADDING);
        entry.aabb.max += Eigen::Vector3f::Constant(BROADPHASE_AABB_PADDING);
        entry.minX = entry.aabb.min.x();
        entry.maxX = entry.aabb.max.x();
        entry.cachedPosition = position;
        entry.cachedRotation = rotation;
        entry.cachedScale = scale;
        entry.hasCachedAABB = true;
    }
}

//================================//
void Solver::EnsureBroadPhaseOrder()
{
    bool isOrdered = true;
    for (std::size_t i = 1; i < broadPhaseEntries.size(); ++i)
    {
        if (broadPhaseEntries[i - 1].minX > broadPhaseEntries[i].minX)
        {
            isOrdered = false;
            break;
        }
    }

    // Insertion sort, must know if list is ordered or not already.
    if (isOrdered)
        return;

    for (std::size_t i = 1; i < broadPhaseEntries.size(); ++i)
    {
        BroadPhaseSweepEntry key = broadPhaseEntries[i];
        std::size_t j = i;
        while (j > 0 && broadPhaseEntries[j - 1].minX > key.minX)
        {
            broadPhaseEntries[j] = broadPhaseEntries[j - 1];
            --j;
        }
        broadPhaseEntries[j] = key;
    }
}

//================================//
std::vector<BroadPhaseSweepPair> Solver::broadPhaseSweep()
{
    std::vector<BroadPhaseSweepPair> pairs;
    RefreshBroadPhaseEntries();
    EnsureBroadPhaseOrder();

    // Sweep and find overlapping AABBs
    for (std::size_t i = 0; i < broadPhaseEntries.size(); ++i)
    {
        const BroadPhaseSweepEntry& entryA = broadPhaseEntries[i];
        Mesh* meshA = entryA.mesh;

        for (std::size_t j = i + 1; j < broadPhaseEntries.size(); ++j)
        {
            const BroadPhaseSweepEntry& entryB = broadPhaseEntries[j];
            Mesh* meshB = entryB.mesh;

            if (entryB.minX > entryA.maxX)
                break;

            if (meshA->isStatic && meshB->isStatic)
                continue;

            if (HasConstraintPair(meshA, meshB))
                continue;

            if (entryA.aabb.Overlaps(entryB.aabb))
            {
                pairs.push_back({meshA, meshB});
            }
        }
    }
    return pairs;
}

//================================//
void Solver::RebuildPrimalColoring()
{
    // 1. Reset status of graph coloring
    primalColoring.Clear();
    primalColoring.bodyColors.assign(bodyPtrs.size(), -1);
    for (Mesh* body : bodyPtrs)
    {
        body->avbdColor = -1;
        if (!body->isStatic)
            primalColoring.dynamicBodyIndices.push_back(body->solverIndex);
    }
    const int dynamicBodyCount = static_cast<int>(primalColoring.dynamicBodyIndices.size());
    if (dynamicBodyCount == 0)
    {
        primalColoring.colorOffsets.push_back(0);
        return;
    }

    // 2. Build global (solver index, static bodies in) to local (dynamic bodies only) index mapping
    std::vector<int> bodyToLocal(bodyPtrs.size(), -1);
    for (int i = 0; i < dynamicBodyCount; ++i)
        bodyToLocal[primalColoring.dynamicBodyIndices[i]] = i;

    // 3. Build adjacency list of the constraint graph (static bodies are ignored since they don't need to be colored)
    std::vector<std::vector<int>> adjacency(dynamicBodyCount);
    std::vector<int> elementBodies;
    auto addElementEdges = [&](const std::vector<Mesh*>& linkedBodies)
    {
        elementBodies.clear();
        elementBodies.reserve(linkedBodies.size());

        for (Mesh* body : linkedBodies)
        {
            if (body->isStatic) continue;

            const int localIndex = bodyToLocal[body->solverIndex];
            if (std::find(elementBodies.begin(), elementBodies.end(), localIndex) == elementBodies.end())
                elementBodies.push_back(localIndex);
        }

        const int count = static_cast<int>(elementBodies.size());
        for (int i = 0; i < count; ++i)
        {
            for (int j = i + 1; j < count; ++j)
            {
                const int a = elementBodies[i];
                const int b = elementBodies[j];
                adjacency[a].push_back(b);
                adjacency[b].push_back(a);
            }
        }
    };
    for (Force* force : forcePtrs)
        addElementEdges(force->linkedBodies);
    for (Energy* energy : energyPtrs)
        addElementEdges(energy->linkedBodies);

    // 4. Important for greedy coloring to work correctly: sort and unique adjacency lists 
    // so that the size of each list is correct (no duplicate neighbors)
    for (auto& neighbors : adjacency)
    {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }

    std::vector<int> order(dynamicBodyCount);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b)
    {
        return adjacency[a].size() == adjacency[b].size()
            ? primalColoring.dynamicBodyIndices[a] < primalColoring.dynamicBodyIndices[b]
            : adjacency[a].size() > adjacency[b].size();
    });

    // 5. Greedy coloring
    // We give the neighbor
    std::vector<int> colors(dynamicBodyCount, -1);
    std::vector<int> usedColors(dynamicBodyCount, -1);
    for (int localIndex : order)
    {
        for (int neighbor : adjacency[localIndex])
        {
            const int neighborColor = colors[neighbor];
            if (neighborColor >= 0)
                usedColors[neighborColor] = localIndex;
        }

        int color = 0;
        while (usedColors[color] == localIndex)
            ++color;

        // Assign color to body. It is the lowest color that is not used by any neighbor.
        colors[localIndex] = color;
        primalColoring.numColors = std::max(primalColoring.numColors, color + 1);
    }

    // 6. This is a compact start - end table
    // To find all bodies for certain color.
    primalColoring.colorOffsets.assign(primalColoring.numColors + 1, 0);
    for (int localIndex = 0; localIndex < dynamicBodyCount; ++localIndex)
    {
        const int bodyIndex = primalColoring.dynamicBodyIndices[localIndex];
        const int color = colors[localIndex];
        primalColoring.bodyColors[bodyIndex] = color;
        bodyPtrs[bodyIndex]->avbdColor = color;
        primalColoring.colorOffsets[color + 1]++;
    }
    for (int color = 1; color <= primalColoring.numColors; ++color)
        primalColoring.colorOffsets[color] += primalColoring.colorOffsets[color - 1];

    // 7. Build the body index list for each color based on the compact offset table
    primalColoring.colorBodyIndices.resize(dynamicBodyCount);
    std::vector<int> writeOffsets = primalColoring.colorOffsets;
    for (int localIndex = 0; localIndex < dynamicBodyCount; ++localIndex)
    {
        const int bodyIndex = primalColoring.dynamicBodyIndices[localIndex];
        const int color = colors[localIndex];
        primalColoring.colorBodyIndices[writeOffsets[color]++] = bodyIndex;
    }
}

//================================//
void Solver::Step()
{
    if (this->emergencyStop) return;

    if (surfaceDirty)
    {
        BuildSoftBodySurface();
        surfaceDirty = false;
    }
    UpdateSoftBodySurfaceData();

    SolverTimings& out = this->timings;

    Time::TimePoint stepStart = Time::Clock::now();

    this->RebuildPtrCaches();

    // 0. Cache Generalized Mass and Rotation Matrices
    for (Mesh* mesh : bodyPtrs)
    {
        if (mesh->isStatic) continue;

        Quaternionf q = mesh->transform.GetRotation();
        mesh->cachedRotationMatrix = q.toRotationMatrix();

        if (!mesh->isStatic && mesh->mass > 0.f)
        {
            Matrix6f& M = mesh->cachedGeneralizedMass;
            M.setZero();
            M.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * mesh->mass;
            M.block<3,3>(3,3) = mesh->cachedRotationMatrix 
                            * mesh->inertiaTensorBody 
                            * mesh->cachedRotationMatrix.transpose();
        }
    }

    // 1. Broad phase detection
    Time::TimePoint phaseStart = Time::Clock::now();
    std::vector<BroadPhaseSweepPair> pairs = broadPhaseSweep();
    for (const auto& pair : pairs)
    {        
        this->AddForce(std::make_unique<Manifold>(this, pair.bodyA, pair.bodyB));
    }
    out.broadPhaseMs = Time::MillisecondsSince(phaseStart);

    // 1.5 Rebuild force cache after broad phase
    phaseStart = Time::Clock::now();

    forcePtrs.clear();
    forcePtrs.reserve(solverForces.size());
    for (auto& f : solverForces) forcePtrs.push_back(f.get());

    // 2. Forces Warmstarting
    this->lineData.clear();
    this->debugPointData.clear();
    std::vector<Force*> toRemove;
    for (Force* force : forcePtrs)
    {
        if (!force->Initialize())
        {
            toRemove.push_back(force);
            continue;
        }

        force->AddLineData(this->lineData);
        force->AddDebugPointData(this->debugPointData);
        const int numConstraints = force->numConstraints();
        for (int i = 0; i < numConstraints; ++i)
        {
            if (!this->postStabilization)
                force->constraintPoints[i].lambda *= alpha * gamma;

            float newPenalty = force->constraintPoints[i].penalty * gamma;
            if (newPenalty < PENALTY_MIN) newPenalty = PENALTY_MIN;
            if (newPenalty > PENALTY_MAX) newPenalty = PENALTY_MAX;
            force->constraintPoints[i].penalty = newPenalty;

            force->constraintPoints[i].penalty = std::min(force->constraintPoints[i].penalty, force->constraintPoints[i].stiffness);
        }
    }
    for (Force* f : toRemove)
        RemoveForce(f);
    forcePtrs.clear();
    for (auto& f : solverForces) forcePtrs.push_back(f.get());


    // 3. Energies Warmstarting
    energyPtrs.clear();
    energyPtrs.reserve(solverEnergies.size());
    for (auto& e : solverEnergies) energyPtrs.push_back(e.get());

    std::vector<Energy*> energyToRemove;
    for (Energy* energy : energyPtrs)
    {
        if (!energy->Initialize())
        {
            energyToRemove.push_back(energy);
            continue;
        }

        energy->ComputeEnergyTerms(nullptr, projectionMode, trustRegionRho);
        energy->AddLineData(this->lineData);
    }

    for (Energy* e : energyToRemove)
        RemoveEnergy(e);
    energyPtrs.clear();
    for (auto& e : solverEnergies) energyPtrs.push_back(e.get());

    out.warmstartMs = Time::MillisecondsSince(phaseStart);

    phaseStart = Time::Clock::now();
    RebuildPrimalColoring();
    out.coloringMs = Time::MillisecondsSince(phaseStart);

    // 4. Bodies Warmstarting
    phaseStart = Time::Clock::now();
    for (Mesh* mesh : bodyPtrs)
    {
        mesh->angularVelocity = mesh->angularVelocity.cwiseMin(Eigen::Vector3f::Constant(MAX_ROTATION_VELOCITY)).cwiseMax(Eigen::Vector3f::Constant(-MAX_ROTATION_VELOCITY));

        Eigen::Vector3f pos = mesh->transform.GetPosition();
        mesh->inertialPosition = pos + mesh->velocity * stepValue;
        if (!mesh->isStatic)
        {
            mesh->inertialPosition += GRAVITY * stepValue * stepValue;
        }

        Quaternionf rot = mesh->transform.GetRotation();
        mesh->inertialRotation = rot;
        if (!mesh->isStatic && !mesh->isParticle)
        {
            Quaternionf q = mesh->inertialRotation;
            Quaternionf w(0.f, mesh->angularVelocity.x(), mesh->angularVelocity.y(), mesh->angularVelocity.z());

            Quaternionf wq;
            wq.w() = w.w()*q.w() - w.vec().dot(q.vec());
            wq.vec() = w.w()*q.vec() + q.w()*w.vec() + w.vec().cross(q.vec());

            q.w() += 0.5f * wq.w() * stepValue;
            q.vec() += 0.5f * wq.vec() * stepValue;
            mesh->inertialRotation = q.normalized();
        }

        Eigen::Vector3f linearAcceleration = (mesh->velocity - mesh->prevVelocity) / stepValue;
        float gravityMagnitude = GRAVITY.norm();
        float accelWeight = 0.0f;

        if (gravityMagnitude > 1e-6f)
        {
            Eigen::Vector3f gravityDir = GRAVITY / gravityMagnitude;
            float accelAlongGravity = linearAcceleration.dot(gravityDir);
            accelWeight = accelAlongGravity / gravityMagnitude;
            accelWeight = std::clamp(accelWeight, 0.f, 1.f);
        }

        mesh->lastPosition = pos;
        mesh->lastRotation = rot;

        Eigen::Vector3f newPos = pos + mesh->velocity * stepValue + GRAVITY * stepValue * stepValue * accelWeight;
        mesh->transform.SetPosition(newPos);

        if(!mesh->isStatic && !mesh->isParticle)
        {
            Quaternionf q = rot;
            Quaternionf w(0.f, mesh->angularVelocity.x(), mesh->angularVelocity.y(), mesh->angularVelocity.z());

            Quaternionf wq;
            wq.w() = w.w()*q.w() - w.vec().dot(q.vec());
            wq.vec() = w.w()*q.vec() + q.w()*w.vec() + w.vec().cross(q.vec());

            q.w() += 0.5f * wq.w() * stepValue;
            q.vec() += 0.5f * wq.vec() * stepValue;
            mesh->transform.SetRotation(q.normalized());
        }
    }
    out.predictionMs = Time::MillisecondsSince(phaseStart);

    // 5. Main Iteration Loop
    out.solveConstraintsMs = 0.0f;
    out.solveEnergiesMs    = 0.0f;
    out.solveLDLTMs        = 0.0f;
    phaseStart = Time::Clock::now();
    for (int iter = 0; iter < this->numIterations; ++iter)
    {
        // 5.1 Primal
        for (int color = 0; color < primalColoring.numColors; ++color)
        {
            for (int colorIndex = primalColoring.colorOffsets[color]; colorIndex < primalColoring.colorOffsets[color + 1]; ++colorIndex)
            {
                Mesh* mesh = bodyPtrs[primalColoring.colorBodyIndices[colorIndex]];
                Matrix6f lhs = mesh->cachedGeneralizedMass / (stepValue * stepValue);
                Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

                Time::TimePoint cStart = Time::Clock::now();
                for (Force* force : mesh->forces)
                {
                    force->ComputeConstraints(alpha);
                    force->ComputeDerivatives(mesh);

                    const int numConstraints = force->numConstraints();
                    for (int i = 0; i < numConstraints; ++i)
                    {
                        float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;
                        float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                        Matrix6f H = force->constraintPoints[i].H;
                        Matrix6f G = Matrix6f::Zero();
                        for (int j = 0; j < 6; ++j)
                        {
                            G(j, j) = H.col(j).norm() * std::abs(f);
                        }

                        const Vector6f J = force->constraintPoints[i].J;
                        rhs += J * f;
                        lhs.noalias() += J * J.transpose() * force->constraintPoints[i].penalty;

                        if (force->includeHessian)
                            lhs.noalias() += G;
                    }
                }
                out.solveConstraintsMs += Time::MillisecondsSince(cStart);

                Time::TimePoint eStart = Time::Clock::now();
                for (Energy* energy : mesh->energies)
                {
                    energy->ComputeEnergyTerms(mesh, projectionMode, trustRegionRho);

                    const Vector6f grad = energy->grad;
                    const Matrix6f hess = energy->hess;

                    rhs += grad;
                    lhs += hess;
                }
                out.solveEnergiesMs += Time::MillisecondsSince(eStart);

                Time::TimePoint lStart = Time::Clock::now();
                ldlt.compute(lhs);
                Vector6f dx = ldlt.solve(rhs);

                Eigen::Vector3f dx_lin = dx.head<3>();
                Eigen::Vector3f dx_ang = dx.tail<3>();

                Eigen::Vector3f pos = mesh->transform.GetPosition();
                mesh->transform.SetPosition(pos - dx_lin);

                Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
                Quaternionf rot = mesh->transform.GetRotation();
                mesh->transform.SetRotation((dq * rot).normalized());
                out.solveLDLTMs += Time::MillisecondsSince(lStart);
            }
        }

        // 5.2 trust region rho update
        if (projectionMode == EigenProjectionMode::ADAPTIVE)
        {
            // TODO... FOR NOW OK TO LEAVE EMPTY
        }

        // 5.3 Dual Update
        for (Force* force : forcePtrs)
        {
            force->ComputeConstraints(alpha);

            const int numConstraints = force->numConstraints();
            for (int i = 0; i < numConstraints; ++i)
            {
                float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;

                force->constraintPoints[i].lambda = lambda + force->constraintPoints[i].penalty * force->constraintPoints[i].C;
                if (force->constraintPoints[i].lambda < force->constraintPoints[i].fminMagnitude)
                    force->constraintPoints[i].lambda = force->constraintPoints[i].fminMagnitude;
                else if (force->constraintPoints[i].lambda > force->constraintPoints[i].fmaxMagnitude)
                    force->constraintPoints[i].lambda = force->constraintPoints[i].fmaxMagnitude;

                if (std::abs(force->constraintPoints[i].lambda) >= force->constraintPoints[i].fracture)
                {
                    for (Mesh* body : force->linkedBodies)
                        for (Force* otherForce : body->forces)
                            if (otherForce->isManifold)
                                otherForce->ResetValues();
                    force->Disable();
                    break;
                }

                if (force->constraintPoints[i].lambda > force->constraintPoints[i].fminMagnitude
                    && force->constraintPoints[i].lambda < force->constraintPoints[i].fmaxMagnitude)
                {
                    force->constraintPoints[i].penalty = std::min(force->constraintPoints[i].penalty + beta * std::abs(force->constraintPoints[i].C), std::min(force->constraintPoints[i].stiffness, PENALTY_MAX));
                }
            }
        }
    }
    out.primalDualMs = Time::MillisecondsSince(phaseStart);

    // 6. Velocity update
    phaseStart = Time::Clock::now();
    std::vector<Mesh*> oob;
    for (Mesh* mesh : bodyPtrs)
    {
        if (mesh->isStatic) continue;

        mesh->prevVelocity = mesh->velocity;
        mesh->prevAngularVelocity = mesh->angularVelocity;

        Eigen::Vector3f pos = mesh->transform.GetPosition();
        Eigen::Vector3f newVelocity = (pos - mesh->lastPosition) / stepValue;

        if (mesh->isDragged)
        {
            newVelocity += mesh->addedDragVelocity;
            mesh->addedDragVelocity.setZero();
        }

        mesh->velocity = newVelocity;

        if (!mesh->isParticle)
        {
            Quaternionf rot = mesh->transform.GetRotation();
            mesh->angularVelocity = RotationDifference(rot, mesh->lastRotation) / stepValue;
        }

        Eigen::Vector3f p = mesh->transform.GetPosition();
        if (p.x() < MINBOUNDS[0] || p.x() > MAXBOUNDS[0] ||
            p.y() < MINBOUNDS[1] || p.y() > MAXBOUNDS[1] ||
            p.z() < MINBOUNDS[2] || p.z() > MAXBOUNDS[2])
        {
            oob.push_back(mesh);
        }
    }
    this->emergencyStop = this->CheckExplosion();
    if (this->emergencyStop) return;
    out.velocityUpdateMs = Time::MillisecondsSince(phaseStart);

    // 6.5 Remove out of bound bodies
    for (Mesh* m : oob)
        RemoveBody(m);

    // 7. Post-stabilization: alpha = 0.0f
    phaseStart = Time::Clock::now();
    if (postStabilization)
    {
        for (int color = 0; color < primalColoring.numColors; ++color)
        {
            for (int colorIndex = primalColoring.colorOffsets[color]; colorIndex < primalColoring.colorOffsets[color + 1]; ++colorIndex)
            {
                Mesh* mesh = bodyPtrs[primalColoring.colorBodyIndices[colorIndex]];
                Matrix6f lhs = mesh->cachedGeneralizedMass / (stepValue * stepValue);
                Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

                for (Force* force : mesh->forces)
                {
                    force->ComputeConstraints(0.0f);
                    force->ComputeDerivatives(mesh);

                    const int numConstraints = force->numConstraints();
                    for (int i = 0; i < numConstraints; ++i)
                    {
                        float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;
                        float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                        Matrix6f H = force->constraintPoints[i].H;
                        Matrix6f G = Matrix6f::Zero();
                        for (int j = 0; j < 6; ++j)
                        {
                            G(j, j) = H.col(j).norm() * std::abs(f);
                        }

                        const Vector6f J = force->constraintPoints[i].J;
                        rhs += J * f;
                        lhs.noalias() += J * J.transpose() * force->constraintPoints[i].penalty;

                        if (force->includeHessian)
                            lhs.noalias() += G;
                    }
                }

                for (Energy* energy : mesh->energies)
                {
                    energy->ComputeEnergyTerms(mesh, projectionMode, trustRegionRho);

                    const Vector6f grad = energy->grad;
                    const Matrix6f hess = energy->hess;

                    rhs += grad;
                    lhs += hess;
                }

                Vector6f dx = lhs.ldlt().solve(rhs);
                Eigen::Vector3f dx_lin = dx.head<3>();
                Eigen::Vector3f dx_ang = dx.tail<3>();

                Eigen::Vector3f pos = mesh->transform.GetPosition();
                mesh->transform.SetPosition(pos - dx_lin);

                Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
                Quaternionf rot = mesh->transform.GetRotation();
                mesh->transform.SetRotation((dq * rot).normalized());
            }
        }
    }
    out.postStabMs = Time::MillisecondsSince(phaseStart);

    out.totalSubstepMs = Time::MillisecondsSince(stepStart);

    // Rolling average
    timingAccumulator.push_back(out);
    if (timingAccumulator.size() > TIMING_WINDOW)
        timingAccumulator.erase(timingAccumulator.begin());

    SolverTimings avg{};
    for (const auto& t : timingAccumulator)
    {
        avg.broadPhaseMs      += t.broadPhaseMs;
        avg.warmstartMs       += t.warmstartMs;
        avg.coloringMs        += t.coloringMs;
        avg.predictionMs      += t.predictionMs;
        avg.primalDualMs      += t.primalDualMs;
        avg.solveConstraintsMs += t.solveConstraintsMs;
        avg.solveEnergiesMs   += t.solveEnergiesMs;
        avg.solveLDLTMs       += t.solveLDLTMs;
        avg.velocityUpdateMs  += t.velocityUpdateMs;
        avg.postStabMs        += t.postStabMs;
        avg.totalSubstepMs    += t.totalSubstepMs;
    }
    float n = static_cast<float>(timingAccumulator.size());
    this->timings.broadPhaseMs      = avg.broadPhaseMs / n;
    this->timings.warmstartMs       = avg.warmstartMs / n;
    this->timings.coloringMs        = avg.coloringMs / n;
    this->timings.predictionMs      = avg.predictionMs / n;
    this->timings.primalDualMs      = avg.primalDualMs / n;
    this->timings.solveConstraintsMs = avg.solveConstraintsMs / n;
    this->timings.solveEnergiesMs   = avg.solveEnergiesMs / n;
    this->timings.solveLDLTMs       = avg.solveLDLTMs / n;
    this->timings.velocityUpdateMs  = avg.velocityUpdateMs / n;
    this->timings.postStabMs        = avg.postStabMs / n;
    this->timings.totalSubstepMs    = avg.totalSubstepMs / n;

    averageStepTime = this->timings.totalSubstepMs;
}
