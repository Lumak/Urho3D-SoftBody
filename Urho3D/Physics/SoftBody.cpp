//
// Copyright (c) 2008-2018 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#include "../Precompiled.h"

#include "../Core/Context.h"
#include "../Core/Profiler.h"
#include "../IO/Log.h"
#include "../Physics/PhysicsUtils.h"
#include "../Physics/PhysicsWorld.h"
#include "../Physics/SoftBody.h"
#include "../Physics/RigidBody.h"
#include "../Scene/Scene.h"
#include "../Scene/SmoothedTransform.h"

#include "../Graphics/StaticModel.h"
#include "../Graphics/Model.h"
#include "../Graphics/Geometry.h"
#include "../Graphics/VertexBuffer.h"
#include "../Graphics/IndexBuffer.h"
#include "../Resource/ResourceCache.h"
#include "..//IO/File.h"
#include "..//IO/FileSystem.h"

#include <Bullet/BulletSoftBody/btSoftBody.h>
#include <Bullet/BulletSoftBody/btSoftBodyHelpers.h>

namespace Urho3D
{

extern const char* PHYSICS_CATEGORY;

static const char* typeNames[] =
{
    "TriMesh",
    "Stick",
    "Rope",
    0
};

static const float DEFAULT_SOFT_BODY_MASS = 0.0f;
static const float DEFAULT_COLLISION_MARGIN = 0.04f;
static const unsigned DEFAULT_COLLISION_LAYER = 0x1;
static const unsigned DEFAULT_COLLISION_MASK = 0xffff;
static const float MIN_DEACTIVATION_VELOCITY = 0.02666f;
static const int MIN_DEACTIVATION_DELAY = 5;
static const float DEFAULT_CONFIG_VALUE = 0.1f;
static const float DEFAULT_CONFIG_PR = 1.0f;
static const float MIN_ROPE_HEIGHT_TOLERANCE = 0.01f;

SoftBody::SoftBody(Context* context) :
    LogicComponent(context),
    softBodyType_(SOFTBODY_TRIMESH),
    centerOfMass_(Vector3::ZERO),
    mass_(DEFAULT_SOFT_BODY_MASS),
    collisionLayer_(DEFAULT_COLLISION_LAYER),
    collisionMask_(DEFAULT_COLLISION_MASK),
    lastPosition_(Vector3::ZERO),
    lastRotation_(Quaternion::IDENTITY),
    useGravity_(true),
    readdBody_(false),
    inWorld_(false),
    enableMassUpdate_(true),
    deactivationVelocity_(MIN_DEACTIVATION_VELOCITY),
    deactivationDelay_(0),
    setToFaceNormals_(false),
    configLST_(DEFAULT_CONFIG_VALUE),
    configMT_(DEFAULT_CONFIG_VALUE),
    configVC_(DEFAULT_CONFIG_VALUE),
    configPR_(DEFAULT_CONFIG_PR),
    configDP_(0.0f),
    configCHR_(0.0f),
    adaptAndClearNodeTransform_(true),
    createFromStaticModel_(false),
    clothParam_(false),
    anchorBody_(false),
    minRopeHeightTolerance_(MIN_ROPE_HEIGHT_TOLERANCE)
{
    SetUpdateEventMask(USE_FIXEDPOSTUPDATE);
}

SoftBody::~SoftBody()
{
    ReleaseBody();

    if (physicsWorld_)
        physicsWorld_->RemoveSoftBody(this);
}

void SoftBody::RegisterObject(Context* context)
{
    context->RegisterFactory<SoftBody>(PHYSICS_CATEGORY);

    URHO3D_ENUM_ATTRIBUTE("SoftBody Type", softBodyType_, typeNames, SOFTBODY_TRIMESH, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Mass", float, mass_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Create From StaticModel", bool, createFromStaticModel_, false, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Use Cloth Param", bool, clothParam_, false, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Anchor Body", bool, anchorBody_, false, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Anchor To RigidBody", String, anchorToRigidBodyName_, String::EMPTY, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Wind Velocity", Vector3, windVelocity_, Vector3::ZERO, AM_DEFAULT);
}

void SoftBody::ApplyAttributes()
{
    switch (softBodyType_)
    {
    case SOFTBODY_TRIMESH:
        CreateFromStaticModel();
        break;

    case SOFTBODY_STICK:
        CreateStickFromStaticModel(1);
        break;

    case SOFTBODY_ROPE:
        CreateRopeFromStaticModel(2);
        break;
    }
    

    ApplyClothSetting();
    ApplyWindSetting();
    CopyNodesTransform();
    AddBodyToWorld();
    AnchorToRigidBody();
}

void SoftBody::SetSoftBodyType(SoftBodyType sftype)
{
    if (sftype != softBodyType_)
    {
        softBodyType_ = sftype;
        MarkNetworkUpdate();
    }
}

void SoftBody::CopyNodesTransform()
{
    SetTransform(node_->GetPosition(), node_->GetRotation());
    node_->SetPosition(Vector3::ZERO);
    node_->SetRotation(Quaternion::IDENTITY);
}

void SoftBody::ApplyClothSetting()
{
    if (body_)
    {
        if (clothParam_)
        {
            SetConfigLST(0.1f);
            SetConfigMT(0.0f);
            SetConfigVC(0.0f);
            SetConfigPR(0.0f);

            body_->generateBendingConstraints(2);
            
            body_->m_cfg.kLF = 0.05f;
            body_->m_cfg.kDG = 0.01f;
        }
    }
}

void SoftBody::ApplyWindSetting()
{
    if (body_)
    {
        if (windVelocity_ != Vector3::ZERO)
        {
            body_->m_cfg.piterations = 2;
            body_->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

            body_->setWindVelocity(ToBtVector3(windVelocity_));

            // disable deactivation
            SetDeactivationVelocity(0.0f);
        }
    }
}

void SoftBody::DelayedStart()
{
}

bool SoftBody::CreateFromStaticModel()
{
    StaticModel *smodel = node_->GetComponent<StaticModel>();

    if (smodel)
    {
        if (softBodyType_ != SOFTBODY_TRIMESH)
        {
            softBodyType_ = SOFTBODY_TRIMESH;
        }

        // clone
        smodel->SetModel(smodel->GetModel()->Clone());

        return CreateFromModel(smodel->GetModel());
    }

    return false;
}

bool SoftBody::CreateStickFromStaticModel(int fixed)
{
    StaticModel *smodel = node_->GetComponent<StaticModel>();

    if (smodel)
    {
        if (softBodyType_ != SOFTBODY_STICK)
        {
            softBodyType_ = SOFTBODY_STICK;
        }

        // clone
        smodel->SetModel(smodel->GetModel()->Clone());

        if (ConstructRopeData(smodel->GetModel()))
        {
            // for sticks, add the zeroth point below the mesh's 1st pt
            PODVector<btVector3> btVectorList(ropePointList_.Size() + 1);
            btVectorList[0] = ToBtVector3(ropePointList_[0] + Vector3(0.0f, -0.2f, 0.0f));

            for ( unsigned i = 0; i < ropePointList_.Size(); ++i )
            {
                btVectorList[i+1] = ToBtVector3(ropePointList_[i]);
            }

            int r = (int)btVectorList.Size();
            body_ = new btSoftBody(physicsWorld_->GetWorldInfo(), r, &btVectorList[0], 0);

            // anchor pts
            if (fixed&1) body_->setMass(0,0);
            if (fixed&2) body_->setMass(r-1,0);

            // clear the default values and change it to softbody's config behavior for rope
            ClearDefaultSettings();
            SetConfigDP(0.005f);
            SetConfigCHR(0.1f);

            // create links
            for ( int i = 1; i < r; ++i )
            {
                body_->appendLink(i-1, i);
            }

            for ( int i = 0; i < (int)ropePointList_.Size() - 1; ++i )
            {
                body_->generateBendingConstraints(2 + i);
            }

            // stick also anchors node 1
            body_->setMass(1,0);
            body_->getCollisionShape()->setMargin(DEFAULT_COLLISION_MARGIN);
        }
    }

    return (body_ != NULL);
}

bool SoftBody::CreateRopeFromStaticModel(int fixed)
{
    StaticModel *smodel = node_->GetComponent<StaticModel>();

    if (smodel)
    {
        if (softBodyType_ != SOFTBODY_ROPE)
        {
            softBodyType_ = SOFTBODY_ROPE;
        }

        // clone
        smodel->SetModel(smodel->GetModel()->Clone());

        if (ConstructRopeData(smodel->GetModel()))
        {
            PODVector<btVector3> btVectorList(ropePointList_.Size());

            for ( unsigned i = 0; i < ropePointList_.Size(); ++i )
            {
                btVectorList[i] = ToBtVector3(ropePointList_[i]);
            }

            int r = (int)btVectorList.Size();
            body_ = new btSoftBody(physicsWorld_->GetWorldInfo(), r, &btVectorList[0], 0);

            // anchor pts
            if (fixed&1) body_->setMass(0,0);
            if (fixed&2) body_->setMass(r-1,0);

            // clear the default values and change it to softbody's config behavior for rope
            ClearDefaultSettings();

            // create links
            for ( int i = 1; i < r; ++i )
            {
                body_->appendLink(i-1, i);
            }

            body_->m_cfg.piterations = 4;
            body_->getCollisionShape()->setMargin(DEFAULT_COLLISION_MARGIN);
        }
    }

    return (body_ != NULL);
}

void SoftBody::ClearDefaultSettings()
{
    SetConfigLST(1.0f);
    SetConfigMT(0.0f);
    SetConfigVC(0.0f);
    SetConfigPR(0.0f);
}

void SoftBody::AnchorToRigidBody()
{
    if (body_ && anchorBody_)
    {
        Node *rbodyNode = GetScene()->GetChild(anchorToRigidBodyName_, true);
        RigidBody *rbody = NULL;

        if (rbodyNode)
        {
            // should be immediate child
            rbody = rbodyNode->GetComponent<RigidBody>();
        }

        AppendAnchor(rbody);
    }
}

void SoftBody::AppendAnchor(RigidBody *rbody)
{
    Model *model = node_->GetComponent<StaticModel>()->GetModel();

    if (model)
    {
        Geometry *geometry = model->GetGeometry(0, 0);
        VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
        IndexBuffer *ibuffer = geometry->GetIndexBuffer();

        unsigned numVertices = vbuffer->GetVertexCount();
        unsigned elementMask = vbuffer->GetElementMask();
        unsigned char *vertexData = (unsigned char*)vbuffer->Lock(0, numVertices);

        if (vertexData)
        {
            const unsigned vertexSize = vbuffer->GetVertexSize();

            if (duplicatePairs_.Size() == 0)
            {
                // vertex orders are same as body->m_nodes
                for (int i = 0; i < body_->m_nodes.size(); ++i)
                {
                    unsigned char *dataAlign = (vertexData + i * vertexSize);

                    if (elementMask & MASK_POSITION) dataAlign += sizeof(Vector3);
                    if (elementMask & MASK_NORMAL) dataAlign += sizeof(Vector3);

                    if (elementMask & MASK_COLOR)
                    {
                        unsigned r = *reinterpret_cast<unsigned*>(dataAlign) & 0xff;

                        // vertex color determines the anchor point
                        if (r > 0xf0)
                        {
                            if (rbody)
                            {
                                body_->appendAnchor(i, rbody->GetBody());
                            }
                            else
                            {
                                body_->setMass(i, 0);
                            }
                        }
                    }
                }
            }
            else
            {
                PODVector<unsigned> anchorList;

                for ( unsigned i = 0; i < numVertices; ++i )
                {
                    unsigned char *dataAlign = (vertexData + i * vertexSize);

                    if (elementMask & MASK_POSITION) dataAlign += sizeof(Vector3);
                    if (elementMask & MASK_NORMAL) dataAlign += sizeof(Vector3);

                    if (elementMask & MASK_COLOR)
                    {
                        unsigned r = *reinterpret_cast<unsigned*>(dataAlign) & 0xff;

                        // vertex color determines the anchor point
                        if (r > 0xf0)
                        {
                            anchorList.Push(i);
                        }
                    }
                }

                for ( unsigned i = 0; i < remapList_.Size(); ++i )
                {
                    for (unsigned j = 0; j < anchorList.Size(); ++j )
                    {
                        if (remapList_[i] == anchorList[j])
                        {
                            if (rbody)
                            {
                                body_->appendAnchor(i, rbody->GetBody());
                            }
                            else
                            {
                                body_->setMass(i, 0);
                            }
                        }
                    }
                }
            }
            vbuffer->Unlock();
        }
    }
}

void SoftBody::AnchorZeroWeight()
{
    AppendAnchor(NULL);
}

void SoftBody::SetMass(float mass)
{
    mass = Max(mass, 0.0f);

    if (mass != mass_)
    {
        mass_ = mass;
        AddBodyToWorld();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetPosition(const Vector3& position)
{
    if (body_)
    {
        btTransform& worldTrans = body_->getWorldTransform();
        worldTrans.setOrigin(ToBtVector3(position + ToQuaternion(worldTrans.getRotation()) * centerOfMass_));
        body_->transform(worldTrans);

        Activate();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetRotation(const Quaternion& rotation)
{
    if (body_)
    {
        Vector3 oldPosition = GetPosition();
        btTransform& worldTrans = body_->getWorldTransform();
        worldTrans.setRotation(ToBtQuaternion(rotation));
        if (!centerOfMass_.Equals(Vector3::ZERO))
            worldTrans.setOrigin(ToBtVector3(oldPosition + rotation * centerOfMass_));

        body_->transform(worldTrans);

        Activate();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetTransform(const Vector3& position, const Quaternion& rotation)
{
    if (body_)
    {
        btTransform worldTrans;
        worldTrans.setRotation(ToBtQuaternion(rotation));
        worldTrans.setOrigin(ToBtVector3(position + rotation * centerOfMass_));

        body_->transform(worldTrans);

        Activate();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetScale(const Vector3& scale)
{
    if (body_)
    {
        body_->scale(ToBtVector3(scale));
    }
}

void SoftBody::SetCollisionLayer(unsigned layer)
{
    if (layer != collisionLayer_)
    {
        collisionLayer_ = layer;
        AddBodyToWorld();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetCollisionMask(unsigned mask)
{
    if (mask != collisionMask_)
    {
        collisionMask_ = mask;
        AddBodyToWorld();
        MarkNetworkUpdate();
    }
}

void SoftBody::SetCollisionLayerAndMask(unsigned layer, unsigned mask)
{
    if (layer != collisionLayer_ || mask != collisionMask_)
    {
        collisionLayer_ = layer;
        collisionMask_ = mask;
        AddBodyToWorld();
        MarkNetworkUpdate();
    }
}

Vector3 SoftBody::GetPosition() const
{
    if (body_)
    {
        return ToVector3(body_->m_pose.m_com);
    }

    return Vector3::ZERO;
}

Quaternion SoftBody::GetRotation() const
{
    if (body_)
    {
        // sofbody uses rotation * scale in the computation
        btTransform xform = btTransform(body_->m_pose.m_rot*body_->m_pose.m_scl);
        return ToQuaternion(xform.getRotation());
    }

    return Quaternion::IDENTITY;
}

Matrix3x4 SoftBody::GetTransform() const
{
    return Matrix3x4(GetPosition(), GetRotation(), 1.0f);
}

void SoftBody::Activate()
{
    if (body_ && mass_ > 0.0f)
        body_->activate(true);
}

bool SoftBody::IsActive() const
{
    return body_?body_->isActive():false;
}

void SoftBody::AddBodyToWorld()
{
    if (!physicsWorld_)
        return;

    URHO3D_PROFILE(AddBodyToWorld);

    if (mass_ < 0.0f)
        mass_ = 0.0f;


    if (body_)
    {
        RemoveBodyFromWorld();
    }

    if (!body_)
        return;

    UpdateMass();

    if (!IsEnabledEffective())
        return;

    btSoftRigidDynamicsWorld* world = (btSoftRigidDynamicsWorld*)physicsWorld_->GetWorld();
    world->addSoftBody(body_.Get(), (int)collisionLayer_, (int)collisionMask_);
    inWorld_ = true;
    readdBody_ = false;

    if (mass_ > 0.0f)
    {
        Activate();
    }
}

void SoftBody::OnMarkedDirty(Node* node)
{
    // If node transform changes, apply it back to the physics transform. However, do not do this when a SmoothedTransform
    // is in use, because in that case the node transform will be constantly updated into smoothed, possibly non-physical
    // states; rather follow the SmoothedTransform target transform directly
    // Also, for kinematic objects Bullet asks the position from us, so we do not need to apply ourselves
    if ((!physicsWorld_ || !physicsWorld_->IsApplyingTransforms()))
    {
        // Physics operations are not safe from worker threads
        Scene* scene = GetScene();
        if (scene && scene->IsThreadedUpdate())
        {
            scene->DelayedMarkedDirty(this);
            return;
        }

        // Check if transform has changed from the last one set in ApplyWorldTransform()
        Vector3 newPosition = node_->GetWorldPosition();
        Quaternion newRotation = node_->GetWorldRotation();

        if (!newRotation.Equals(lastRotation_))
        {
            lastRotation_ = newRotation;
            SetRotation(newRotation);
        }
        if (!newPosition.Equals(lastPosition_))
        {
            lastPosition_ = newPosition;
            SetPosition(newPosition);
        }
    }
}

void SoftBody::OnNodeSet(Node* node)
{
    if (node)
        node->AddListener(this);
}

void SoftBody::OnSceneSet(Scene* scene)
{
    // call base
    LogicComponent::OnSceneSet(scene);

    if (scene)
    {
        if (scene == node_)
            URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

        physicsWorld_ = scene->GetOrCreateComponent<PhysicsWorld>();
        physicsWorld_->AddSoftBody(this);

        AddBodyToWorld();
    }
    else
    {
        ReleaseBody();

        if (physicsWorld_)
            physicsWorld_->RemoveSoftBody(this);
    }
}

void SoftBody::ReleaseBody()
{
    if (body_)
    {
        RemoveBodyFromWorld();

        body_.Reset();
    }
}

void SoftBody::RemoveBodyFromWorld()
{
    if (physicsWorld_ && body_ && inWorld_)
    {
        btSoftRigidDynamicsWorld* world = (btSoftRigidDynamicsWorld*)physicsWorld_->GetWorld();
        world->removeSoftBody(body_.Get());
        inWorld_ = false;
    }
}

void SoftBody::UpdateMass()
{
    if (!body_ || !enableMassUpdate_)
        return;

    if (mass_ > 0.0f)
    {
        body_->setTotalMass(mass_);
    }
}

void SoftBody::SetVelocity(const Vector3& velocity)
{
    if (body_)
    {
        body_->setVelocity(ToBtVector3(velocity));
    }
}

void SoftBody::SetDefaultConfiguration()
{
    if (body_)
    {
        // minimum default settings just to keep the volume, some what, intact
        body_->m_materials[0]->m_kLST = (btScalar)configLST_; // Linear stiffness coefficient [0,1]
        body_->m_cfg.kMT              = (btScalar)configMT_;  // Pose matching coefficient [0,1]
        body_->m_cfg.kVC              = (btScalar)configVC_;  // Volume conservation coefficient [0,+inf]
        body_->m_cfg.kPR              = (btScalar)configPR_;  // Pressure coefficient [-inf,+inf]

        body_->setPose(true, true);
        body_->getCollisionShape()->setMargin(DEFAULT_COLLISION_MARGIN);
    }
}

void SoftBody::SetConfigLST(float lst)
{
    if (body_)
    {
        configLST_ = lst;
        body_->m_materials[0]->m_kLST = configLST_;
    }
}

void SoftBody::SetConfigMT(float mt)
{
    if (body_)
    {
        configMT_ = mt;
        body_->m_cfg.kMT = configMT_;
    }
}

void SoftBody::SetConfigVC(float vc)
{
    if (body_)
    {
        configVC_ = vc;
        body_->m_cfg.kVC = configVC_;
    }
}

void SoftBody::SetConfigPR(float pr)
{
    if (body_)
    {
        configPR_ = pr;
        body_->m_cfg.kPR = configPR_;
    }
}

void SoftBody::SetConfigDP(float dp)
{
    if (body_)
    {
        configDP_ = dp;
        body_->m_cfg.kDP = configDP_;
    }
}

void SoftBody::SetConfigCHR(float chr)
{
    if (body_)
    {
        configCHR_ = chr;
        body_->m_cfg.kCHR = configCHR_;
    }
}

void SoftBody::GenerateBendingConstraints(int dist)
{
    if (body_)
    {
        body_->generateBendingConstraints(dist);
    }
}

bool SoftBody::CreateFromModel(Model *model)
{
    if (model)
    {
        // prune the model and remove all duplicate verts
        SharedPtr<Model> pmodel = PruneModel(model);

        Geometry *geometry = pmodel->GetGeometry(0, 0);
        VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
        IndexBuffer *ibuffer = geometry->GetIndexBuffer();

        unsigned numVertices = vbuffer->GetVertexCount();
        const unsigned char *vertexData = (const unsigned char*)vbuffer->Lock(0, numVertices);

        // temp
        PODVector<btScalar> btVertices;
        PODVector<int> btIndices;

        if (vertexData)
        {
            const unsigned vertexSize = vbuffer->GetVertexSize();
            btVertices.Resize(numVertices * 3);

            for ( unsigned i = 0; i < numVertices; ++i )
            {
                const Vector3& v0 = *reinterpret_cast<const Vector3*>(vertexData + i * vertexSize);
                btVertices[i*3 + 0] = (btScalar)v0.x_;
                btVertices[i*3 + 1] = (btScalar)v0.y_;
                btVertices[i*3 + 2] = (btScalar)v0.z_;
            }

            vbuffer->Unlock();
        }

        unsigned numIndices = ibuffer->GetIndexCount();
        const unsigned char *indexData = (const unsigned char*)ibuffer->Lock(0, numIndices);

        if (indexData)
        {
            const unsigned indexSize = ibuffer->GetIndexSize();
            btIndices.Resize(numIndices);

            for ( unsigned i = 0; i < numIndices; ++i )
            {
                if (indexSize == sizeof(unsigned short))
                {
                    btIndices[i] = (int)*reinterpret_cast<const unsigned short*>(indexData + i * indexSize);
                }
                else
                {
                    btIndices[i] = (int)*reinterpret_cast<const unsigned*>(indexData + i * indexSize);
                }
            }

            ibuffer->Unlock();
        }

        // create
        body_ = btSoftBodyHelpers::CreateFromTriMesh(*physicsWorld_->GetWorldInfo(), &btVertices[0], &btIndices[0], (int)numIndices/3);

        // set default config
        SetDefaultConfiguration();
    }

    return (body_ != NULL);
}

bool SoftBody::ConstructRopeData(Model *model)
{
    Geometry *geometry = model->GetGeometry(0, 0);
    VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
    IndexBuffer *ibuffer = geometry->GetIndexBuffer();

    unsigned numVertices = vbuffer->GetVertexCount();
    unsigned vertexSize = vbuffer->GetVertexSize();
    const unsigned char *vertexData = (const unsigned char*)vbuffer->Lock(0, numVertices);

    if (vertexData)
    {
        // gather model heights from bottom to top
        GatherRopeHeightOrder(ropePointList_, model->GetBoundingBox().min_.y_, vertexData, vertexSize, numVertices, minRopeHeightTolerance_);
        CollectRopeMeshPoints(meshPointList_, ropePointList_, vertexData, vertexSize, numVertices, minRopeHeightTolerance_);
        vbuffer->Unlock();
    }

    return (ropePointList_.Size() && meshPointList_.Size());
}

void SoftBody::GatherRopeHeightOrder(PODVector<Vector3> &ropePointList, float startingHeight,
                                     const unsigned char *vertexData, unsigned vertexSize, unsigned numVertices,
                                     const float heightTolerance)
{
    ropePointList.Clear();
    float curHeight = startingHeight;

    while (true)
    {
        Vector3 avgPoint = Vector3::ZERO;
        int numPts = 0;
        float minHdelta = M_LARGE_VALUE;
        float nextHeight = M_LARGE_VALUE;

        for ( unsigned i = 0; i < numVertices; ++i )
        {
            const Vector3 &v0 = *reinterpret_cast<const Vector3*>(vertexData + i * vertexSize);
            float hdelta = Abs(v0.y_ - curHeight);

            if (hdelta < heightTolerance)
            {
                avgPoint += v0;
                ++numPts;
            }
            else if (hdelta < minHdelta && v0.y_ > curHeight + heightTolerance)
            {
                minHdelta = hdelta;
                nextHeight = v0.y_;
            }
        }

        if (numPts)
        {
            avgPoint = avgPoint * 1.0f/(float)numPts;
            ropePointList.Push(avgPoint);

            if (nextHeight < M_LARGE_VALUE)
            {
                curHeight = nextHeight;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
}

void SoftBody::CollectRopeMeshPoints(PODVector<MeshPoint> &meshPointList, const PODVector<Vector3> &ropePointList, 
                                     const unsigned char *vertexData, unsigned vertexSize, unsigned numVertices,
                                     const float heightTolerance)
{
    assert(ropePointList.Size() && "no rope point list found");

    meshPointList.Clear();

    for ( unsigned i = 0; i < ropePointList.Size(); ++i )
    {
        Vector3 curPoint = ropePointList[i];
        Vector3 prevPoint = (i==0)?ropePointList[0] + Vector3(0.0f, -0.2f, 0.0f):ropePointList[i-1];
        Vector3 upVec = (curPoint - prevPoint).Normalized();
        float curHeight = ropePointList[i].y_;

        for ( unsigned j = 0; j < numVertices; ++j )
        {
            const Vector3 &v0 = *reinterpret_cast<const Vector3*>(vertexData + j * vertexSize);
            float hdelta = Abs(v0.y_ - curHeight);

            if (hdelta < heightTolerance)
            {
                meshPointList.Resize(meshPointList.Size() + 1);
                MeshPoint &meshPoint = meshPointList[meshPointList.Size() - 1];
                meshPoint.crossVec_ = Vector3::ZERO;
                Vector3 seg = v0 - curPoint;
                if (seg.LengthSquared() > M_EPSILON)
                {
                    meshPoint.crossVec_ = upVec.CrossProduct(seg);
                }

                // store node and buff idx
                meshPoint.nodeIdx_ = i;
                meshPoint.buffIdx_ = j;
            }
        }
    }
}

SharedPtr<Model> SoftBody::PruneModel(Model *model)
{
    Geometry *geometry = model->GetGeometry(0, 0);
    VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
    IndexBuffer *ibuffer = geometry->GetIndexBuffer();

    unsigned numVertices = vbuffer->GetVertexCount();
    unsigned vertexSize = vbuffer->GetVertexSize();
    const unsigned char *vertexData = (const unsigned char*)vbuffer->Lock(0, numVertices);

    // find duplicate verts
    if (vertexData)
    {
        for ( unsigned i = 0; i < numVertices-1; ++i )
        {
            const Vector3 &v0 = *reinterpret_cast<const Vector3*>(vertexData + i * vertexSize);

            for ( unsigned j = i+1; j < numVertices; ++j )
            {
                const Vector3 &v1 = *reinterpret_cast<const Vector3*>(vertexData + j * vertexSize);

                if ((v1 - v0).LengthSquared() < M_EPSILON)
                {
                    duplicatePairs_.Push(MakePair(i, j));
                }
            }
        }
        vbuffer->Unlock();
    }

    // clone model
    SharedPtr<Model> cloneModel = model->Clone();

    // remap index and vertex buffer
    if (duplicatePairs_.Size() != 0)
    {
        Geometry *geometryPruned = cloneModel->GetGeometry(0, 0);
        VertexBuffer *vbufferPruned = geometryPruned->GetVertexBuffer(0);
        IndexBuffer *ibufferPruned = geometryPruned->GetIndexBuffer();

        unsigned short *ushortData = (unsigned short*)ibuffer->Lock(0, ibuffer->GetIndexCount());
        unsigned *unsignedData = (unsigned*)ushortData;
        unsigned short *prunedUShortData = (unsigned short*)ibufferPruned->Lock(0, ibufferPruned->GetIndexCount());
        unsigned *prunedUnsignedData = (unsigned*)prunedUShortData;
        bool isShort = (ibuffer->GetIndexSize() == sizeof(unsigned short));

        if (ushortData && prunedUShortData)
        {
            unsigned numIndices = ibuffer->GetIndexCount();
            Vector<unsigned> indexList;
            indexList.Resize(numIndices);

            // copy index list
            for( unsigned i = 0; i < numIndices; ++i )
            {
                indexList[i] = (unsigned)(isShort?ushortData[i]:unsignedData[i]);
            }

            // swap duplicate indices
            for( unsigned i = 0; i < numIndices; ++i )
            {
                for ( unsigned j = 0; j < duplicatePairs_.Size(); ++j )
                {
                    const Pair<unsigned, unsigned> &pr = duplicatePairs_[j];

                    if (indexList[i] == pr.second_)
                    {
                        indexList[i] = pr.first_;
                    }
                }
            }

            // store indices in the order that they appear
            for( unsigned i = 0; i < numIndices; ++i )
            {
                unsigned idx = indexList[i];
                bool found = false;

                for ( unsigned j = 0; j < remapList_.Size(); ++j )
                {
                    if (remapList_[j] == idx)
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    remapList_.Push(idx);
                }
            }

            // populate new index order
            for( unsigned i = 0; i < numIndices; ++i )
            {
                unsigned idx = indexList[i];

                for ( unsigned j = 0; j < remapList_.Size(); ++j )
                {
                    if (remapList_[j] == idx)
                    {
                        if (isShort)
                        {
                            prunedUShortData[i] = (unsigned short)j;
                        }
                        else
                        {
                            prunedUnsignedData[i] = j;
                        }
                    }
                }
            }
            ibuffer->Unlock();
            ibufferPruned->Unlock();
        }

        // generate new pruned vertex buffer
        unsigned uElementMask = vbufferPruned->GetElementMask();
        vbufferPruned->SetSize(remapList_.Size(), uElementMask);

        vertexData = (const unsigned char*)vbuffer->Lock(0, vbuffer->GetVertexCount());
        unsigned char *prunedVertexData = (unsigned char*)vbufferPruned->Lock(0, vbufferPruned->GetVertexCount());

        if (vertexData && prunedVertexData)
        {
            unsigned vertexSize = vbufferPruned->GetVertexSize();

            for ( unsigned i = 0; i < remapList_.Size(); ++i )
            {
                memcpy((prunedVertexData + i * vertexSize), (vertexData + remapList_[i] * vertexSize), vertexSize);
            }

            vbufferPruned->Unlock();
            vbuffer->Unlock();
        }
    }

    return cloneModel;
}

void SoftBody::SetToFaceNormals(Model *model)
{
    if (model && IsActive() && duplicatePairs_.Size())
    {
        Geometry *geometry = model->GetGeometry(0, 0);
        VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
        IndexBuffer *ibuffer = geometry->GetIndexBuffer();

        const unsigned numVertices = vbuffer->GetVertexCount();
        const unsigned vertexSize = vbuffer->GetVertexSize();
        unsigned elementMask = vbuffer->GetElementMask();
        const unsigned numIndices = ibuffer->GetIndexCount();
        const unsigned indexSize = ibuffer->GetIndexSize();

        if ((elementMask & MASK_NORMAL) == 0)
        {
            return;
        }

        unsigned char *vertexData = (unsigned char*)vbuffer->Lock(0, numVertices);
        const unsigned char *indexData = (const unsigned char*)ibuffer->Lock(0, numIndices);
        unsigned i0, i1, i2;

        if (vertexData && indexData)
        {
            for (unsigned i = 0; i < numIndices / 3; ++i)
            {
                if (indexSize == sizeof(unsigned short))
                {
                    i0 = (unsigned)*reinterpret_cast<const unsigned short*>(indexData + (i*3 + 0) * indexSize);
                    i1 = (unsigned)*reinterpret_cast<const unsigned short*>(indexData + (i*3 + 1) * indexSize);
                    i2 = (unsigned)*reinterpret_cast<const unsigned short*>(indexData + (i*3 + 2) * indexSize);
                }
                else
                {
                    i0 = (unsigned)*reinterpret_cast<const unsigned*>(indexData + (i*3 + 0) * indexSize);
                    i1 = (unsigned)*reinterpret_cast<const unsigned*>(indexData + (i*3 + 1) * indexSize);
                    i2 = (unsigned)*reinterpret_cast<const unsigned*>(indexData + (i*3 + 2) * indexSize);
                }

                Vector3 &v0 = *reinterpret_cast<Vector3*>(vertexData + i0 * vertexSize);
                Vector3 &v1 = *reinterpret_cast<Vector3*>(vertexData + i1 * vertexSize);
                Vector3 &v2 = *reinterpret_cast<Vector3*>(vertexData + i2 * vertexSize);

                Vector3 n = ((v1 - v0).CrossProduct(v2 - v0)).Normalized();
                *reinterpret_cast<Vector3*>(vertexData + i0 * vertexSize + sizeof(Vector3)) = n;
                *reinterpret_cast<Vector3*>(vertexData + i1 * vertexSize + sizeof(Vector3)) = n;
                *reinterpret_cast<Vector3*>(vertexData + i2 * vertexSize + sizeof(Vector3)) = n;
            }
            vbuffer->Unlock();
            ibuffer->Unlock();
        }
    }
}

void SoftBody::UpdateVertexBuffer(Model *model)
{
    if (model && IsActive())
    {
        Geometry *geometry = model->GetGeometry(0, 0);
        VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
        IndexBuffer *ibuffer = geometry->GetIndexBuffer();

        unsigned numVertices = vbuffer->GetVertexCount();
        unsigned elementMask = vbuffer->GetElementMask();
        unsigned char *vertexData = (unsigned char*)vbuffer->Lock(0, numVertices);

        boundingBox_.Clear();

        if (vertexData)
        {
            const unsigned vertexSize = vbuffer->GetVertexSize();

            if (duplicatePairs_.Size() == 0)
            {
                for (int i = 0; i < body_->m_nodes.size(); ++i)
                {
                    btSoftBody::Node& n = body_->m_nodes[i];
                    Vector3 pos = ToVector3(n.m_x);
                    boundingBox_.Merge(pos);
                    unsigned char *dataAlign = (vertexData + i * vertexSize);

                    if (elementMask & MASK_POSITION)
                    {
                        *reinterpret_cast<Vector3*>(dataAlign) = pos;
                        dataAlign += sizeof(Vector3);
                    }
                    if (elementMask & MASK_NORMAL)
                    {
                        *reinterpret_cast<Vector3*>(dataAlign) = ToVector3(n.m_n);
                    }
                }
            }
            else
            {
                Vector<unsigned> updateList;

                for ( int i = 0; i < body_->m_nodes.size(); ++i )
                {
                    btSoftBody::Node& n = body_->m_nodes[i];
                    Vector3 pos = ToVector3(n.m_x);
                    Vector3 normal = ToVector3(n.m_n);
                    boundingBox_.Merge(pos);

                    // get the vert idx of the node
                    unsigned remapIndex = remapList_[i];

                    // restore duplicate verts associated with 
                    // the vert idx of the node's pos and normal
                    updateList.Clear();
                    updateList.Push(remapIndex);

                    for ( unsigned j = 0; j < duplicatePairs_.Size(); ++j )
                    {
                        if (duplicatePairs_[j].first_ == remapIndex)
                        {
                            updateList.Push(duplicatePairs_[j].second_);
                        }
                    }

                    for ( unsigned j = 0; j < updateList.Size(); ++j )
                    {
                        unsigned char *dataAlign = (vertexData + updateList[j] * vertexSize);

                        if (elementMask & MASK_POSITION)
                        {
                            *reinterpret_cast<Vector3*>(dataAlign) = pos;
                            dataAlign += sizeof(Vector3);
                        }
                        if (elementMask & MASK_NORMAL)
                        {
                            *reinterpret_cast<Vector3*>(dataAlign) = normal;
                        }
                    }
                }
            }
            vbuffer->Unlock();
        }

        if (setToFaceNormals_)
        {
            SetToFaceNormals(model);
        }
    }
}

void SoftBody::UpdateRope(Model *model)
{
    if (model && IsActive())
    {
        Geometry *geometry = model->GetGeometry(0, 0);
        VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
        IndexBuffer *ibuffer = geometry->GetIndexBuffer();

        unsigned numVertices = vbuffer->GetVertexCount();
        unsigned elementMask = vbuffer->GetElementMask();
        const unsigned vertexSize = vbuffer->GetVertexSize();
        unsigned char *vertexData = (unsigned char*)vbuffer->Lock(0, numVertices);

        boundingBox_.Clear();

        if (vertexData)
        {
            if (softBodyType_ == SOFTBODY_STICK)
            {
                // skip the zeroth node
                for ( unsigned i = 1; i < (unsigned)body_->m_nodes.size(); ++i )
                {
                    Vector3 curPoint = ToVector3(body_->m_nodes[i].m_x);
                    Vector3 prevPoint = ToVector3(body_->m_nodes[i-1].m_x);
                    Vector3 upVec = (curPoint - prevPoint).Normalized();

                    for ( unsigned j = 0; j < meshPointList_.Size(); ++j )
                    {
                        MeshPoint &meshPoint = meshPointList_[j];

                        // stick node idx is off by 1 due to the zeroth node that we added
                        if (meshPoint.nodeIdx_ == i - 1)
                        {
                            Vector3 pt;

                            if (meshPoint.crossVec_ != Vector3::ZERO)
                            {
                                pt = curPoint + meshPoint.crossVec_.CrossProduct(upVec);
                            }
                            else
                            {
                                pt = curPoint;
                            }

                            *reinterpret_cast<Vector3*>(vertexData + meshPoint.buffIdx_ * vertexSize) = pt;

                            boundingBox_.Merge(pt);
                        }
                    }
                }
            }
            else
            {
                for ( unsigned i = 0; i < (unsigned)body_->m_nodes.size(); ++i )
                {
                    Vector3 curPoint = ToVector3(body_->m_nodes[i].m_x);
                    Vector3 prevPoint = (i==0)?ToVector3(body_->m_nodes[i+1].m_x):ToVector3(body_->m_nodes[i-1].m_x);
                    Vector3 upVec = (i==0)?(prevPoint-curPoint).Normalized():(curPoint - prevPoint).Normalized();

                    for ( unsigned j = 0; j < meshPointList_.Size(); ++j )
                    {
                        MeshPoint &meshPoint = meshPointList_[j];

                        if (meshPoint.nodeIdx_ == i)
                        {
                            Vector3 pt;

                            if (meshPoint.crossVec_ != Vector3::ZERO)
                            {
                                pt = curPoint + meshPoint.crossVec_.CrossProduct(upVec);
                            }
                            else
                            {
                                pt = curPoint;
                            }

                            *reinterpret_cast<Vector3*>(vertexData + meshPoint.buffIdx_ * vertexSize) = pt;

                            boundingBox_.Merge(pt);
                        }
                    }
                }
            }
            vbuffer->Unlock();
        }
    }
}

void SoftBody::CheckRestCondition()
{
    if (IsActive())
    {
        float cumulativeVelocity = 0.0f;

        for ( int i = 0; i < body_->m_nodes.size(); ++i )
        {
            cumulativeVelocity += body_->m_nodes[i].m_v.length2();
        }

        // put it to sleep
        cumulativeVelocity /= (float)body_->m_nodes.size();

        if (cumulativeVelocity < deactivationVelocity_ && --deactivationDelay_ < 0)
        {
            body_->forceActivationState(ISLAND_SLEEPING);

            // delay deactivation 
            deactivationDelay_ = MIN_DEACTIVATION_DELAY;
        }
    }
    else
    {
        // when a softbody transitions from sleep state to active state, a few ticks are 
        // required for the body's nodes to accumulate velocity, hence, the delay
        deactivationDelay_ = MIN_DEACTIVATION_DELAY;
    }
}

void SoftBody::FixedPostUpdate(float timeStep)
{
    if (IsActive())
    {
        StaticModel *statModel = node_->GetComponent<StaticModel>();

        if (statModel)
        {
            if (softBodyType_ == SOFTBODY_TRIMESH)
            {
                UpdateVertexBuffer(statModel->GetModel());
            }
            else
            {
                UpdateRope(statModel->GetModel());
            }

            statModel->SetBoundingBox(boundingBox_);
        }
    }

    // check if at rest
    CheckRestCondition();
}

SharedPtr<Model> SoftBody::CreateModelFromBulletMesh(Context *context, float *varray, int numVertices, int *iarray, int numTriangles)
{
    //**note** I find it easier to rewrite an existing model with new data than to create one from 
    // scratch to avoid initializing all the required settings for geometry, vertex and index buffer.
    // The orig model should be the box model, which is the simplest model to clone.
    Model *origModel = context->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
    SharedPtr<Model> model = origModel->Clone();
    Geometry *geometry = model->GetGeometry(0, 0);
    VertexBuffer *vbuffer = geometry->GetVertexBuffer(0);
    IndexBuffer *ibuffer = geometry->GetIndexBuffer();
    unsigned elementMask = vbuffer->GetElementMask();

    // change vertex buff
    unsigned newElementMask = MASK_POSITION | MASK_NORMAL;
    vbuffer->SetSize(numVertices, newElementMask);
    unsigned vertexSize = vbuffer->GetVertexSize();
    unsigned char *vertexData = (unsigned char*)vbuffer->Lock(0, numVertices);

    // change index buff
    ibuffer->SetSize(numTriangles*3, false);
    unsigned short *ushortData = (unsigned short*)ibuffer->Lock(0, ibuffer->GetIndexCount());
    BoundingBox boundingBox;

    if (vertexData && ushortData)
    {
        for ( int i = 0; i < numVertices; ++i )
        {
            Vector3 v = Vector3(varray[i*3], varray[i*3 + 1], varray[i*3 + 2]);

            boundingBox.Merge(v);

            *reinterpret_cast<Vector3*>(vertexData + i * vertexSize) = v;

            // init normal
            *reinterpret_cast<Vector3*>(vertexData + i * vertexSize + sizeof(Vector3)) = Vector3::ZERO;
        }

        for ( int i = 0; i < numTriangles; ++i )
        {
            unsigned short i0 = (unsigned short)iarray[i * 3 + 0];
            unsigned short i1 = (unsigned short)iarray[i * 3 + 1];
            unsigned short i2 = (unsigned short)iarray[i * 3 + 2];

            *reinterpret_cast<unsigned short*>(ushortData + (i * 3 + 0)) = i0;
            *reinterpret_cast<unsigned short*>(ushortData + (i * 3 + 1)) = i1;
            *reinterpret_cast<unsigned short*>(ushortData + (i * 3 + 2)) = i2;

            // calc and sum normals
            const Vector3 &v0 = *reinterpret_cast<Vector3*>(vertexData + i0 * vertexSize);
            const Vector3 &v1 = *reinterpret_cast<Vector3*>(vertexData + i1 * vertexSize);
            const Vector3 &v2 = *reinterpret_cast<Vector3*>(vertexData + i2 * vertexSize);

            Vector3 n = ((v1 - v0).CrossProduct(v2 - v0)).Normalized();
            *reinterpret_cast<Vector3*>(vertexData + i0 * vertexSize + sizeof(Vector3)) += n;
            *reinterpret_cast<Vector3*>(vertexData + i1 * vertexSize + sizeof(Vector3)) += n;
            *reinterpret_cast<Vector3*>(vertexData + i2 * vertexSize + sizeof(Vector3)) += n;
        }

        // normalize normals
        for ( int i = 0; i < numVertices; ++i )
        {
            Vector3 &n = *reinterpret_cast<Vector3*>(vertexData + i * vertexSize + sizeof(Vector3));
            n.Normalize();
        }

        vbuffer->Unlock();
        ibuffer->Unlock();
    }

    // finalize
    geometry->SetDrawRange(geometry->GetPrimitiveType(), 0, numTriangles*3, 0, numVertices, false);
    model->SetBoundingBox(boundingBox);

    return model;
}


}

