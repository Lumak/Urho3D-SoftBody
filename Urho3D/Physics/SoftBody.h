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
#pragma once

#include "../Scene/LogicComponent.h"

class btSoftBody;
class btTransform;

namespace Urho3D
{
class PhysicsWorld;
class BoundingBox;
class Model;
class RigidBody;

enum SoftBodyType
{
    SOFTBODY_TRIMESH,
    SOFTBODY_STICK,
    SOFTBODY_ROPE,
};

/// SoftBody component.
class URHO3D_API SoftBody : public LogicComponent
{
    URHO3D_OBJECT(SoftBody, LogicComponent);

public:
    /// Construct.
    SoftBody(Context* context);
    /// Destruct. Free the rigid body and geometries.
    virtual ~SoftBody();
    /// Register object factory.
    static void RegisterObject(Context* context);

    virtual void ApplyAttributes();
    /// Generate model based on Bullet Mesh.
    static SharedPtr<Model> CreateModelFromBulletMesh(Context *context, float *varray, int numVertices, int *iarray, int numTriangles);

    /// Called before the first update. At this point all other components of the node should exist. Will also be called if update events are not wanted; in that case the event is immediately unsubscribed afterward.
    virtual void DelayedStart();
    /// Called on physics post-update, fixed timestep.
    virtual void FixedPostUpdate(float timeStep);

    bool CreateFromStaticModel();
    bool CreateStickFromStaticModel(int fixed);
    bool CreateRopeFromStaticModel(int fixed);
    void AnchorZeroWeight();

    void SetSoftBodyType(SoftBodyType sftype);
    SoftBodyType GetSoftBodyType() const { return softBodyType_; }

    /// Set collision layer.
    void SetCollisionLayer(unsigned layer);
    /// Set collision mask.
    void SetCollisionMask(unsigned mask);
    /// Set collision group and mask.
    void SetCollisionLayerAndMask(unsigned layer, unsigned mask);
    /// Set mass. Zero mass makes the body static.
    void SetMass(float mass);
    /// Set softbody position in world space.
    void SetPosition(const Vector3& position);
    /// Set softbody rotation in world space.
    void SetRotation(const Quaternion& rotation);
    /// Set softbody position and rotation in world space as atomic peration. 
    void SetTransform(const Vector3& position, const Quaternion& rotation);
    /// Set scale.
    void SetScale(const Vector3& scale);
    /// Set velocity.
    void SetVelocity(const Vector3& velocity);

    /// Return Bullet soft body.
    btSoftBody* GetBody() const { return body_.Get(); }
    /// Return mass.
    float GetMass() const { return mass_; }
    /// Return softbody position in world space.
    Vector3 GetPosition() const;
    /// Return softbody rotation in world space.
    Quaternion GetRotation() const;
    /// Return softbody transform in world space.
    Matrix3x4 GetTransform() const;

    /// Activate.
    void Activate();
    /// Is Active.
    bool IsActive() const;
    /// Remove the rigid body.
    void ReleaseBody();
    /// Update mass.
    void UpdateMass();

    void SetDeactivationVelocity(float deactiveVel) { deactivationVelocity_ = deactiveVel; }
    void SetFaceNormals(bool setFaceNormals)        { setToFaceNormals_ = setFaceNormals; }
    void SetConfigLST(float lst);
    void SetConfigMT(float mt);
    void SetConfigVC(float vc);
    void SetConfigPR(float pr);
    void SetConfigDP(float dp);
    void SetConfigCHR(float chr);

    void GenerateBendingConstraints(int dist);

    float GetDeactivationVelocity() const   { return deactivationVelocity_; }
    bool GetFaceNormals() const             { return setToFaceNormals_; }
    float GetConfigLST() const              { return configLST_; }
    float GetConfigMT() const               { return configMT_; }
    float GetConfigVC() const               { return configVC_; }
    float GetConfigPR() const               { return configPR_; }
    float GetConfigDP() const               { return configDP_; }
    float GetConfigCHR() const              { return configCHR_; }

protected:
    bool CreateFromModel(Model *model);
    bool ConstructRopeData(Model *model);
    void ClearDefaultSettings();
    void SetToFaceNormals(Model *model);
    void AnchorToRigidBody();
    void AppendAnchor(RigidBody *rbody);
    void ApplyClothSetting();
    void ApplyWindSetting();
    void CopyNodesTransform();
    void UpdateVertexBuffer(Model *model);
    void UpdateRope(Model *model);
    /// Handle node transform being dirtied.
    virtual void OnMarkedDirty(Node* node);
    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);
    /// Handle scene being assigned.
    virtual void OnSceneSet(Scene* scene);
    /// Create the rigid body, or re-add to the physics world with changed flags. Calls UpdateMass().
    void AddBodyToWorld();
    /// Remove the rigid body from the physics world.
    void RemoveBodyFromWorld();

    void SetDefaultConfiguration();
    SharedPtr<Model> PruneModel(Model *model);
    void CheckRestCondition();

protected:
    /// Bullet soft body.
    UniquePtr<btSoftBody> body_;
    WeakPtr<PhysicsWorld> physicsWorld_;
    /// softbody type.
    SoftBodyType softBodyType_;
    /// Gravity override vector.
    Vector3 gravityOverride_;
    /// Center of mass offset.
    Vector3 centerOfMass_;
    /// Mass.
    float mass_;
    /// Collision layer.
    unsigned collisionLayer_;
    /// Collision mask.
    unsigned collisionMask_;
    /// Last interpolated position from the simulation.
    mutable Vector3 lastPosition_;
    /// Last interpolated rotation from the simulation.
    mutable Quaternion lastRotation_;
    /// Use gravity flag.
    bool useGravity_;
    /// Readd body to world flag.
    bool readdBody_;
    /// Body exists in world flag.
    bool inWorld_;
    /// Mass update enable flag.
    bool enableMassUpdate_;
    /// Deactivation velocity.
    float deactivationVelocity_;
    /// Deactivation delay.
    int deactivationDelay_;
    /// Boundingbox
    BoundingBox boundingBox_;
    /// Vertex duplicate pairs.
    Vector<Pair<unsigned, unsigned> > duplicatePairs_;
    /// Vertex remap list.
    PODVector<unsigned> remapList_;
    /// Normals based on MDL model
    bool setToFaceNormals_;
    /// Linear stiffness coefficient [0,1]
    float configLST_;
    /// Pose matching coefficient [0,1]
    float configMT_;
    /// Volume conservation coefficient [0,+inf]
    float configVC_;
    /// Pressure coefficient [-inf,+inf]
    float configPR_;
    float configDP_;
    float configCHR_;

    bool adaptAndClearNodeTransform_;
    bool createFromStaticModel_;
    bool clothParam_;
    bool anchorBody_;
    Vector3 windVelocity_;
    String anchorToRigidBodyName_;

private:
    struct MeshPoint
    {
        Vector3 crossVec_;
        unsigned nodeIdx_;
        unsigned buffIdx_;
    };

    PODVector<Vector3> ropePointList_;
    PODVector<MeshPoint> meshPointList_;
    float minRopeHeightTolerance_;

    static void GatherRopeHeightOrder(PODVector<Vector3> &heightList, float startingHeight,
                                      const unsigned char *vertexData, unsigned vertexSize, unsigned numVertices,
                                      const float heightTolerance);
    static void CollectRopeMeshPoints(PODVector<MeshPoint> &meshPointList, const PODVector<Vector3> &heightList, 
                                      const unsigned char *vertexData, unsigned vertexSize, unsigned numVertices,
                                      const float heightTolerance);

};

}
