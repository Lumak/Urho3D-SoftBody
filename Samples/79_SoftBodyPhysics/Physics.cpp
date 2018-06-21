//
// Copyright (c) 2008-2017 the Urho3D project.
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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/SoftBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include "Physics.h"
#include "SoftBodyHelper.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(Physics)

//=============================================================================
//=============================================================================
Physics::Physics(Context* context) :
    Sample(context),
    drawDebug_(false)
{
    SoftBodyHelper::RegisterObject(context_);
}

void Physics::Setup()
{
    engineParameters_["WindowTitle"]   = GetTypeName();
    engineParameters_["LogName"]       = GetSubsystem<FileSystem>()->GetProgramDir() + "softbody.log";
    engineParameters_["FullScreen"]    = false;
    engineParameters_["Headless"]      = false;
    engineParameters_["WindowWidth"]   = 1280; 
    engineParameters_["WindowHeight"]  = 720;
    engineParameters_["ResourcePaths"] = "Data;CoreData;Data/SoftBody;";
}

void Physics::Start()
{
    Sample::Start();

    CreateScene();

    CreateInstructions();

    SetupViewport();

    SubscribeToEvents();

    Sample::InitMouseMode(MM_RELATIVE);

    ChangeDebugHudText();
}

void Physics::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // **note** how-to create softbody physic world in code
    // PhysicsWorld *physicsWorld = new PhysicsWorld(context_, true);
    // scene_->AddComponent(physicsWorld, 0, REPLICATED);

    scene_ = new Scene(context_);
    XMLFile *xmlLevel = cache->GetResource<XMLFile>("SoftBody/Scenes/scene1.xml");
    scene_->LoadXML(xmlLevel->GetRoot());

    // Create the camera.
    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(500.0f);

    // Set an initial position for the camera scene node above the floor
    Node *camSpawnNode = scene_->GetChild("cameraSpawn");
    if (camSpawnNode)
    {
        cameraNode_->SetPosition(camSpawnNode->GetPosition());

        pitch_ = camSpawnNode->GetRotation().EulerAngles().z_;
        yaw_ = camSpawnNode->GetRotation().EulerAngles().y_;
    }
    else
    {
        cameraNode_->SetPosition(Vector3(0.0f, 2.0f, -5.0f));
    }
}

void Physics::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
        "Use WASD keys and mouse to move\n"
        "LMB: softbody object, RMB: box object\n"
        "MMB: sphere object, SPACEBAR: dbg physics\n"
    );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    instructionText->SetColor(Color::CYAN);
    instructionText->SetTextEffect(TE_SHADOW);
    instructionText->SetEffectColor(Color(0.0f, 0.3f, 0.3f));
    instructionText->SetEffectStrokeThickness(1);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetPosition(0, 10);
}

void Physics::SetupViewport()
{
    Renderer* renderer = GetSubsystem<Renderer>();

    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void Physics::ChangeDebugHudText()
{
    // change profiler text
    if (GetSubsystem<DebugHud>())
    {
        Text *dbgText = GetSubsystem<DebugHud>()->GetProfilerText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetStatsText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetMemoryText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetModeText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);
    }
}

void Physics::SubscribeToEvents()
{
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Physics, HandleUpdate));
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(Physics, HandlePostRenderUpdate));
}

void Physics::MoveCamera(float timeStep)
{
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    Input* input = GetSubsystem<Input>();

    const float MOVE_SPEED = 20.0f;
    const float MOUSE_SENSITIVITY = 0.1f;

    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);

    // "Shoot" a physics object with left mousebutton
    if (input->GetMouseButtonPress(MOUSEB_LEFT))
        SpawnObject(0);
    if (input->GetMouseButtonPress(MOUSEB_RIGHT))
        SpawnObject(1);
    if (input->GetMouseButtonPress(MOUSEB_MIDDLE))
        SpawnObject(2);

    // Toggle physics debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;
}

void Physics::SpawnObject(int objtype)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    const float OBJECT_VELOCITY = 10.0f;

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild("SmallBox");

    if (objtype == 0)
    {
        StaticModel *boxObject = boxNode->CreateComponent<StaticModel>();
        boxObject->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
        boxObject->SetMaterial(cache->GetResource<Material>("Materials/uvMat.xml"));
        boxObject->SetCastShadows(true);

        SoftBody *softbody = boxNode->CreateComponent<SoftBody>();
        softbody->CreateFromStaticModel();
        softbody->SetTransform(cameraNode_->GetPosition() + cameraNode_->GetDirection(), cameraNode_->GetRotation());
        softbody->SetMass(10.0f);
        softbody->SetVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);

        // recalculate normals based on the mdl triangle faces
        // consider enabling this only for models such as a box
        //softbody->SetFaceNormals(true);
    }
    else if (objtype == 1)
    {
        boxNode->SetPosition(cameraNode_->GetPosition() + cameraNode_->GetDirection());
        boxNode->SetRotation(cameraNode_->GetRotation());
        boxNode->SetScale(0.5f);
        StaticModel* boxObject = boxNode->CreateComponent<StaticModel>();
        boxObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        boxObject->SetMaterial(cache->GetResource<Material>("Materials/cubeMat.xml"));
        boxObject->SetCastShadows(true);

        // Create physics components, use a smaller mass also
        RigidBody* body = boxNode->CreateComponent<RigidBody>();
        body->SetMass(0.25f);
        body->SetFriction(0.75f);
        CollisionShape* shape = boxNode->CreateComponent<CollisionShape>();
        shape->SetBox(Vector3::ONE);
        body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);
    }
    else
    {
        boxNode->SetPosition(cameraNode_->GetPosition() + cameraNode_->GetDirection());
        boxNode->SetRotation(cameraNode_->GetRotation());
        boxNode->SetScale(0.75f);
        StaticModel* boxObject = boxNode->CreateComponent<StaticModel>();
        boxObject->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
        boxObject->SetMaterial(cache->GetResource<Material>("Materials/cubeMat.xml"));
        boxObject->SetCastShadows(true);

        // Create physics components, use a smaller mass also
        RigidBody* body = boxNode->CreateComponent<RigidBody>();
        body->SetMass(0.5f);
        body->SetFriction(0.75f);
        body->SetAngularDamping(0.5f);
        CollisionShape* shape = boxNode->CreateComponent<CollisionShape>();
        shape->SetSphere(1.0f);
        body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);
    }
}

void Physics::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    float timeStep = eventData[P_TIMESTEP].GetFloat();

    MoveCamera(timeStep);
}

void Physics::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    if (drawDebug_)
    {
        DebugRenderer *debugRenderer = scene_->GetComponent<DebugRenderer>();
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(debugRenderer, true);
    }
}

