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

#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/SoftBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>

#include "SoftBodyHelper.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
SoftBodyHelper::SoftBodyHelper(Context* context)
    : Component(context)
    , makeType_(Make_Invalid)
    , spacing_(1.0f)
{
}

void SoftBodyHelper::RegisterObject(Context* context)
{
    context->RegisterFactory<SoftBodyHelper>();
   
    URHO3D_ATTRIBUTE("Make Type", unsigned, makeType_, Make_Invalid, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Spacing", float, spacing_, 1.0f, AM_DEFAULT);
}

void SoftBodyHelper::ApplyAttributes()
{
    if (makeType_ == Make_Sticks)
    {
        MakeSticks();
    }
}

void SoftBodyHelper::MakeSticks()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Model *model = cache->GetResource<Model>("Models/Stick.mdl");
    Material *material = cache->GetResource<Material>("Materials/uvMat.xml");
    Vector3 nodepos = node_->GetWorldPosition();

    int n = 16;
    int sg = 4;
    float sz = 5.0f;
    float hg = 2.0f;

    for ( int y = 0; y < n; ++y )
    {
        for ( int x = 0; x < n; ++x )
        {
            Vector3 pos = nodepos + Vector3((float)x * spacing_, 0.0f, (float)y * spacing_);
            Node *node = GetScene()->CreateChild();
            node->SetPosition(pos);

            StaticModel *statModel = node->CreateComponent<StaticModel>();
            statModel->SetModel(model);
            statModel->SetMaterial(material);

            SoftBody *softbody = node->CreateComponent<SoftBody>();
            softbody->SetSoftBodyType(SOFTBODY_STICK);
            softbody->SetMass(0.01f);
            softbody->SetDeactivationVelocity(0.001f);
        }
    }
}



