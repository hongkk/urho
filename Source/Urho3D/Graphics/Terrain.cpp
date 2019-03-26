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

#include "../Precompiled.h"

#include "../Core/Context.h"
#include "../Core/Profiler.h"
#include "../Graphics/DrawableEvents.h"
#include "../Graphics/Geometry.h"
#include "../Graphics/IndexBuffer.h"
#include "../Graphics/Material.h"
#include "../Graphics/Octree.h"
#include "../Graphics/Terrain.h"
#include "../Graphics/TerrainPatch.h"
#include "../Graphics/VertexBuffer.h"
#include "../IO/Log.h"
#include "../Resource/Image.h"
#include "../Resource/ResourceCache.h"
#include "../Resource/ResourceEvents.h"
#include "../Scene/Node.h"
#include "../Scene/Scene.h"

#include "../DebugNew.h"

namespace Urho3D
{

extern const char* GEOMETRY_CATEGORY;

static const Vector3 DEFAULT_SPACING(1.0f, 0.25f, 1.0f);
static const unsigned MIN_LOD_LEVELS = 1;
static const unsigned MAX_LOD_LEVELS = 4;
static const int DEFAULT_PATCH_SIZE = 32;
static const int MIN_PATCH_SIZE = 4;
static const int MAX_PATCH_SIZE = 128;
static const unsigned STITCH_NORTH = 1;
static const unsigned STITCH_SOUTH = 2;
static const unsigned STITCH_WEST = 4;
static const unsigned STITCH_EAST = 8;

// 调整updateRegion 的合适大小
inline void GrowUpdateRegion(IntRect& updateRegion, int x, int y)
{
    if (updateRegion.left_ < 0)
    {
        updateRegion.left_ = updateRegion.right_ = x;
        updateRegion.top_ = updateRegion.bottom_ = y;
    }
    else
    {
        if (x < updateRegion.left_)
            updateRegion.left_ = x;
        if (x > updateRegion.right_)
            updateRegion.right_ = x;
        if (y < updateRegion.top_)
            updateRegion.top_ = y;
        if (y > updateRegion.bottom_)
            updateRegion.bottom_ = y;
    }
}

Terrain::Terrain(Context* context) :
    Component(context),
    indexBuffer_(new IndexBuffer(context)),
    spacing_(DEFAULT_SPACING),
    lastSpacing_(Vector3::ZERO),
    patchWorldOrigin_(Vector2::ZERO),
    patchWorldSize_(Vector2::ZERO),
    numVertices_(IntVector2::ZERO),
    lastNumVertices_(IntVector2::ZERO),
    numPatches_(IntVector2::ZERO),
    patchSize_(DEFAULT_PATCH_SIZE),
    lastPatchSize_(0),
    numLodLevels_(1),
    maxLodLevels_(MAX_LOD_LEVELS),
    occlusionLodLevel_(M_MAX_UNSIGNED),
    smoothing_(false),
    visible_(true),
    castShadows_(false),
    occluder_(false),
    occludee_(true),
    viewMask_(DEFAULT_VIEWMASK),
    lightMask_(DEFAULT_LIGHTMASK),
    shadowMask_(DEFAULT_SHADOWMASK),
    zoneMask_(DEFAULT_ZONEMASK),
    drawDistance_(0.0f),
    shadowDistance_(0.0f),
    lodBias_(1.0f),
    maxLights_(0),
    northID_(0),
    southID_(0),
    westID_(0),
    eastID_(0),
    recreateTerrain_(false),
    neighborsDirty_(false)
{
    indexBuffer_->SetShadowed(true);
}

Terrain::~Terrain()
{
}

void Terrain::RegisterObject(Context* context)
{
    context->RegisterFactory<Terrain>(GEOMETRY_CATEGORY);

    URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);
    URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Height Map", GetHeightMapAttr, SetHeightMapAttr, ResourceRef, ResourceRef(Image::GetTypeStatic()),
        AM_DEFAULT);
    URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Material", GetMaterialAttr, SetMaterialAttr, ResourceRef, ResourceRef(Material::GetTypeStatic()),
        AM_DEFAULT);
    URHO3D_ATTRIBUTE("North Neighbor NodeID", unsigned, northID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("South Neighbor NodeID", unsigned, southID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("West Neighbor NodeID", unsigned, westID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("East Neighbor NodeID", unsigned, eastID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Vertex Spacing", Vector3, spacing_, DEFAULT_SPACING, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Patch Size", GetPatchSize, SetPatchSizeAttr, int, DEFAULT_PATCH_SIZE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max LOD Levels", GetMaxLodLevels, SetMaxLodLevelsAttr, unsigned, MAX_LOD_LEVELS, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Smooth Height Map", bool, smoothing_, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Is Occluder", IsOccluder, SetOccluder, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Can Be Occluded", IsOccludee, SetOccludee, bool, true, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Cast Shadows", GetCastShadows, SetCastShadows, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Draw Distance", GetDrawDistance, SetDrawDistance, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Shadow Distance", GetShadowDistance, SetShadowDistance, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("LOD Bias", GetLodBias, SetLodBias, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max Lights", GetMaxLights, SetMaxLights, unsigned, 0, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("View Mask", GetViewMask, SetViewMask, unsigned, DEFAULT_VIEWMASK, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Light Mask", GetLightMask, SetLightMask, unsigned, DEFAULT_LIGHTMASK, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Shadow Mask", GetShadowMask, SetShadowMask, unsigned, DEFAULT_SHADOWMASK, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Zone Mask", GetZoneMask, SetZoneMask, unsigned, DEFAULT_ZONEMASK, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Occlusion LOD level", GetOcclusionLodLevel, SetOcclusionLodLevelAttr, unsigned, M_MAX_UNSIGNED, AM_DEFAULT);
}

void Terrain::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
{
    Serializable::OnSetAttribute(attr, src);

    // Change of any non-accessor attribute requires recreation of the terrain, or setting the neighbor terrains
    if (!attr.accessor_)
    {
        if (attr.mode_ & AM_NODEID)
            neighborsDirty_ = true;
        else
            recreateTerrain_ = true;
    }
}

void Terrain::ApplyAttributes()
{
    if (recreateTerrain_)
        CreateGeometry();

    if (neighborsDirty_)
    {
        Scene* scene = GetScene();
        Node* north = scene ? scene->GetNode(northID_) : (Node*)0;
        Node* south = scene ? scene->GetNode(southID_) : (Node*)0;
        Node* west = scene ? scene->GetNode(westID_) : (Node*)0;
        Node* east = scene ? scene->GetNode(eastID_) : (Node*)0;
        Terrain* northTerrain = north ? north->GetComponent<Terrain>() : (Terrain*)0;
        Terrain* southTerrain = south ? south->GetComponent<Terrain>() : (Terrain*)0;
        Terrain* westTerrain = west ? west->GetComponent<Terrain>() : (Terrain*)0;
        Terrain* eastTerrain = east ? east->GetComponent<Terrain>() : (Terrain*)0;
        SetNeighbors(northTerrain, southTerrain, westTerrain, eastTerrain);
        neighborsDirty_ = false;
    }
}

void Terrain::OnSetEnabled()
{
    bool enabled = IsEnabledEffective();

    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetEnabled(enabled);
    }
}

void Terrain::SetPatchSize(int size)
{
    if (size < MIN_PATCH_SIZE || size > MAX_PATCH_SIZE || !IsPowerOfTwo((unsigned)size))
        return;

    if (size != patchSize_)
    {
        patchSize_ = size;

        CreateGeometry();
        MarkNetworkUpdate();
    }
}

void Terrain::SetSpacing(const Vector3& spacing)
{
    if (spacing != spacing_)
    {
        spacing_ = spacing;

        CreateGeometry();
        MarkNetworkUpdate();
    }
}
// 设置最大Lod等级
void Terrain::SetMaxLodLevels(unsigned levels)
{
    levels = Clamp(levels, MIN_LOD_LEVELS, MAX_LOD_LEVELS);
    if (levels != maxLodLevels_)
    {
        maxLodLevels_ = levels;
        lastPatchSize_ = 0; // Force full recreate

        CreateGeometry();
        MarkNetworkUpdate();
    }
}

// 设置剃除Lod等级
void Terrain::SetOcclusionLodLevel(unsigned level)
{
    if (level != occlusionLodLevel_)
    {
        occlusionLodLevel_ = level;
        lastPatchSize_ = 0; // Force full recreate

        CreateGeometry();
        MarkNetworkUpdate();
    }
}

//设置平滑
void Terrain::SetSmoothing(bool enable)
{
    if (enable != smoothing_)
    {
        smoothing_ = enable;

        CreateGeometry();
        MarkNetworkUpdate();
    }
}

// 设置高度图
bool Terrain::SetHeightMap(Image* image)
{
    bool success = SetHeightMapInternal(image, true);

    MarkNetworkUpdate();
    return success;
}

void Terrain::SetMaterial(Material* material)
{
    material_ = material;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetMaterial(material);
    }

    MarkNetworkUpdate();
}

void Terrain::SetNorthNeighbor(Terrain* north)
{
    if (north == north_)
        return;

    if (north_ && north_->GetNode())
        UnsubscribeFromEvent(north_->GetNode(), E_TERRAINCREATED);

    north_ = north;
    if (north_ && north_->GetNode())
    {
        northID_ = north_->GetNode()->GetID();
        SubscribeToEvent(north_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }

    UpdateEdgePatchNeighbors();
    MarkNetworkUpdate();
}

void Terrain::SetSouthNeighbor(Terrain* south)
{
    if (south == south_)
        return;

    if (south_ && south_->GetNode())
        UnsubscribeFromEvent(south_->GetNode(), E_TERRAINCREATED);

    south_ = south;
    if (south_ && south_->GetNode())
    {
        southID_ = south_->GetNode()->GetID();
        SubscribeToEvent(south_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }

    UpdateEdgePatchNeighbors();
    MarkNetworkUpdate();
}

void Terrain::SetWestNeighbor(Terrain* west)
{
    if (west == west_)
        return;

    if (west_ && west_->GetNode())
        UnsubscribeFromEvent(west_->GetNode(), E_TERRAINCREATED);

    west_ = west;
    if (west_ && west_->GetNode())
    {
        westID_ = west_->GetNode()->GetID();
        SubscribeToEvent(west_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }

    UpdateEdgePatchNeighbors();
    MarkNetworkUpdate();
}

void Terrain::SetEastNeighbor(Terrain* east)
{
    if (east == east_)
        return;

    if (east_ && east_->GetNode())
        UnsubscribeFromEvent(east_->GetNode(), E_TERRAINCREATED);

    east_ = east;
    if (east_ && east_->GetNode())
    {
        eastID_ = east_->GetNode()->GetID();
        SubscribeToEvent(east_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }

    UpdateEdgePatchNeighbors();
    MarkNetworkUpdate();
}

void Terrain::SetNeighbors(Terrain* north, Terrain* south, Terrain* west, Terrain* east)
{
    if (north_ && north_->GetNode())
        UnsubscribeFromEvent(north_->GetNode(), E_TERRAINCREATED);
    if (south_ && south_->GetNode())
        UnsubscribeFromEvent(south_->GetNode(), E_TERRAINCREATED);
    if (west_ && west_->GetNode())
        UnsubscribeFromEvent(west_->GetNode(), E_TERRAINCREATED);
    if (east_ && east_->GetNode())
        UnsubscribeFromEvent(east_->GetNode(), E_TERRAINCREATED);

    north_ = north;
    if (north_ && north_->GetNode())
    {
        northID_ = north_->GetNode()->GetID();
        SubscribeToEvent(north_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }
    south_ = south;
    if (south_ && south_->GetNode())
    {
        southID_ = south_->GetNode()->GetID();
        SubscribeToEvent(south_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }
    west_ = west;
    if (west_ && west_->GetNode())
    {
        westID_ = west_->GetNode()->GetID();
        SubscribeToEvent(west_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }
    east_ = east;
    if (east_ && east_->GetNode())
    {
        eastID_ = east_->GetNode()->GetID();
        SubscribeToEvent(east_->GetNode(), E_TERRAINCREATED, URHO3D_HANDLER(Terrain, HandleNeighborTerrainCreated));
    }

    UpdateEdgePatchNeighbors();
    MarkNetworkUpdate();
}

// 设置当前Terrain与镜头的距离
void Terrain::SetDrawDistance(float distance)
{
    drawDistance_ = distance;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetDrawDistance(distance);
    }

    MarkNetworkUpdate();
}

void Terrain::SetShadowDistance(float distance)
{
    shadowDistance_ = distance;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetShadowDistance(distance);
    }

    MarkNetworkUpdate();
}

void Terrain::SetLodBias(float bias)
{
    lodBias_ = bias;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetLodBias(bias);
    }

    MarkNetworkUpdate();
}

void Terrain::SetViewMask(unsigned mask)
{
    viewMask_ = mask;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetViewMask(mask);
    }

    MarkNetworkUpdate();
}

void Terrain::SetLightMask(unsigned mask)
{
    lightMask_ = mask;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetLightMask(mask);
    }

    MarkNetworkUpdate();
}

void Terrain::SetShadowMask(unsigned mask)
{
    shadowMask_ = mask;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetShadowMask(mask);
    }

    MarkNetworkUpdate();
}

void Terrain::SetZoneMask(unsigned mask)
{
    zoneMask_ = mask;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetZoneMask(mask);
    }

    MarkNetworkUpdate();
}

void Terrain::SetMaxLights(unsigned num)
{
    maxLights_ = num;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetMaxLights(num);
    }

    MarkNetworkUpdate();
}

void Terrain::SetCastShadows(bool enable)
{
    castShadows_ = enable;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetCastShadows(enable);
    }

    MarkNetworkUpdate();
}

void Terrain::SetOccluder(bool enable)
{
    occluder_ = enable;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetOccluder(enable);
    }

    MarkNetworkUpdate();
}

void Terrain::SetOccludee(bool enable)
{
    occludee_ = enable;
    for (unsigned i = 0; i < patches_.Size(); ++i)
    {
        if (patches_[i])
            patches_[i]->SetOccludee(enable);
    }

    MarkNetworkUpdate();
}

void Terrain::ApplyHeightMap()
{
    if (heightMap_)
        CreateGeometry();
}

Image* Terrain::GetHeightMap() const
{
    return heightMap_;
}

Material* Terrain::GetMaterial() const
{
    return material_;
}

TerrainPatch* Terrain::GetPatch(unsigned index) const
{
    return index < patches_.Size() ? patches_[index] : (TerrainPatch*)0;
}

TerrainPatch* Terrain::GetPatch(int x, int z) const
{
    if (x < 0 || x >= numPatches_.x_ || z < 0 || z >= numPatches_.y_)
        return 0;
    else
        return GetPatch((unsigned)(z * numPatches_.x_ + x));
}

TerrainPatch* Terrain::GetNeighborPatch(int x, int z) const
{
    if (z >= numPatches_.y_ && north_)
        return north_->GetPatch(x, z - numPatches_.y_);
    else if (z < 0 && south_)
        return south_->GetPatch(x, z + south_->GetNumPatches().y_);
    else if (x < 0 && west_)
        return west_->GetPatch(x + west_->GetNumPatches().x_, z);
    else if (x >= numPatches_.x_ && east_)
        return east_->GetPatch(x - numPatches_.x_, z);
    else
        return GetPatch(x, z);
}

// 根据世界坐标获取高度值
float Terrain::GetHeight(const Vector3& worldPosition) const
{
    if (node_)
    {
        Vector3 position = node_->GetWorldTransform().Inverse() * worldPosition;
        float xPos = (position.x_ - patchWorldOrigin_.x_) / spacing_.x_;
        float zPos = (position.z_ - patchWorldOrigin_.y_) / spacing_.z_;
        float xFrac = Fract(xPos);
        float zFrac = Fract(zPos);
        float h1, h2, h3;

        if (xFrac + zFrac >= 1.0f)
        {
            h1 = GetRawHeight((unsigned)xPos + 1, (unsigned)zPos + 1);
            h2 = GetRawHeight((unsigned)xPos, (unsigned)zPos + 1);
            h3 = GetRawHeight((unsigned)xPos + 1, (unsigned)zPos);
            xFrac = 1.0f - xFrac;
            zFrac = 1.0f - zFrac;
        }
        else
        {
            h1 = GetRawHeight((unsigned)xPos, (unsigned)zPos);
            h2 = GetRawHeight((unsigned)xPos + 1, (unsigned)zPos);
            h3 = GetRawHeight((unsigned)xPos, (unsigned)zPos + 1);
        }

        float h = h1 * (1.0f - xFrac - zFrac) + h2 * xFrac + h3 * zFrac;
        /// \todo This assumes that the terrain scene node is upright
        return node_->GetWorldScale().y_ * h + node_->GetWorldPosition().y_;
    }
    else
        return 0.0f;
}

// 根据世界坐标获取法线值
Vector3 Terrain::GetNormal(const Vector3& worldPosition) const
{
    if (node_)
    {
        Vector3 position = node_->GetWorldTransform().Inverse() * worldPosition;
        float xPos = (position.x_ - patchWorldOrigin_.x_) / spacing_.x_;
        float zPos = (position.z_ - patchWorldOrigin_.y_) / spacing_.z_;
        float xFrac = Fract(xPos);
        float zFrac = Fract(zPos);
        Vector3 n1, n2, n3;

        if (xFrac + zFrac >= 1.0f)
        {
            n1 = GetRawNormal((unsigned)xPos + 1, (unsigned)zPos + 1);
            n2 = GetRawNormal((unsigned)xPos, (unsigned)zPos + 1);
            n3 = GetRawNormal((unsigned)xPos + 1, (unsigned)zPos);
            xFrac = 1.0f - xFrac;
            zFrac = 1.0f - zFrac;
        }
        else
        {
            n1 = GetRawNormal((unsigned)xPos, (unsigned)zPos);
            n2 = GetRawNormal((unsigned)xPos + 1, (unsigned)zPos);
            n3 = GetRawNormal((unsigned)xPos, (unsigned)zPos + 1);
        }

        Vector3 n = (n1 * (1.0f - xFrac - zFrac) + n2 * xFrac + n3 * zFrac).Normalized();
        return node_->GetWorldRotation() * n;
    }
    else
        return Vector3::UP;
}

IntVector2 Terrain::WorldToHeightMap(const Vector3& worldPosition) const
{
    if (!node_)
        return IntVector2::ZERO;

    Vector3 position = node_->GetWorldTransform().Inverse() * worldPosition;
    int xPos = (int)((position.x_ - patchWorldOrigin_.x_) / spacing_.x_ + 0.5f);
    int zPos = (int)((position.z_ - patchWorldOrigin_.y_) / spacing_.z_ + 0.5f);
    xPos = Clamp(xPos, 0, numVertices_.x_ - 1);
    zPos = Clamp(zPos, 0, numVertices_.y_ - 1);

    return IntVector2(xPos, numVertices_.y_ - 1 - zPos);
}

Vector3 Terrain::HeightMapToWorld(const IntVector2& pixelPosition) const
{
    if (!node_)
        return Vector3::ZERO;

    IntVector2 pos(pixelPosition.x_, numVertices_.y_ - 1 - pixelPosition.y_);
    float xPos = (float)(pos.x_ * spacing_.x_ + patchWorldOrigin_.x_);
    float zPos = (float)(pos.y_ * spacing_.z_ + patchWorldOrigin_.y_);
    Vector3 lPos(xPos, 0.0f, zPos);
    Vector3 wPos = node_->GetWorldTransform() * lPos;
    wPos.y_ = GetHeight(wPos);

    return wPos;
}


//生成TerrainPatch的顶点数据
void Terrain::CreatePatchGeometry(TerrainPatch* patch)
{
    URHO3D_PROFILE(CreatePatchGeometry);

    unsigned row = (unsigned)(patchSize_ + 1);
    VertexBuffer* vertexBuffer = patch->GetVertexBuffer();
    Geometry* geometry = patch->GetGeometry();
    Geometry* maxLodGeometry = patch->GetMaxLodGeometry();
    Geometry* occlusionGeometry = patch->GetOcclusionGeometry();

    if (vertexBuffer->GetVertexCount() != row * row)
        vertexBuffer->SetSize(row * row, MASK_POSITION | MASK_NORMAL | MASK_TEXCOORD1 | MASK_TANGENT);

    SharedArrayPtr<unsigned char> cpuVertexData(new unsigned char[row * row * sizeof(Vector3)]);
    SharedArrayPtr<unsigned char> occlusionCpuVertexData(new unsigned char[row * row * sizeof(Vector3)]);

    float* vertexData = (float*)vertexBuffer->Lock(0, vertexBuffer->GetVertexCount());
    float* positionData = (float*)cpuVertexData.Get();
    float* occlusionData = (float*)occlusionCpuVertexData.Get();
    BoundingBox box;

    unsigned occlusionLevel = occlusionLodLevel_;
    if (occlusionLevel > numLodLevels_ - 1)
        occlusionLevel = numLodLevels_ - 1;

    if (vertexData)
    {
        const IntVector2& coords = patch->GetCoordinates();
        int lodExpand = (1 << (occlusionLevel)) - 1;
        int halfLodExpand = (1 << (occlusionLevel)) / 2;

        for (int z = 0; z <= patchSize_; ++z)
        {
            for (int x = 0; x <= patchSize_; ++x)
            {
				//xPos zPos是在Terrain中的真实位置，这里算出来只是为了下面计算GetRawHeight(xPos, zPos)时，计算当前点在高度图中的位置
                int xPos = coords.x_ * patchSize_ + x;
                int zPos = coords.y_ * patchSize_ + z;

				// 这里存入vbo中的顶点数据是相对位置，第一个点是（0，height1,0),第二个点是 （100，height2,0) 依此类推
				// 注意这里每个点的高度都是不一样的，这也是为什么所有TerrainPatch的索引数据是共享的，但顶点vbo数据不能共享
                // Position
                Vector3 position((float)x * spacing_.x_, GetRawHeight(xPos, zPos), (float)z * spacing_.z_);
                *vertexData++ = position.x_;
                *vertexData++ = position.y_;
                *vertexData++ = position.z_;
                *positionData++ = position.x_;
                *positionData++ = position.y_;
                *positionData++ = position.z_;

                box.Merge(position);
				//	对于作为遮挡LOD一部分的顶点，计算相邻区域的最小高度，以防止由于遮挡LOD和可见LOD之间的不准确而导致假阳性遮挡。
                // For vertices that are part of the occlusion LOD, calculate the minimum height in the neighborhood
                // to prevent false positive occlusion due to inaccuracy between occlusion LOD & visible LOD
                float minHeight = position.y_;
                if (halfLodExpand > 0 && (x & lodExpand) == 0 && (z & lodExpand) == 0)
                {
                    int minX = Max(xPos - halfLodExpand, 0);
                    int maxX = Min(xPos + halfLodExpand, numVertices_.x_ - 1);
                    int minZ = Max(zPos - halfLodExpand, 0);
                    int maxZ = Min(zPos + halfLodExpand, numVertices_.y_ - 1);
                    for (int nZ = minZ; nZ <= maxZ; ++nZ)
                    {
                        for (int nX = minX; nX <= maxX; ++nX)
                            minHeight = Min(minHeight, GetRawHeight(nX, nZ));
                    }
                }
                *occlusionData++ = position.x_;
                *occlusionData++ = minHeight;
                *occlusionData++ = position.z_;

                // Normal
                Vector3 normal = GetRawNormal(xPos, zPos);
                *vertexData++ = normal.x_;
                *vertexData++ = normal.y_;
                *vertexData++ = normal.z_;

                // Texture coordinate
                Vector2 texCoord((float)xPos / (float)(numVertices_.x_ - 1), 1.0f - (float)zPos / (float)(numVertices_.y_ - 1));
                *vertexData++ = texCoord.x_;
                *vertexData++ = texCoord.y_;

                // Tangent
                Vector3 xyz = (Vector3::RIGHT - normal * normal.DotProduct(Vector3::RIGHT)).Normalized();
                *vertexData++ = xyz.x_;
                *vertexData++ = xyz.y_;
                *vertexData++ = xyz.z_;
                *vertexData++ = 1.0f;
            }
        }

        vertexBuffer->Unlock();
        vertexBuffer->ClearDataLost();
    }

    patch->SetBoundingBox(box);

    if (drawRanges_.Size())
    {
        unsigned occlusionDrawRange = occlusionLevel << 4;

        geometry->SetIndexBuffer(indexBuffer_);
        geometry->SetDrawRange(TRIANGLE_LIST, drawRanges_[0].first_, drawRanges_[0].second_, false);
        geometry->SetRawVertexData(cpuVertexData, MASK_POSITION);
        maxLodGeometry->SetIndexBuffer(indexBuffer_);
        maxLodGeometry->SetDrawRange(TRIANGLE_LIST, drawRanges_[0].first_, drawRanges_[0].second_, false);
        maxLodGeometry->SetRawVertexData(cpuVertexData, MASK_POSITION);
        occlusionGeometry->SetIndexBuffer(indexBuffer_);
        occlusionGeometry->SetDrawRange(TRIANGLE_LIST, drawRanges_[occlusionDrawRange].first_, drawRanges_[occlusionDrawRange].second_, false);
        occlusionGeometry->SetRawVertexData(occlusionCpuVertexData, MASK_POSITION);
    }

    patch->ResetLod();
}

void Terrain::UpdatePatchLod(TerrainPatch* patch)
{
    Geometry* geometry = patch->GetGeometry();

    // All LOD levels except the coarsest have 16 versions for stitching
    unsigned lodLevel = patch->GetLodLevel();
    unsigned drawRangeIndex = lodLevel << 4;
    if (lodLevel < numLodLevels_ - 1)
    {
        TerrainPatch* north = patch->GetNorthPatch();
        TerrainPatch* south = patch->GetSouthPatch();
        TerrainPatch* west = patch->GetWestPatch();
        TerrainPatch* east = patch->GetEastPatch();

        if (north && north->GetLodLevel() > lodLevel)
            drawRangeIndex |= STITCH_NORTH;
        if (south && south->GetLodLevel() > lodLevel)
            drawRangeIndex |= STITCH_SOUTH;
        if (west && west->GetLodLevel() > lodLevel)
            drawRangeIndex |= STITCH_WEST;
        if (east && east->GetLodLevel() > lodLevel)
            drawRangeIndex |= STITCH_EAST;
    }

    if (drawRangeIndex < drawRanges_.Size())
        geometry->SetDrawRange(TRIANGLE_LIST, drawRanges_[drawRangeIndex].first_, drawRanges_[drawRangeIndex].second_, false);
}

void Terrain::SetMaterialAttr(const ResourceRef& value)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    SetMaterial(cache->GetResource<Material>(value.name_));
}

void Terrain::SetHeightMapAttr(const ResourceRef& value)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Image* image = cache->GetResource<Image>(value.name_);
    SetHeightMapInternal(image, false);
}

void Terrain::SetPatchSizeAttr(int value)
{
    if (value < MIN_PATCH_SIZE || value > MAX_PATCH_SIZE || !IsPowerOfTwo((unsigned)value))
        return;

    if (value != patchSize_)
    {
        patchSize_ = value;
        recreateTerrain_ = true;
    }
}

void Terrain::SetMaxLodLevelsAttr(unsigned value)
{
    value = Clamp(value, MIN_LOD_LEVELS, MAX_LOD_LEVELS);

    if (value != maxLodLevels_)
    {
        maxLodLevels_ = value;
        lastPatchSize_ = 0; // Force full recreate
        recreateTerrain_ = true;
    }
}

void Terrain::SetOcclusionLodLevelAttr(unsigned value)
{
    if (value != occlusionLodLevel_)
    {
        occlusionLodLevel_ = value;
        lastPatchSize_ = 0; // Force full recreate
        recreateTerrain_ = true;
    }
}

ResourceRef Terrain::GetMaterialAttr() const
{
    return GetResourceRef(material_, Material::GetTypeStatic());
}

ResourceRef Terrain::GetHeightMapAttr() const
{
    return GetResourceRef(heightMap_, Image::GetTypeStatic());
}

//生成地形顶点数据
void Terrain::CreateGeometry()
{
    recreateTerrain_ = false;

    if (!node_)
        return;

    URHO3D_PROFILE(CreateTerrainGeometry);

    unsigned prevNumPatches = patches_.Size();
	//确定 lod 的等级数量
    // Determine number of LOD levels
    unsigned lodSize = (unsigned)patchSize_;
    numLodLevels_ = 1;
    while (lodSize > MIN_PATCH_SIZE && numLodLevels_ < maxLodLevels_)
    {
        lodSize >>= 1;
        ++numLodLevels_;
    }
	// 确定所有的 terrain 大小
    // Determine total terrain size
    patchWorldSize_ = Vector2(spacing_.x_ * (float)patchSize_, spacing_.z_ * (float)patchSize_);
    bool updateAll = false;
	// 处理高度图数据
    if (heightMap_)
    {
        numPatches_ = IntVector2((heightMap_->GetWidth() - 1) / patchSize_, (heightMap_->GetHeight() - 1) / patchSize_);
        numVertices_ = IntVector2(numPatches_.x_ * patchSize_ + 1, numPatches_.y_ * patchSize_ + 1);
        patchWorldOrigin_ =
            Vector2(-0.5f * (float)numPatches_.x_ * patchWorldSize_.x_, -0.5f * (float)numPatches_.y_ * patchWorldSize_.y_);
        if (numVertices_ != lastNumVertices_ || lastSpacing_ != spacing_ || patchSize_ != lastPatchSize_)
            updateAll = true;
        unsigned newDataSize = (unsigned)(numVertices_.x_ * numVertices_.y_);
		// 如果terrain大小惊变，要重新生成高度图数据
        // Create new height data if terrain size changed
        if (!heightData_ || updateAll)
            heightData_ = new float[newDataSize];

		// 如果smoothing开启，则创建多一个 sourceHeightData_ 存储高度图数据
        // Ensure that the source (unsmoothed) data exists if smoothing is active
        if (smoothing_ && (!sourceHeightData_ || updateAll))
        {
            sourceHeightData_ = new float[newDataSize];
            updateAll = true;
        }
        else if (!smoothing_)
            sourceHeightData_.Reset();
    }
    else
    {
        numPatches_ = IntVector2::ZERO;
        numVertices_ = IntVector2::ZERO;
        patchWorldOrigin_ = Vector2::ZERO;
        heightData_.Reset();
        sourceHeightData_.Reset();
    }

    lastNumVertices_ = numVertices_;
    lastPatchSize_ = patchSize_;
    lastSpacing_ = spacing_;
	// 删除旧的 patch 节点
    // Remove old patch nodes which are not needed
    if (updateAll)
    {
        URHO3D_PROFILE(RemoveOldPatches);

        PODVector<Node*> oldPatchNodes;
        node_->GetChildrenWithComponent<TerrainPatch>(oldPatchNodes);
        for (PODVector<Node*>::Iterator i = oldPatchNodes.Begin(); i != oldPatchNodes.End(); ++i)
        {
            bool nodeOk = false;
            Vector<String> coords = (*i)->GetName().Substring(6).Split('_');
            if (coords.Size() == 2)
            {
                int x = ToInt(coords[0]);
                int z = ToInt(coords[1]);
                if (x < numPatches_.x_ && z < numPatches_.y_)
                    nodeOk = true;
            }

            if (!nodeOk)
                node_->RemoveChild(*i);
        }
    }
	//
    // Keep track of which patches actually need an update
    PODVector<bool> dirtyPatches((unsigned)(numPatches_.x_ * numPatches_.y_));
    for (unsigned i = 0; i < dirtyPatches.Size(); ++i)
        dirtyPatches[i] = updateAll;

    patches_.Clear();

	// 填充高度图数据到 heightData_ 或 sourceHeightData_
    if (heightMap_)
    {
        // Copy heightmap data
        const unsigned char* src = heightMap_->GetData();
        float* dest = smoothing_ ? sourceHeightData_ : heightData_;
        unsigned imgComps = heightMap_->GetComponents();
        unsigned imgRow = heightMap_->GetWidth() * imgComps;
        IntRect updateRegion(-1, -1, -1, -1);

        if (imgComps == 1)
        {
            URHO3D_PROFILE(CopyHeightData);

            for (int z = 0; z < numVertices_.y_; ++z)
            {
                for (int x = 0; x < numVertices_.x_; ++x)
                {
                    float newHeight = (float)src[imgRow * (numVertices_.y_ - 1 - z) + x] * spacing_.y_;

                    if (updateAll)
                        *dest = newHeight;
                    else
                    {
                        if (*dest != newHeight)
                        {
                            *dest = newHeight;
                            GrowUpdateRegion(updateRegion, x, z);
                        }
                    }

                    ++dest;
                }
            }
        }
        else
        {
            URHO3D_PROFILE(CopyHeightData);

            // If more than 1 component, use the green channel for more accuracy
            for (int z = 0; z < numVertices_.y_; ++z)
            {
                for (int x = 0; x < numVertices_.x_; ++x)
                {
                    float newHeight = ((float)src[imgRow * (numVertices_.y_ - 1 - z) + imgComps * x] +
                                       (float)src[imgRow * (numVertices_.y_ - 1 - z) + imgComps * x + 1] / 256.0f) * spacing_.y_;

                    if (updateAll)
                        *dest = newHeight;
                    else
                    {
                        if (*dest != newHeight)
                        {
                            *dest = newHeight;
                            GrowUpdateRegion(updateRegion, x, z);
                        }
                    }

                    ++dest;
                }
            }
        }
		// 如果更新一个区域的高度图，检查一下哪个与之相关的patch需要更新
        // If updating a region of the heightmap, check which patches change
        if (!updateAll)
        {
            int lodExpand = 1 << (numLodLevels_ - 1);
            // Expand the right & bottom 1 pixel more, as patches share vertices at the edge
            updateRegion.left_ -= lodExpand;
            updateRegion.right_ += lodExpand + 1;
            updateRegion.top_ -= lodExpand;
            updateRegion.bottom_ += lodExpand + 1;

            int sX = Max(updateRegion.left_ / patchSize_, 0);
            int eX = Min(updateRegion.right_ / patchSize_, numPatches_.x_ - 1);
            int sY = Max(updateRegion.top_ / patchSize_, 0);
            int eY = Min(updateRegion.bottom_ / patchSize_, numPatches_.y_ - 1);
            for (int y = sY; y <= eY; ++y)
            {
                for (int x = sX; x <= eX; ++x)
                    dirtyPatches[y * numPatches_.x_ + x] = true;
            }
        }

        patches_.Reserve((unsigned)(numPatches_.x_ * numPatches_.y_));

        bool enabled = IsEnabledEffective();
		//生成patchNode节点和TerrainPatch
		// TerrainPatch 是真正最终被渲染出来成地形块
		// 一个Terrain会根据heightMap_->GetWidth() 和 patchSize_，把自已分成很多个 TerrainPatch
		// 例如在test.scene地图中，patchsize = 32,heightMap->width = 257
		// patchsize = 32 表示一个 TerrainPatch的大小为长宽32个spacing_
		// 所以 test.scene地形会把自己分成长宽为 8*8 = 64 个 TerrainPatch
        {
            URHO3D_PROFILE(CreatePatches);
			// 创建节点和TerrainPatch，并设置节点位置
            // Create patches and set node transforms
            for (int z = 0; z < numPatches_.y_; ++z)
            {
                for (int x = 0; x < numPatches_.x_; ++x)
                {
                    String nodeName = "Patch_" + String(x) + "_" + String(z);
                    Node* patchNode = node_->GetChild(nodeName);

                    if (!patchNode)
                    {
                        // Create the patch scene node as local and temporary so that it is not unnecessarily serialized to either
                        // file or replicated over the network
                        patchNode = node_->CreateTemporaryChild(nodeName, LOCAL);
                    }
					// patchWorldSize_ = spacing_.x_ * patchSize_ 一个TerrainPatch块的大小，
					// 如test.scene中为 100*32 = 3200
                    patchNode->SetPosition(Vector3(patchWorldOrigin_.x_ + (float)x * patchWorldSize_.x_, 0.0f,
                        patchWorldOrigin_.y_ + (float)z * patchWorldSize_.y_));

                    TerrainPatch* patch = patchNode->GetComponent<TerrainPatch>();
                    if (!patch)
                    {
                        patch = patchNode->CreateComponent<TerrainPatch>();
                        patch->SetOwner(this);
                        patch->SetCoordinates(IntVector2(x, z));

                        // Copy initial drawable parameters
                        patch->SetEnabled(enabled);
                        patch->SetMaterial(material_);
                        patch->SetDrawDistance(drawDistance_);
                        patch->SetShadowDistance(shadowDistance_);
                        patch->SetLodBias(lodBias_);
                        patch->SetViewMask(viewMask_);
                        patch->SetLightMask(lightMask_);
                        patch->SetShadowMask(shadowMask_);
                        patch->SetZoneMask(zoneMask_);
                        patch->SetMaxLights(maxLights_);
                        patch->SetCastShadows(castShadows_);
                        patch->SetOccluder(occluder_);
                        patch->SetOccludee(occludee_);
                    }

                    patches_.Push(WeakPtr<TerrainPatch>(patch));
                }
            }
        }
		// 生成共享的顶点索引数据
        // Create the shared index data
        if (updateAll)
            CreateIndexData();

        // Create vertex data for patches. First update smoothing to ensure normals are calculated correctly across patch borders
        if (smoothing_)
        {
            URHO3D_PROFILE(UpdateSmoothing);

            for (unsigned i = 0; i < patches_.Size(); ++i)
            {
                if (dirtyPatches[i])
                {
                    TerrainPatch* patch = patches_[i];
                    const IntVector2& coords = patch->GetCoordinates();
                    int startX = coords.x_ * patchSize_;
                    int endX = startX + patchSize_;
                    int startZ = coords.y_ * patchSize_;
                    int endZ = startZ + patchSize_;

                    for (int z = startZ; z <= endZ; ++z)
                    {
                        for (int x = startX; x <= endX; ++x)
                        {
                            float smoothedHeight = (
                                GetSourceHeight(x - 1, z - 1) + GetSourceHeight(x, z - 1) * 2.0f + GetSourceHeight(x + 1, z - 1) +
                                GetSourceHeight(x - 1, z) * 2.0f + GetSourceHeight(x, z) * 4.0f + GetSourceHeight(x + 1, z) * 2.0f +
                                GetSourceHeight(x - 1, z + 1) + GetSourceHeight(x, z + 1) * 2.0f + GetSourceHeight(x + 1, z + 1)
                            ) / 16.0f;

                            heightData_[z * numVertices_.x_ + x] = smoothedHeight;
                        }
                    }
                }
            }
        }
		// 为什么 索引数据要共享而顶点数据要每个TerrainPatch都生成一份？
		// 因为在每个TerrainPatch中虽然每个点的x,z的相对位置都是一样的，但是每个点的y都是不同的，也就是高度和法线信息是一直在变化的，所以顶点数据不能共享
		// 遍历所有 patchs，创建对应patch的顶点数据
        for (unsigned i = 0; i < patches_.Size(); ++i)
        {
            TerrainPatch* patch = patches_[i];

            if (dirtyPatches[i])
            {
                CreatePatchGeometry(patch);
                CalculateLodErrors(patch);
            }

            SetPatchNeighbors(patch);
        }
    }

    // Send event only if new geometry was generated, or the old was cleared
    if (patches_.Size() || prevNumPatches)
    {
        using namespace TerrainCreated;

        VariantMap& eventData = GetEventDataMap();
        eventData[P_NODE] = node_;
        node_->SendEvent(E_TERRAINCREATED, eventData);
    }
}

//生成顶点索引数据
// 这里只会生成单个 TerrainPatch 的索引数据，
// 因为所有TerrainPatch的顶点数据也是针对单个 TerrainPatch 生成的
void Terrain::CreateIndexData()
{
    URHO3D_PROFILE(CreateIndexData);

    PODVector<unsigned short> indices;
    drawRanges_.Clear();
    unsigned row = (unsigned)(patchSize_ + 1);
	// stitch  缝
	/*为每个LOD级别构建索引数据。 除最低点之外的每个LOD级别都可以从边缘缝合到下一个较低的LOD：
		北，南，西，东或它们的任意组合，需要16个不同版本的每个LOD级别的索引数据*/
    /* Build index data for each LOD level. Each LOD level except the lowest can stitch to the next lower LOD from the edges:
       north, south, west, east, or any combination of them, requiring 16 different versions of each LOD level's index data

       Normal edge:     Stitched edge:
       +----+----+      +---------+
       |\   |\   |      |\       /|
       | \  | \  |      | \     / |
       |  \ |  \ |      |  \   /  |
       |   \|   \|      |   \ /   |
       +----+----+      +----+----+
    */
	// 当i = 0 时，表示最密的那一级，前两个放入的三角开索引为（0，1，33）（33，34，1），四个边与相接边不同密度的情况有16种，1种情况是四个边都一样，1种情况是四个边都不一样，4种情况是3个边不一样，6种情况是二个边不一样，4种情况是1个边不一样
	// 当i = 1 时，表示第二级，前两个放入的三角形索引为 （0，2，66）（66，68，2） 以此类推
    for (unsigned i = 0; i < numLodLevels_; ++i)
    {
        unsigned combinations = (i < numLodLevels_ - 1) ? 16 : 1;
        int skip = 1 << i;

        for (unsigned j = 0; j < combinations; ++j)
        {
            unsigned indexStart = indices.Size();

            int zStart = 0;
            int xStart = 0;
            int zEnd = patchSize_;
            int xEnd = patchSize_;

            if (j & STITCH_NORTH)
                zEnd -= skip;
            if (j & STITCH_SOUTH)
                zStart += skip;
            if (j & STITCH_WEST)
                xStart += skip;
            if (j & STITCH_EAST)
                xEnd -= skip;

            // Build the main grid
            for (int z = zStart; z < zEnd; z += skip)
            {
                for (int x = xStart; x < xEnd; x += skip)
                {
                    indices.Push((unsigned short)((z + skip) * row + x));
                    indices.Push((unsigned short)(z * row + x + skip));
                    indices.Push((unsigned short)(z * row + x));
                    indices.Push((unsigned short)((z + skip) * row + x));
                    indices.Push((unsigned short)((z + skip) * row + x + skip));
                    indices.Push((unsigned short)(z * row + x + skip));
                }
            }

            // Build the north edge
            if (j & STITCH_NORTH)
            {
                int z = patchSize_ - skip;
                for (int x = 0; x < patchSize_; x += skip * 2)
                {
                    if (x > 0 || (j & STITCH_WEST) == 0)
                    {
                        indices.Push((unsigned short)((z + skip) * row + x));
                        indices.Push((unsigned short)(z * row + x + skip));
                        indices.Push((unsigned short)(z * row + x));
                    }
                    indices.Push((unsigned short)((z + skip) * row + x));
                    indices.Push((unsigned short)((z + skip) * row + x + 2 * skip));
                    indices.Push((unsigned short)(z * row + x + skip));
                    if (x < patchSize_ - skip * 2 || (j & STITCH_EAST) == 0)
                    {
                        indices.Push((unsigned short)((z + skip) * row + x + 2 * skip));
                        indices.Push((unsigned short)(z * row + x + 2 * skip));
                        indices.Push((unsigned short)(z * row + x + skip));
                    }
                }
            }

            // Build the south edge
            if (j & STITCH_SOUTH)
            {
                int z = 0;
                for (int x = 0; x < patchSize_; x += skip * 2)
                {
                    if (x > 0 || (j & STITCH_WEST) == 0)
                    {
                        indices.Push((unsigned short)((z + skip) * row + x));
                        indices.Push((unsigned short)((z + skip) * row + x + skip));
                        indices.Push((unsigned short)(z * row + x));
                    }
                    indices.Push((unsigned short)(z * row + x));
                    indices.Push((unsigned short)((z + skip) * row + x + skip));
                    indices.Push((unsigned short)(z * row + x + 2 * skip));
                    if (x < patchSize_ - skip * 2 || (j & STITCH_EAST) == 0)
                    {
                        indices.Push((unsigned short)((z + skip) * row + x + skip));
                        indices.Push((unsigned short)((z + skip) * row + x + 2 * skip));
                        indices.Push((unsigned short)(z * row + x + 2 * skip));
                    }
                }
            }

            // Build the west edge
            if (j & STITCH_WEST)
            {
                int x = 0;
                for (int z = 0; z < patchSize_; z += skip * 2)
                {
                    if (z > 0 || (j & STITCH_SOUTH) == 0)
                    {
                        indices.Push((unsigned short)(z * row + x));
                        indices.Push((unsigned short)((z + skip) * row + x + skip));
                        indices.Push((unsigned short)(z * row + x + skip));
                    }
                    indices.Push((unsigned short)((z + 2 * skip) * row + x));
                    indices.Push((unsigned short)((z + skip) * row + x + skip));
                    indices.Push((unsigned short)(z * row + x));
                    if (z < patchSize_ - skip * 2 || (j & STITCH_NORTH) == 0)
                    {
                        indices.Push((unsigned short)((z + 2 * skip) * row + x));
                        indices.Push((unsigned short)((z + 2 * skip) * row + x + skip));
                        indices.Push((unsigned short)((z + skip) * row + x + skip));
                    }
                }
            }

            // Build the east edge
            if (j & STITCH_EAST)
            {
                int x = patchSize_ - skip;
                for (int z = 0; z < patchSize_; z += skip * 2)
                {
                    if (z > 0 || (j & STITCH_SOUTH) == 0)
                    {
                        indices.Push((unsigned short)(z * row + x));
                        indices.Push((unsigned short)((z + skip) * row + x));
                        indices.Push((unsigned short)(z * row + x + skip));
                    }
                    indices.Push((unsigned short)((z + skip) * row + x));
                    indices.Push((unsigned short)((z + 2 * skip) * row + x + skip));
                    indices.Push((unsigned short)(z * row + x + skip));
                    if (z < patchSize_ - skip * 2 || (j & STITCH_NORTH) == 0)
                    {
                        indices.Push((unsigned short)((z + skip) * row + x));
                        indices.Push((unsigned short)((z + 2 * skip) * row + x));
                        indices.Push((unsigned short)((z + 2 * skip) * row + x + skip));
                    }
                }
            }

            drawRanges_.Push(MakePair(indexStart, indices.Size() - indexStart));
        }
    }

    indexBuffer_->SetSize(indices.Size(), false);
    indexBuffer_->SetData(&indices[0]);
}

float Terrain::GetRawHeight(int x, int z) const
{
    if (!heightData_)
        return 0.0f;

    x = Clamp(x, 0, numVertices_.x_ - 1);
    z = Clamp(z, 0, numVertices_.y_ - 1);
    return heightData_[z * numVertices_.x_ + x];
}

float Terrain::GetSourceHeight(int x, int z) const
{
    if (!sourceHeightData_)
        return 0.0f;

    x = Clamp(x, 0, numVertices_.x_ - 1);
    z = Clamp(z, 0, numVertices_.y_ - 1);
    return sourceHeightData_[z * numVertices_.x_ + x];
}

float Terrain::GetLodHeight(int x, int z, unsigned lodLevel) const
{
    unsigned offset = (unsigned)(1 << lodLevel);
    float divisor = (float)offset;
    float xFrac = (float)(x % offset) / divisor;
    float zFrac = (float)(z % offset) / divisor;
    float h1, h2, h3;

    if (xFrac + zFrac >= 1.0f)
    {
        h1 = GetRawHeight(x + offset, z + offset);
        h2 = GetRawHeight(x, z + offset);
        h3 = GetRawHeight(x + offset, z);
        xFrac = 1.0f - xFrac;
        zFrac = 1.0f - zFrac;
    }
    else
    {
        h1 = GetRawHeight(x, z);
        h2 = GetRawHeight(x + offset, z);
        h3 = GetRawHeight(x, z + offset);
    }

    return h1 * (1.0f - xFrac - zFrac) + h2 * xFrac + h3 * zFrac;
}

Vector3 Terrain::GetRawNormal(int x, int z) const
{
    float baseHeight = GetRawHeight(x, z);
    float nSlope = GetRawHeight(x, z - 1) - baseHeight;
    float neSlope = GetRawHeight(x + 1, z - 1) - baseHeight;
    float eSlope = GetRawHeight(x + 1, z) - baseHeight;
    float seSlope = GetRawHeight(x + 1, z + 1) - baseHeight;
    float sSlope = GetRawHeight(x, z + 1) - baseHeight;
    float swSlope = GetRawHeight(x - 1, z + 1) - baseHeight;
    float wSlope = GetRawHeight(x - 1, z) - baseHeight;
    float nwSlope = GetRawHeight(x - 1, z - 1) - baseHeight;
    float up = 0.5f * (spacing_.x_ + spacing_.z_);

    return (Vector3(0.0f, up, nSlope) +
            Vector3(-neSlope, up, neSlope) +
            Vector3(-eSlope, up, 0.0f) +
            Vector3(-seSlope, up, -seSlope) +
            Vector3(0.0f, up, -sSlope) +
            Vector3(swSlope, up, -swSlope) +
            Vector3(wSlope, up, 0.0f) +
            Vector3(nwSlope, up, nwSlope)).Normalized();
}

void Terrain::CalculateLodErrors(TerrainPatch* patch)
{
    URHO3D_PROFILE(CalculateLodErrors);

    const IntVector2& coords = patch->GetCoordinates();
    PODVector<float>& lodErrors = patch->GetLodErrors();
    lodErrors.Clear();
    lodErrors.Reserve(numLodLevels_);

    int xStart = coords.x_ * patchSize_;
    int zStart = coords.y_ * patchSize_;
    int xEnd = xStart + patchSize_;
    int zEnd = zStart + patchSize_;

    for (unsigned i = 0; i < numLodLevels_; ++i)
    {
        float maxError = 0.0f;
        int divisor = 1 << i;

        if (i > 0)
        {
            for (int z = zStart; z <= zEnd; ++z)
            {
                for (int x = xStart; x <= xEnd; ++x)
                {
                    if (x % divisor || z % divisor)
                    {
                        float error = Abs(GetLodHeight(x, z, i) - GetRawHeight(x, z));
                        maxError = Max(error, maxError);
                    }
                }
            }

            // Set error to be at least same as (half vertex spacing x LOD) to prevent horizontal stretches getting too inaccurate
            maxError = Max(maxError, 0.25f * (spacing_.x_ + spacing_.z_) * (float)(1 << i));
        }

        lodErrors.Push(maxError);
    }
}

void Terrain::SetPatchNeighbors(TerrainPatch* patch)
{
    if (!patch)
        return;

    const IntVector2& coords = patch->GetCoordinates();
    patch->SetNeighbors(GetNeighborPatch(coords.x_, coords.y_ + 1), GetNeighborPatch(coords.x_, coords.y_ - 1),
        GetNeighborPatch(coords.x_ - 1, coords.y_), GetNeighborPatch(coords.x_ + 1, coords.y_));
}

bool Terrain::SetHeightMapInternal(Image* image, bool recreateNow)
{
    if (image && image->IsCompressed())
    {
        URHO3D_LOGERROR("Can not use a compressed image as a terrain heightmap");
        return false;
    }

    // Unsubscribe from the reload event of previous image (if any), then subscribe to the new
    if (heightMap_)
        UnsubscribeFromEvent(heightMap_, E_RELOADFINISHED);
    if (image)
        SubscribeToEvent(image, E_RELOADFINISHED, URHO3D_HANDLER(Terrain, HandleHeightMapReloadFinished));

    heightMap_ = image;

    if (recreateNow)
        CreateGeometry();
    else
        recreateTerrain_ = true;

    return true;
}

void Terrain::HandleHeightMapReloadFinished(StringHash eventType, VariantMap& eventData)
{
    CreateGeometry();
}

void Terrain::HandleNeighborTerrainCreated(StringHash eventType, VariantMap& eventData)
{
    UpdateEdgePatchNeighbors();
}

void Terrain::UpdateEdgePatchNeighbors()
{
    for (int x = 1; x < numPatches_.x_ - 1; ++x)
    {
        SetPatchNeighbors(GetPatch(x, 0));
        SetPatchNeighbors(GetPatch(x, numPatches_.y_ - 1));
    }
    for (int z = 1; z < numPatches_.y_ - 1; ++z)
    {
        SetPatchNeighbors(GetPatch(0, z));
        SetPatchNeighbors(GetPatch(numPatches_.x_ - 1, z));
    }

    SetPatchNeighbors(GetPatch(0, 0));
    SetPatchNeighbors(GetPatch(numPatches_.x_ - 1, 0));
    SetPatchNeighbors(GetPatch(0, numPatches_.y_ - 1));
    SetPatchNeighbors(GetPatch(numPatches_.x_ - 1, numPatches_.y_ - 1));
}

}
