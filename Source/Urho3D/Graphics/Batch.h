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

#pragma once

#include "../Container/Ptr.h"
#include "../Graphics/Drawable.h"
#include "../Graphics/Material.h"
#include "../Math/MathDefs.h"
#include "../Math/Matrix3x4.h"
#include "../Math/Rect.h"

namespace Urho3D
{

class Camera;
class Drawable;
class Geometry;
class Light;
class Material;
class Matrix3x4;
class Pass;
class ShaderVariation;
class Texture2D;
class VertexBuffer;
class View;
class Zone;
struct LightBatchQueue;

/// Queued 3D geometry draw call.
struct Batch
{
    /// Construct with defaults.
    Batch() :
        isBase_(false),
        lightQueue_(0)
    {
    }

    /// Construct from a drawable's source batch.
    Batch(const SourceBatch& rhs) :
        distance_(rhs.distance_),
        renderOrder_(rhs.material_ ? rhs.material_->GetRenderOrder() : DEFAULT_RENDER_ORDER),
        isBase_(false),
        geometry_(rhs.geometry_),
        material_(rhs.material_),
        worldTransform_(rhs.worldTransform_),
        numWorldTransforms_(rhs.numWorldTransforms_),
        instancingData_(rhs.instancingData_),
        lightQueue_(0),
        geometryType_(rhs.geometryType_)
    {
    }
	// 计算排序的键
    /// Calculate state sorting key, which consists of base pass flag, light, pass and geometry.
    void CalculateSortKey();
	//准备渲染
    /// Prepare for rendering.
    void Prepare(View* view, Camera* camera, bool setModelTransform, bool allowDepthWrite) const;
	//执行渲染
    /// Prepare and draw.
    void Draw(View* view, Camera* camera, bool allowDepthWrite) const;
	//排序的键
    /// State sorting key.
    unsigned long long sortKey_;
    /// Distance from camera.
	//到摄像机的距离
    float distance_;
	//8位 渲染顺序
    /// 8-bit render order modifier from material.
    unsigned char renderOrder_;
	// 8位 光源掩码 用于延迟渲染
    /// 8-bit light mask for stencil marking in deferred rendering.
    unsigned char lightMask_;
	//基础批次标识 表示渲染的时候不需要做光源优化
    /// Base batch flag. This tells to draw the object fully without light optimizations.
    bool isBase_;
	//顶点数据
    /// Geometry.
    Geometry* geometry_;
	//本批次相关材质
    /// Material.
    Material* material_;
	//世界矩阵，对于蒙皮模型来说是骨骼矩阵
    /// World transform(s). For a skinned model, these are the bone transforms.
    const Matrix3x4* worldTransform_;
	//世界矩阵的个数
    /// Number of world transforms.
    unsigned numWorldTransforms_;
	// 实例 不知道是什么意思
    /// Per-instance data. If not null, must contain enough data to fill instancing buffer.
    void* instancingData_;
    /// Zone.
    Zone* zone_;
    /// Light properties.
    LightBatchQueue* lightQueue_;
    /// Material pass.
    Pass* pass_;
    /// Vertex shader.
    ShaderVariation* vertexShader_;
    /// Pixel shader.
    ShaderVariation* pixelShader_;
    /// %Geometry type.
    GeometryType geometryType_;
};

//用于存储 Batch 中的每一个worldTransform_
/// Data for one geometry instance.
struct InstanceData
{
    /// Construct undefined.
    InstanceData()
    {
    }

    /// Construct with transform, instancing data and distance.
    InstanceData(const Matrix3x4* worldTransform, const void* instancingData, float distance) :
        worldTransform_(worldTransform),
        instancingData_(instancingData),
        distance_(distance)
    {
    }

    /// World transform.
    const Matrix3x4* worldTransform_;
    /// Instancing data buffer.
    const void* instancingData_;
    /// Distance from camera.
    float distance_;
};

// 实例化3d顶点数据的 drawcall (怎么理解呢）
/// Instanced 3D geometry draw call.
struct BatchGroup : public Batch
{
    /// Construct with defaults.
    BatchGroup() :
        startIndex_(M_MAX_UNSIGNED)
    {
    }

    /// Construct from a batch.
    BatchGroup(const Batch& batch) :
        Batch(batch),
        startIndex_(M_MAX_UNSIGNED)
    {
    }

    /// Destruct.
    ~BatchGroup()
    {
    }

    /// Add world transform(s) from a batch.
    void AddTransforms(const Batch& batch)
    {
        InstanceData newInstance;
        newInstance.distance_ = batch.distance_;
        newInstance.instancingData_ = batch.instancingData_;

        for (unsigned i = 0; i < batch.numWorldTransforms_; ++i)
        {
            newInstance.worldTransform_ = &batch.worldTransform_[i];
            instances_.Push(newInstance);
        }
    }

    /// Pre-set the instance data. Buffer must be big enough to hold all data.
    void SetInstancingData(void* lockedData, unsigned stride, unsigned& freeIndex);
    /// Prepare and draw.
    void Draw(View* view, Camera* camera, bool allowDepthWrite) const;

    /// Instance data.
    PODVector<InstanceData> instances_;
    /// Instance stream start index, or M_MAX_UNSIGNED if transforms not pre-set.
    unsigned startIndex_;
};

/// Instanced draw call grouping key.
struct BatchGroupKey
{
    /// Construct undefined.
    BatchGroupKey()
    {
    }

    /// Construct from a batch.
    BatchGroupKey(const Batch& batch) :
        zone_(batch.zone_),
        lightQueue_(batch.lightQueue_),
        pass_(batch.pass_),
        material_(batch.material_),
        geometry_(batch.geometry_),
        renderOrder_(batch.renderOrder_)
    {
    }

    /// Zone.
    Zone* zone_;
    /// Light properties.
    LightBatchQueue* lightQueue_;
    /// Material pass.
    Pass* pass_;
    /// Material.
    Material* material_;
    /// Geometry.
    Geometry* geometry_;
    /// 8-bit render order modifier from material.
    unsigned char renderOrder_;

    /// Test for equality with another batch group key.
    bool operator ==(const BatchGroupKey& rhs) const
    {
        return zone_ == rhs.zone_ && lightQueue_ == rhs.lightQueue_ && pass_ == rhs.pass_ && material_ == rhs.material_ &&
               geometry_ == rhs.geometry_ && renderOrder_ == rhs.renderOrder_;
    }

    /// Test for inequality with another batch group key.
    bool operator !=(const BatchGroupKey& rhs) const
    {
        return zone_ != rhs.zone_ || lightQueue_ != rhs.lightQueue_ || pass_ != rhs.pass_ || material_ != rhs.material_ ||
               geometry_ != rhs.geometry_ || renderOrder_ != rhs.renderOrder_;
    }

    /// Return hash value.
    unsigned ToHash() const;
};

/// Queue that contains both instanced and non-instanced draw calls.
struct BatchQueue
{
public:
	//开始使用之前清理原来的数据
    /// Clear for new frame by clearing all groups and batches.
    void Clear(int maxSortedInstances);
	// 对 非实例化 draw call 从后往前排序
    /// Sort non-instanced draw calls back to front.
    void SortBackToFront();
	// 对 实例化和非实例化 draw call 从前往后排序
    /// Sort instanced and non-instanced draw calls front to back.
    void SortFrontToBack();
    /// Sort batches front to back while also maintaining state sorting.
    void SortFrontToBack2Pass(PODVector<Batch*>& batches);
    /// Pre-set instance data of all groups. The vertex buffer must be big enough to hold all data.
    void SetInstancingData(void* lockedData, unsigned stride, unsigned& freeIndex);
    /// Draw.
    void Draw(View* view, Camera* camera, bool markToStencil, bool usingLightOptimization, bool allowDepthWrite) const;
    /// Return the combined amount of instances.
    unsigned GetNumInstances() const;

    /// Return whether the batch group is empty.
    bool IsEmpty() const { return batches_.Empty() && batchGroups_.Empty(); }

	// 需要实例化渲染的批次
    /// Instanced draw calls.
    HashMap<BatchGroupKey, BatchGroup> batchGroups_;
    /// Shader remapping table for 2-pass state and distance sort.
    HashMap<unsigned, unsigned> shaderRemapping_;
    /// Material remapping table for 2-pass state and distance sort.
    HashMap<unsigned short, unsigned short> materialRemapping_;
    /// Geometry remapping table for 2-pass state and distance sort.
    HashMap<unsigned short, unsigned short> geometryRemapping_;

	// 没排序的非实例化drawcall
    /// Unsorted non-instanced draw calls.
    PODVector<Batch> batches_;
	// 排序的非实例化drawcall
    /// Sorted non-instanced draw calls.
    PODVector<Batch*> sortedBatches_;
	// 没排序的实例化drawcall
    /// Sorted instanced draw calls.
    PODVector<BatchGroup*> sortedBatchGroups_;
    /// Maximum sorted instances.
    unsigned maxSortedInstances_;
    /// Whether the pass command contains extra shader defines.
    bool hasExtraDefines_;
    /// Vertex shader extra defines.
    String vsExtraDefines_;
    /// Pixel shader extra defines.
    String psExtraDefines_;
    /// Hash for vertex shader extra defines.
    StringHash vsExtraDefinesHash_;
    /// Hash for pixel shader extra defines.
    StringHash psExtraDefinesHash_;
};

/// Queue for shadow map draw calls
struct ShadowBatchQueue
{
    /// Shadow map camera.
    Camera* shadowCamera_;
    /// Shadow map viewport.
    IntRect shadowViewport_;
    /// Shadow caster draw calls.
    BatchQueue shadowBatches_;
    /// Directional light cascade near split distance.
    float nearSplit_;
    /// Directional light cascade far split distance.
    float farSplit_;
};

/// Queue for light related draw calls.
struct LightBatchQueue
{
    /// Per-pixel light.
    Light* light_;
    /// Light negative flag.
    bool negative_;
    /// Shadow map depth texture.
    Texture2D* shadowMap_;
    /// Lit geometry draw calls, base (replace blend mode)
    BatchQueue litBaseBatches_;
    /// Lit geometry draw calls, non-base (additive)
    BatchQueue litBatches_;
    /// Shadow map split queues.
    Vector<ShadowBatchQueue> shadowSplits_;
    /// Per-vertex lights.
    PODVector<Light*> vertexLights_;
    /// Light volume draw calls.
    PODVector<Batch> volumeBatches_;
};

}
