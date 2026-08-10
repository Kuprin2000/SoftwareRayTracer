#pragma once
#include <vector>
#include <array>
#include <string>
#include <cinttypes>
#include "LiteMath/LiteMath.h"

namespace Octree
{
	using LiteMath::float3;

	struct SdfOctreeNode
	{
		std::array<float, 8> values;
		unsigned offset = 0;			// offset for children (they are stored together). 0 offset means it's a leaf
	};

	struct SelectedNode
	{
		int m_node_id = -1;
		float m_near_intersection = FLT_MAX;
		float m_far_intersection = FLT_MAX;
		float3 m_min_corner;
		float3 m_max_corner;
		std::array<float, 8> m_values = { 0.0f };

		SelectedNode() = default;

		SelectedNode(int node_id, float near_intersection, float far_intersection,
			const float3& min_corner, const float3& max_corner, const std::array<float, 8>& values);
	};

	enum class SamplingMode
	{
		SPHERE_TRACING,
		ANALYTIC
	};

	enum class SamplingResult
	{
		NO_INTERSECTION,
		INTERSECTION,
		STEP
	};

	struct SdfOctree
	{
	public:
		_NODISCARD bool contains(const float3& point) const;

		_NODISCARD SamplingResult sample(const float3& ray_start,
			const float3& ray_dir, SamplingMode mode, float3& point, float3& normal) const;

	public:
		std::vector<SdfOctreeNode> m_nodes;

	private:
		struct QueryContext
		{
			uint8_t a;
			float3 m_min_corner = { -1.0f, -1.0f, -1.0f };
			float3 m_max_corner = { 1.0f, 1.0f, 1.0f };

			explicit QueryContext(uint8_t a);

			void goToChild(size_t child_index);
		};

		_NODISCARD int getChildOffset(size_t parent_index, size_t child_index) const;

		_NODISCARD int firstNode(float tx0, float ty0, float tz0, float txm, float tym, float tzm) const;

		_NODISCARD int newNode(float t1, int a, float t2, int b, float t3, int c) const;

		_NODISCARD SelectedNode selectNode(int32_t idx, float tx0,
			float ty0, float tz0, float tx1, float ty1, float tz1, QueryContext& ctx) const;

		_NODISCARD SamplingResult calculateIntersection(SelectedNode selected_node,
			const float3& ray_start, const float3& ray_dir, SamplingMode mode, float3& point, float3& normal) const;

		_NODISCARD SamplingResult calculateIntersectionSphereTracing(SelectedNode selected_node, const float3& ray_start,
			const float3& ray_dir, float3& point, float3& normal) const;

		_NODISCARD SamplingResult calculateIntersectionAnalytic(SelectedNode selected_node, const float3& ray_start,
			const float3& ray_dir, float3& point, float3& normal) const;

	private:
		const float3 m_size = { 2.0f, 2.0f, 2.0f };
		const float3 m_min_corner = { -1.0f, -1.0f, -1.0f };
		const float3 m_max_corner = { 1.0f, 1.0f, 1.0f };
	};

	void save_sdf_octree(const SdfOctree& scene, const std::string& path);

	void load_sdf_octree(SdfOctree& scene, const std::string& path);
}