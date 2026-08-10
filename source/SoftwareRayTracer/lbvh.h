#pragma once
#include "bounding_box.h"
#include <vector>	

using LiteMath::uint3;
using LiteMath::int2;
using LiteMath::float3;
using LiteMath::float4;

static const uint32_t NO_PRIMITIVE = UINT32_MAX;

struct Node
{
	Node* m_parent = nullptr;
	Node* m_left_child = nullptr;
	Node* m_right_child = nullptr;
	uint32_t m_primitive_id = NO_PRIMITIVE;
	BoundingBox m_bnd_box;
	uint16_t reserved[4] = { 0u, 0u, 0u, 0u };
};

class LinearBVH
{
public:
	LinearBVH() = default;

	LinearBVH(const LinearBVH&) = delete;

	LinearBVH(LinearBVH&&) = default;

	LinearBVH& operator=(LinearBVH&& other) = default;

	void insertTriangles(const std::vector<float4>& vertices_coords, const std::vector<unsigned int>& vertices_indices);

	_NODISCARD std::vector<int> findCollisionsWithRay(const float3& ray_start, const float3& ray_dir) const;

	_NODISCARD bool findClosestTriangle(const std::vector<float4>& vertices, const std::vector<unsigned int>& indices,
		const float3& point, int& res_triangle, float3& res_point, float& res_distance) const;

	_NODISCARD std::vector<int> findCollisionsWithBndBox(const BoundingBox& bnd_box) const;

private:
	struct PointWithCode
	{
		uint32_t index = 0u;
		uint32_t code = 0u;
	};

	_NODISCARD uint32_t expandBits(uint32_t value) const
	{
		value = (value * 0x00010001u) & 0xFF0000FFu;
		value = (value * 0x00000101u) & 0x0F00F00Fu;
		value = (value * 0x00000011u) & 0xC30C30C3u;
		value = (value * 0x00000005u) & 0x49249249u;
		return value;
	}

	_NODISCARD std::vector<PointWithCode> sortMorton(const std::vector<float3>& points) const;

	_NODISCARD int delta(const std::vector<PointWithCode>& sorted_points, int a, int b) const;

	_NODISCARD int safe_delta(const std::vector<PointWithCode>& sorted_points, int a, int b) const;

	_NODISCARD int2 determineRange(const std::vector<PointWithCode>& sorted_points, int i) const;

	_NODISCARD int findSplit(const std::vector<PointWithCode>& sorted_points, int2 range) const;

	void generateHeirarchy(const std::vector<PointWithCode>& sorted_points);

	void createLayers(Node* current_node, int layer, std::vector<std::vector<Node*>>& layers) const;

	void calcBoundingBoxes(const std::vector<float4>& vertices_coords, const std::vector<unsigned int>& vertices_indices);

private:
	std::vector<Node> m_leaf_nodes;
	std::vector<Node> m_internal_nodes;
};