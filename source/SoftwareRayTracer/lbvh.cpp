#include "lbvh.h"
#include <execution>
#include <stack>
#include <list>
#include <cmath>

namespace
{
	inline int sign(int value)
	{
		return (0 < value) - (value < 0);
	}

	_NODISCARD float3 float4_to_float3(const float4& vec)
	{
		return { vec.x, vec.y, vec.z };
	}

	void distance_to_triangle(const float3& point, const float3& a, const float3& b, const float3& c, float& res_distance, float3& res_point)
	{
		const float3 ab = b - a;
		const float3 ac = c - a;
		const float3 ap = point - a;

		const float d1 = dot(ab, ap);
		const float d2 = dot(ac, ap);

		if (d1 <= 0.0f && d2 <= 0.0f)
		{
			res_point = a;
			res_distance = length(res_point - point);
			return;
		}

		const float3 bp = point - b;
		const float d3 = dot(ab, bp);
		const float d4 = dot(ac, bp);

		if (d3 >= 0.0f && d4 <= d3)
		{
			res_point = b;
			res_distance = length(res_point - point);
			return;
		}

		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		{
			const float v = d1 / (d1 - d3);
			res_point = a + v * ab;
			res_distance = length(res_point - point);
			return;
		}

		const float3 cp = point - c;
		const float d5 = dot(ab, cp);
		const float d6 = dot(ac, cp);

		if (d6 >= 0.0f && d5 <= d6)
		{
			res_point = c;
			res_distance = length(res_point - point);
			return;
		}

		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		{
			const float w = d2 / (d2 - d6);
			res_point = a + w * ac;
			res_distance = length(res_point - point);
			return;
		}

		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		{
			const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			res_point = b + w * (c - b);
			res_distance = length(res_point - point);
			return;
		}

		const float denom = 1.0f / (va + vb + vc);
		const float v = vb * denom;
		const float w = vc * denom;

		res_point = a + v * ab + w * ac;
		res_distance = length(res_point - point);
	}

	struct QueueElement
	{
		const Node* node;
		float distance;

		_NODISCARD bool operator<(const QueueElement& other) const
		{
			return distance > other.distance;
		}
	};
}

void LinearBVH::insertTriangles(const std::vector<float4>& vertices_coords, const std::vector<unsigned int>& vertices_indices)
{
	std::vector<float3> centers(vertices_indices.size() / 3, { 0.0f, 0.0f, 0.0f });
	const float coeff_1 = 1.0f / 3.0f;
	int indices_offset = 0;
	for (int i = 0; i < centers.size(); ++i)
	{
		const float4& vertex_a = vertices_coords[vertices_indices[indices_offset]];
		const float4& vertex_b = vertices_coords[vertices_indices[indices_offset + 1]];
		const float4& vertex_c = vertices_coords[vertices_indices[indices_offset + 2]];

		const float4 tmp_center = vertex_a + vertex_b + vertex_c;
		centers[i] = float3(tmp_center.x, tmp_center.y, tmp_center.z) * coeff_1;

		indices_offset += 3;
	}

	float3 min_values = { FLT_MAX, FLT_MAX, FLT_MAX };
	for (const auto& elem : centers)
	{
		min_values = { std::min(min_values.x, elem.x), std::min(min_values.y, elem.y), std::min(min_values.z, elem.z) };
	}

	float max_delta = 0.0f;
	for (auto& elem : centers)
	{
		elem -= min_values;
		max_delta = std::max({ max_delta , fabs(elem.x), fabs(elem.y), fabs(elem.z) });
	}

	const float coeff = 1.0f / max_delta;

	for (auto& elem : centers)
	{
		elem *= coeff;
	}

	const std::vector<PointWithCode> sorted_vertices = sortMorton(centers);

	generateHeirarchy(sorted_vertices);

	calcBoundingBoxes(vertices_coords, vertices_indices);
}

std::vector<int> LinearBVH::findCollisionsWithRay(const float3& ray_start, const float3& ray_dir) const
{
	if (m_leaf_nodes.empty())
	{
		return {};
	}

	std::vector<int> result;

	const Node* current_node = &m_internal_nodes[0];

	std::stack<const Node*> candidates_stack;
	candidates_stack.push(nullptr);

	float3 dummy_point;
	while (current_node)
	{
		const bool left_intersects =
			current_node->m_left_child && current_node->m_left_child->m_bnd_box.intersectsRay(ray_start, ray_dir, dummy_point, dummy_point);
		const bool right_intersects =
			current_node->m_right_child && current_node->m_right_child->m_bnd_box.intersectsRay(ray_start, ray_dir, dummy_point, dummy_point);

		if (left_intersects && current_node->m_left_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_left_child->m_primitive_id);
		}
		if (right_intersects && current_node->m_right_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_right_child->m_primitive_id);
		}

		const bool left_is_candidate = left_intersects && (current_node->m_left_child->m_primitive_id == NO_PRIMITIVE);
		const bool right_is_candidate = right_intersects && (current_node->m_right_child->m_primitive_id == NO_PRIMITIVE);

		if (left_is_candidate || right_is_candidate)
		{
			const Node* right_child = current_node->m_right_child;
			current_node = left_is_candidate ? current_node->m_left_child : current_node->m_right_child;

			if (left_is_candidate && right_is_candidate)
			{
				candidates_stack.push(right_child);
			}
		}
		else
		{
			current_node = candidates_stack.top();
			candidates_stack.pop();
		}
	}

	return result;
}

bool LinearBVH::findClosestTriangle(const std::vector<float4>& vertices, const std::vector<unsigned int>& indices,
	const float3& point, int& res_triangle, float3& res_point, float& res_distance) const
{
	if (m_leaf_nodes.empty())
	{
		return false;
	}

	res_triangle = -1;
	res_point = { FLT_MAX, FLT_MAX, FLT_MAX };
	res_distance = FLT_MAX;

	std::priority_queue<QueueElement> queue;
	const Node* root_node = &m_internal_nodes[0];
	queue.push({ root_node, root_node->m_bnd_box.distanceFromPoint(point) });

	while (!queue.empty())
	{
		const QueueElement current_node = queue.top();
		queue.pop();

		if (current_node.distance >= res_distance)
		{
			continue;
		}

		const Node* left_child = current_node.node->m_left_child;
		const Node* right_child = current_node.node->m_right_child;

		const bool has_left_child = left_child;
		const bool has_right_child = right_child;

		const bool left_is_triangle = has_left_child && left_child->m_primitive_id != NO_PRIMITIVE;
		const bool right_is_triangle = has_right_child && right_child->m_primitive_id != NO_PRIMITIVE;

		if (left_is_triangle)
		{
			const size_t index_offset = 3u * left_child->m_primitive_id;
			const float3 vertex_a = float4_to_float3(vertices[indices[index_offset]]);
			const float3 vertex_b = float4_to_float3(vertices[indices[index_offset + 1]]);
			const float3 vertex_c = float4_to_float3(vertices[indices[index_offset + 2]]);

			float tmp_distance;
			float3 tmp_point;
			distance_to_triangle(point, vertex_a, vertex_b, vertex_c, tmp_distance, tmp_point);
			if (tmp_distance < res_distance)
			{
				res_triangle = left_child->m_primitive_id;
				res_point = tmp_point;
				res_distance = tmp_distance;
			}
		}

		if (right_is_triangle)
		{
			const size_t index_offset = 3u * right_child->m_primitive_id;
			const float3 vertex_a = float4_to_float3(vertices[indices[index_offset]]);
			const float3 vertex_b = float4_to_float3(vertices[indices[index_offset + 1]]);
			const float3 vertex_c = float4_to_float3(vertices[indices[index_offset + 2]]);

			float tmp_distance;
			float3 tmp_point;
			distance_to_triangle(point, vertex_a, vertex_b, vertex_c, tmp_distance, tmp_point);
			if (tmp_distance < res_distance)
			{
				res_triangle = right_child->m_primitive_id;
				res_point = tmp_point;
				res_distance = tmp_distance;
			}
		}

		if (has_left_child && !left_is_triangle)
		{
			const float distance = left_child->m_bnd_box.distanceFromPoint(point);
			queue.push({ left_child, distance });
		}

		if (has_right_child && !right_is_triangle)
		{
			const float distance = right_child->m_bnd_box.distanceFromPoint(point);
			queue.push({ right_child, distance });
		}
	}

	return true;
}

std::vector<int> LinearBVH::findCollisionsWithBndBox(const BoundingBox& bnd_box) const
{
	if (m_leaf_nodes.empty())
	{
		return {};
	}

	std::vector<int> result;

	const Node* current_node = &m_internal_nodes[0];

	std::stack<const Node*> candidates_stack;
	candidates_stack.push(nullptr);

	while (current_node)
	{
		const bool left_overlaps = current_node->m_left_child && BoundingBox::thereIsOVerlap(current_node->m_left_child->m_bnd_box, bnd_box);
		const bool right_overlaps = current_node->m_right_child && BoundingBox::thereIsOVerlap(current_node->m_right_child->m_bnd_box, bnd_box);

		if (left_overlaps && current_node->m_left_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_left_child->m_primitive_id);
		}
		if (right_overlaps && current_node->m_right_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_right_child->m_primitive_id);
		}

		const bool left_is_candidate = left_overlaps && (current_node->m_left_child->m_primitive_id == NO_PRIMITIVE);
		const bool right_is_candidate = right_overlaps && (current_node->m_right_child->m_primitive_id == NO_PRIMITIVE);

		if (left_is_candidate || right_is_candidate)
		{
			const Node* right_child = current_node->m_right_child;
			current_node = left_is_candidate ? current_node->m_left_child : current_node->m_right_child;

			if (left_is_candidate && right_is_candidate)
			{
				candidates_stack.push(right_child);
			}
		}
		else
		{
			current_node = candidates_stack.top();
			candidates_stack.pop();
		}
	}

	return result;
}

std::vector<LinearBVH::PointWithCode> LinearBVH::sortMorton(const std::vector<float3>& points) const
{
	std::vector<PointWithCode> points_to_sort(points.size());

	for (uint32_t i = 0; i < points.size(); ++i)
	{
		float3 elem = points[i];
		elem.x = std::min(std::max(elem.x * 1024.0f, 0.0f), 1023.0f);
		elem.y = std::min(std::max(elem.y * 1024.0f, 0.0f), 1023.0f);
		elem.z = std::min(std::max(elem.z * 1024.0f, 0.0f), 1023.0f);
		points_to_sort[i] =
		{ i, expandBits((uint32_t)elem.x) * 4 + expandBits((uint32_t)elem.y) * 2 + expandBits((uint32_t)elem.z) };
	}

	std::sort(std::execution::par, points_to_sort.begin(), points_to_sort.end(), [](PointWithCode a, PointWithCode b) { return a.code < b.code; });

	return points_to_sort;
}

int LinearBVH::delta(const std::vector<PointWithCode>& sorted_points, int a, int b) const
{
	return __lzcnt(sorted_points[a].code ^ sorted_points[b].code);
}

int LinearBVH::safe_delta(const std::vector<PointWithCode>& sorted_points, int a, int b) const
{
	return (b < 0 || b >= sorted_points.size()) ? -1 : __lzcnt(sorted_points[a].code ^ sorted_points[b].code);
}

int2 LinearBVH::determineRange(const std::vector<PointWithCode>& sorted_points, int i) const
{
	const int direction = sign(delta(sorted_points, i, i + 1) - safe_delta(sorted_points, i, i - 1));
	const int delta_min = safe_delta(sorted_points, i, i - direction);

	int l_max = 2;
	while (safe_delta(sorted_points, i, i + l_max * direction) > delta_min)
	{
		l_max *= 2;
	}

	int l = 0;
	for (int t = l_max / 2; t; t /= 2)
	{
		if (safe_delta(sorted_points, i, i + (l + t) * direction) > delta_min)
		{
			l += t;
		}
	}

	return { i, i + l * direction };
}

int LinearBVH::findSplit(const std::vector<PointWithCode>& sorted_points, int2 range) const
{
	const int delta_node = safe_delta(sorted_points, range.x, range.y);
	int s = 0;
	int t = (range.x > range.y) ? range.x - range.y : range.y - range.x;
	int direction = (range.x > range.y) ? -1 : 1;

	do
	{
		t = (t + 1) >> 1;
		if (safe_delta(sorted_points, range.x, range.x + (s + t) * direction) > delta_node)
		{
			s += t;
		}
	} while (t > 1);

	return range.x + s * direction + std::min(direction, 0);
}

void LinearBVH::generateHeirarchy(const std::vector<PointWithCode>& sorted_points)
{
	m_leaf_nodes.clear();
	m_internal_nodes.clear();

	const int points_count = (int)sorted_points.size();
	m_leaf_nodes.resize(points_count);
	m_internal_nodes.resize(points_count - 1);

	for (int i = 0; i < points_count; ++i)
	{
		m_leaf_nodes[i].m_primitive_id = sorted_points[i].index;
	}

#pragma omp parallel for
	for (int i = 0; i < points_count - 1; ++i)
	{
		const int2 range = determineRange(sorted_points, i);
		const int split = findSplit(sorted_points, range);

		Node* left_child = (std::min(range.x, range.y) == split) ? &m_leaf_nodes[split] : &m_internal_nodes[split];
		Node* right_child = (std::max(range.x, range.y) == split + 1) ? &m_leaf_nodes[split + 1] : &m_internal_nodes[split + 1];

		m_internal_nodes[i].m_left_child = left_child;
		m_internal_nodes[i].m_right_child = right_child;
		left_child->m_parent = &m_internal_nodes[i];
		right_child->m_parent = &m_internal_nodes[i];
	}
}

void LinearBVH::createLayers(Node* current_node, int layer, std::vector<std::vector<Node*>>& layers) const
{
	if (layers.size() < layer + 1)
	{
		layers.resize(layer + 1);
	}

	layers[layer].push_back(current_node);

	if (current_node->m_left_child && current_node->m_left_child->m_primitive_id == NO_PRIMITIVE)
	{
		createLayers(current_node->m_left_child, layer + 1, layers);
	}
	if (current_node->m_right_child && current_node->m_right_child->m_primitive_id == NO_PRIMITIVE)
	{
		createLayers(current_node->m_right_child, layer + 1, layers);
	}
}

void LinearBVH::calcBoundingBoxes(const std::vector<float4>& vertices_coords, const std::vector<unsigned int>& vertices_indices)
{
#pragma omp parallel for
	for (int i = 0; i < m_leaf_nodes.size(); ++i)
	{
		const size_t indices_offset = 3 * m_leaf_nodes[i].m_primitive_id;

		const float4& vertex_a = vertices_coords[vertices_indices[indices_offset]];
		const float4& vertex_b = vertices_coords[vertices_indices[indices_offset + 1]];
		const float4& vertex_c = vertices_coords[vertices_indices[indices_offset + 2]];

		const float3 triangle_vertices_coords[3] =
		{
			float3(vertex_a.x, vertex_a.y, vertex_a.z),
			float3(vertex_b.x, vertex_b.y, vertex_b.z),
			float3(vertex_c.x, vertex_c.y, vertex_c.z),
		};

		BoundingBox box;
		box.setTriangle(triangle_vertices_coords, 1e-5);
		m_leaf_nodes[i].m_bnd_box = box;
	}

	std::vector<std::vector<Node*>> layers((uint32_t)log2f(float(m_internal_nodes.size())) + 2u);
	createLayers(&m_internal_nodes[0], 0, layers);

#pragma omp parallel
	{
		for (int i = (int)layers.size() - 1; i >= 0; --i)
		{
#pragma omp for
			for (int j = 0; j < layers[i].size(); ++j)
			{
				BoundingBox box;
				Node* node = layers[i][j];
				if (node->m_left_child)
				{
					box.addBndBox(node->m_left_child->m_bnd_box);
				}
				if (node->m_right_child)
				{
					box.addBndBox(node->m_right_child->m_bnd_box);
				}
				node->m_bnd_box = box;
			}
		}
	}
}
