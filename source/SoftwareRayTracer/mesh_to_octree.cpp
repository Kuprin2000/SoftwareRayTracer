#include "mesh_to_octree.h"
#include <stack>
#include "octree.h"

namespace
{
	_NODISCARD float3 float4_to_float3(const float4& vec)
	{
		return { vec.x, vec.y, vec.z };
	}

	inline float sign(float value)
	{
		return (0.0f < value) - (value < 0.0f);
	}

	_NODISCARD float3 calculate_normal(const float3& a, const float3& b, const float3& c)
	{
		const float3 edge_a = a - b;
		const float3 edge_b = a - c;
		return normalize(cross(edge_a, edge_b));
	}

	_NODISCARD float calculate_sdf(const cmesh4::SimpleMesh& mesh, const LinearBVH& bvh, const float3& point)
	{
		int res_triangle = 0;
		float3 res_point;
		float res_distance;
		const bool ok = bvh.findClosestTriangle(mesh.vPos4f, mesh.indices, point, res_triangle, res_point, res_distance);

		const size_t index_offset = 3u * res_triangle;
		const float3 vertex_a = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset]]);
		const float3 vertex_b = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 1]]);
		const float3 vertex_c = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 2]]);

		const float3 res_normal = calculate_normal(vertex_a, vertex_b, vertex_c);

		return sign(dot(res_normal, point - res_point)) * length(point - res_point);
	}

	struct StackElement
	{
		BoundingBox parent_bounding_box;
		int parent_offset = 0;
		int depth = 0;
	};

	_NODISCARD std::array<float, 8> calculate_values(const BoundingBox& bounding_box, const cmesh4::SimpleMesh& mesh, const LinearBVH& bvh)
	{
		const float3 min_corner = bounding_box.getMinCorner();
		const float3 max_corner = bounding_box.getMaxCorner();

		std::array<float, 8> result;

		const float3 point_s000 = min_corner;
		result[0] = calculate_sdf(mesh, bvh, point_s000);

		const float3 point_s001 = { min_corner.x, min_corner.y, max_corner.z };
		result[1] = calculate_sdf(mesh, bvh, point_s001);

		const float3 point_s010 = { min_corner.x, max_corner.y, min_corner.z };
		result[2] = calculate_sdf(mesh, bvh, point_s010);

		const float3 point_s011 = { min_corner.x, max_corner.y, max_corner.z };
		result[3] = calculate_sdf(mesh, bvh, point_s011);

		const float3 point_s100 = { max_corner.x, min_corner.y, min_corner.z };
		result[4] = calculate_sdf(mesh, bvh, point_s100);

		const float3 point_s101 = { max_corner.x, min_corner.y, max_corner.z };
		result[5] = calculate_sdf(mesh, bvh, point_s101);

		const float3 point_s110 = { max_corner.x, max_corner.y, min_corner.z };
		result[6] = calculate_sdf(mesh, bvh, point_s110);

		const float3 point_s111 = { max_corner.x, max_corner.y, max_corner.z };
		result[7] = calculate_sdf(mesh, bvh, point_s111);

		return result;
	}
}

namespace MeshToOctree
{
	Data::Data(int d, const char* mesh_path, const char* octree_path) : m_d(d),
		m_mesh(cmesh4::LoadMeshFromObj(mesh_path)), m_octree_path(octree_path)
	{
		resizeMesh();

		if (m_mesh.VerticesNum())
		{
			m_bvh.insertTriangles(m_mesh.vPos4f, m_mesh.indices);
		}
	}

	void Data::getMeshExtent(float4& min_corner, float4& max_corner) const
	{
		for (const auto& vertex : m_mesh.vPos4f)
		{
			min_corner = {
				std::min(min_corner.x, vertex.x),
				std::min(min_corner.y, vertex.y),
				std::min(min_corner.z, vertex.z),
				0.0f
			};

			max_corner = {
				std::max(max_corner.x, vertex.x),
				std::max(max_corner.y, vertex.y),
				std::max(max_corner.z, vertex.z),
				0.0f
			};
		}
	}

	void Data::resizeMesh()
	{
		float4 min_corner;
		float4 max_corner;
		getMeshExtent(min_corner, max_corner);

		float4 scale_coeff = 1.9f / (max_corner - min_corner);
		const float min_coeff = std::min({ scale_coeff.x, scale_coeff.y, scale_coeff.z });
		scale_coeff = {
			min_coeff,
			min_coeff,
			min_coeff,
			1.0f
		};

		const float4 movement(-0.95f, -0.95f, -0.95f, 0.0f);

		for (auto& vertex : m_mesh.vPos4f)
		{
			vertex -= min_corner;
			vertex *= scale_coeff;
			vertex += movement;
		}
	}

	void mesh_to_octree(AppData& app_data)
	{
		const Data* const data = (const Data*)app_data.m_data.get();

		const int d = data->m_d;
		const cmesh4::SimpleMesh& mesh = data->m_mesh;
		const LinearBVH& bvh = data->m_bvh;
		const std::string& dst_path = data->m_octree_path;

		Octree::SdfOctree result;

		BoundingBox root_bounding_box;
		root_bounding_box.setBox(float3(-1.0f, -1.0f, -1.0f), float3(1.0f, 1.0f, 1.0f));
		const std::array<float, 8> root_values = calculate_values(root_bounding_box, mesh, bvh);
		result.m_nodes.push_back({ root_values, 0u });

		std::stack<StackElement> stack;
		for (int i = 0; i < 8; ++i)
		{
			stack.push({ root_bounding_box, 0u, 1 });
		}

		while (!stack.empty())
		{
			const StackElement element = stack.top();
			stack.pop();

			result.m_nodes[element.parent_offset].offset = result.m_nodes.size();
			for (int i = 0; i < 8; ++i)
			{
				const BoundingBox tmp_bounding_box = element.parent_bounding_box.getOctalPart(i);

				const bool is_empty = bvh.findCollisionsWithBndBox(tmp_bounding_box).empty();

				std::array<float, 8> values = { 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f };
				if (!is_empty)
				{
					values = calculate_values(tmp_bounding_box, mesh, bvh);
				}

				result.m_nodes.push_back({ values, 0u });

				if (element.depth == d || is_empty)
				{
					continue;
				}

				stack.push({ tmp_bounding_box, (int)result.m_nodes.size() - 1, element.depth + 1 });
			}
		}

		Octree::save_sdf_octree(result, dst_path);
	}
}