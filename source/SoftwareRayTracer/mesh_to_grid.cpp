#include "mesh_to_grid.h"
#include "grid.h"

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
}

namespace MeshToGrid
{
	Data::Data(int size, const char* mesh_path, const char* grid_path) : m_size(size),
		m_mesh(cmesh4::LoadMeshFromObj(mesh_path)), m_grid_path(grid_path)
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

	void mesh_to_grid(AppData& app_data)
	{
		const Data* const data = (const Data*)app_data.m_data.get();

		const int size = data->m_size;
		const cmesh4::SimpleMesh& mesh = data->m_mesh;
		const LinearBVH& bvh = data->m_bvh;
		const std::string& dst_path = data->m_grid_path;

		Grid::SdfGrid result(data->m_size);

#pragma omp parallel for
		for (int x = 0; x < size; ++x)
		{
			for (int y = 0; y < size; ++y)
			{
				for (int z = 0; z < size; ++z)
				{
					float3 point((float)x, (float)y, (float)z);
					point *= result.m_step;
					point += 0.5f * result.m_step;
					point += -1.0f;

					const float value = calculate_sdf(mesh, bvh, point);
					result.set(x, y, z, value);
				}
			}
		}

		Grid::save_sdf_grid(result, dst_path);
	}
}