#include "draw_mesh_bvh.h"
#include"mesh.h"

using LiteMath::float3;
using LiteMath::float4;

namespace
{
	_NODISCARD float3 float4_to_float3(const float4& vec)
	{
		return { vec.x, vec.y, vec.z };
	}

	_NODISCARD std::tuple<float3, float3 > unpack_plane(const float4& parameters)
	{
		float3 normal = float4_to_float3(parameters);
		normal = normalize(normal);

		if (fabs(parameters.w) < FLT_EPSILON)
		{
			return std::make_tuple(normal, float3(0.0f, 0.0f, 0.0f));
		}

		if (fabs(parameters.z) > FLT_EPSILON)
		{
			return std::make_tuple(normal, normalize(float3(0.0f, 0.0f, -parameters.w / parameters.z)));
		}
		else if (fabs(parameters.y) > FLT_EPSILON)
		{
			return std::make_tuple(normal, normalize(float3(0.0f, -parameters.w / parameters.y, 0.0f)));
		}

		// else if (fabs(x) > FLT_EPSILON)
		return std::make_tuple(normal, normalize(float3(-parameters.w / parameters.x, 0.0f, 0.0f)));
	}

	_NODISCARD bool ray_intersects_triangle(const float3& a, const float3& b, const float3& c,
		const float3& ray_point, const float3& ray_dir, float3& barycentric, float& t)
	{
		const float3 edge1 = b - a;
		const float3 edge2 = c - a;
		const float3 h = cross(ray_dir, edge2);

		const float dot_prod = dot(edge1, h);
		if (fabs(dot_prod) < FLT_EPSILON)
		{
			return false;
		}

		const float f = 1.0f / dot_prod;
		const float3 s = ray_point - a;
		const float u = f * dot(s, h);
		if (u < 0.0f || u > 1.0f)
		{
			return false;
		}

		const float3 q = cross(s, edge1);
		const float v = f * dot(ray_dir, q);
		if (v < 0.0f || u + v > 1.0f)
		{
			return false;
		}

		const float w = 1.0f - u - v;
		barycentric = { w, u, v };
		t = f * dot(edge2, q);
		return t > FLT_EPSILON;
	}

	_NODISCARD float3 calculate_normal(const cmesh4::SimpleMesh& mesh, int triangle, const float3& barycentric)
	{
		const size_t index_offset = 3u * (size_t)triangle;
		const float3 vertex_a = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset]]);
		const float3 vertex_b = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 1]]);
		const float3 vertex_c = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 2]]);

		const float3 edge_a = vertex_a - vertex_b;
		const float3 edge_b = vertex_a - vertex_c;
		return normalize(cross(edge_a, edge_b));
	}

	_NODISCARD bool ray_intersects_mesh(const cmesh4::SimpleMesh& mesh, const LinearBVH& bvh, const float3& ray_point,
		const float3& ray_dir, float& t, float3& point, float3& normal)
	{
		point = { 0.0f, 0.0f, 0.0f };
		normal = { 0.0f, 0.0f, 0.0f };

		const size_t triangles_count = mesh.TrianglesNum();
		size_t index_offset = 0u;

		bool found_intersection = false;
		float3 barycentric;
		t = FLT_MAX;
		int triangle_index = 0;

		const std::vector<int> intersections = bvh.findCollisionsWithRay(ray_point, ray_dir);
		for (int triangle : intersections)
		{
			const size_t index_offset = 3u * (size_t)triangle;
			const float3 vertex_a = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset]]);
			const float3 vertex_b = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 1]]);
			const float3 vertex_c = float4_to_float3(mesh.vPos4f[mesh.indices[index_offset + 2]]);

			float3 tmp_barycentric;
			float tmp_t = FLT_MAX;

			if (ray_intersects_triangle(vertex_a, vertex_b, vertex_c, ray_point, ray_dir, tmp_barycentric, tmp_t) && t > tmp_t)
			{
				found_intersection = true;
				barycentric = tmp_barycentric;
				t = tmp_t;
				triangle_index = triangle;
			}
		}

		if (found_intersection)
		{
			point = ray_point + t * ray_dir;
			normal = calculate_normal(mesh, triangle_index, barycentric);
		}

		return found_intersection;
	}

	_NODISCARD bool ray_intersects_plane(const float3& plane_normal, const float3& plane_point,
		const float3& ray_point, const float3& ray_dir, float& t, float3& point)
	{
		const float denom = dot(plane_normal, ray_dir);
		if (std::fabs(denom) < FLT_EPSILON)
		{
			return false;
		}

		const float3 plane_point_to_ray_point = plane_point - ray_point;
		t = dot(plane_point_to_ray_point, plane_normal) / denom;
		point = ray_point + t * ray_dir;
		return t >= 0.0f;
	}

	void calc_mesh_color(const AppData& app_data, const cmesh4::SimpleMesh& mesh, const LinearBVH& bvh, const float3& point, const float3& normal, uint32_t& result)
	{
		float dummy_t;
		float3 dummy_point;
		float3 dummy_normal;
		uint8_t light_intencity_int = 0u;

		if (!ray_intersects_mesh(mesh, bvh, point, app_data.light_dir, dummy_t, dummy_point, dummy_normal))
		{
			float light_intencity = app_data.light_brightnes * std::max(0.0f, dot(app_data.light_dir, normal)) + app_data.ambient_brightnes;
			light_intencity = std::clamp(light_intencity, 0.0f, 255.0f);
			light_intencity_int = (uint8_t)std::round(light_intencity);
		}
		else
		{
			float light_intencity = app_data.ambient_brightnes;
			light_intencity = std::clamp(light_intencity, 0.0f, 255.0f);
			light_intencity_int = (uint8_t)std::round(light_intencity);
		}

		uint8_t* pixel = (uint8_t*)&result;
		pixel[0] = light_intencity_int;
		pixel[1] = light_intencity_int;
		pixel[2] = light_intencity_int;
		pixel[3] = 255u;
	}

	void calc_plane_color(const AppData& app_data, const cmesh4::SimpleMesh& mesh, const LinearBVH& bvh,
		const float3& prev_ray_dir, const float3& point, const float3& normal, uint32_t& result)
	{
		const float3 new_ray_dir = prev_ray_dir - 2 * dot(prev_ray_dir, normal) * normal;

		float mesh_intersection_t;
		float3 mesh_intersection_point;
		float3 mesh_intersection_normal;
		const bool intersects_mesh =
			ray_intersects_mesh(mesh, bvh, point, new_ray_dir, mesh_intersection_t, mesh_intersection_point, mesh_intersection_normal);

		if (intersects_mesh)
		{
			mesh_intersection_point += 0.01f * mesh_intersection_normal;
			calc_mesh_color(app_data, mesh, bvh, mesh_intersection_point, mesh_intersection_normal, result);
		}
	}
}

namespace DrawMeshBVH
{
	Data::Data(const char* mesh_path, float4 plane_parameters) :
		m_mesh(cmesh4::LoadMeshFromObj(mesh_path)), m_plane_parameters(plane_parameters)
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

	void draw_mesh_bvh(AppData& app_data, const Camera& camera)
	{
		const Data* const data = (const Data*)app_data.m_data.get();
		const cmesh4::SimpleMesh& mesh = data->m_mesh;
		const LinearBVH& bvh = data->m_bvh;
		const auto [plane_normal, plane_point] = unpack_plane(data->m_plane_parameters);

		const float3 camera_pos = camera.m_position;
		const float3x3 camera_mat = camera.create_camera_rotation_mat();

		AlignedVector::AlignedVector<uint32_t>& frame_buffer = app_data.m_frame_buffer;

#pragma omp parallel for
		for (int i = 0; i < app_data.m_height; ++i)
		{
			for (int j = 0; j < app_data.m_width; ++j)
			{
				float3 ray_dir = camera.create_ray_dir((float)j, (float)i, (float)app_data.m_width, (float)app_data.m_height);
				ray_dir = normalize(camera_mat * ray_dir);

				float plane_intersection_t;
				float3 plane_intersection_point;
				const bool intersects_plane =
					ray_intersects_plane(plane_normal, plane_point, camera_pos, ray_dir, plane_intersection_t, plane_intersection_point);

				float mesh_intersection_t;
				float3 mesh_intersection_point;
				float3 mesh_intersection_normal;
				const bool intersects_mesh =
					ray_intersects_mesh(mesh, bvh, camera_pos, ray_dir, mesh_intersection_t, mesh_intersection_point, mesh_intersection_normal);

				if (!intersects_plane && !intersects_mesh)
				{
					continue;
				}

				const bool need_draw_plane = (!intersects_mesh && intersects_plane) || (plane_intersection_t >= 0.0f && plane_intersection_t < mesh_intersection_t);
				if (need_draw_plane)
				{
					plane_intersection_point += 0.01f * plane_normal;
					const uint32_t dst_offset = i * app_data.m_width + j;
					calc_plane_color(app_data, mesh, bvh, ray_dir, plane_intersection_point, plane_normal, frame_buffer[dst_offset]);
				}
				else
				{
					mesh_intersection_point += 0.01f * mesh_intersection_normal;
					const uint32_t dst_offset = i * app_data.m_width + j;
					calc_mesh_color(app_data, mesh, bvh, mesh_intersection_point, mesh_intersection_normal, frame_buffer[dst_offset]);
				}
			}
		}
	}
}