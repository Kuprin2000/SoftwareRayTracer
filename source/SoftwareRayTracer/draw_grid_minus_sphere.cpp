#include "draw_grid_minus_sphere.h"
#include "bounding_box.h"

namespace
{
	bool ray_intersects_grid(const DrawGridMinusSphere::SdfGridMinusSphere& grid, const float3& ray_start, const float3& ray_dir, float3& point, float3& normal)
	{
		point = ray_start;

		if (!grid.contains(point))
		{
			BoundingBox grid_bounding_box;
			grid_bounding_box.setBox(float3(-1.0f, -1.0f, -1.0f), float3(1.0f, 1.0f, 1.0f));

			float3 intersection_point;
			float3 dummy_point;
			if (!grid_bounding_box.intersectsRay(point, ray_dir, intersection_point, dummy_point))
			{
				return false;
			}

			point = intersection_point + 0.01f * ray_dir;

			if (!grid.contains(point))
			{
				return false;
			}
		}

		float step = grid.sample_value(point);
		while (step > 0.0001f)
		{
			point += step * ray_dir;
			if (!grid.contains(point))
			{
				return false;
			}

			step = grid.sample_value(point);
		}

		normal = grid.sample_grad(point);

		return true;
	}

	void calc_grid_color(const AppData& app_data, const DrawGridMinusSphere::SdfGridMinusSphere& grid, const float3& point, const float3& normal, uint32_t& result)
	{
		float3 dummy_point;
		float3 dummy_normal;
		uint8_t light_intencity_int = 0u;

		if (!ray_intersects_grid(grid, point, app_data.light_dir, dummy_point, dummy_normal))
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
}

namespace DrawGridMinusSphere
{

	SdfGridMinusSphere::SdfGridMinusSphere(const float3& center, float raduis) :
		m_sphere_center(center), m_sphere_radius(raduis)
	{
	}

	_NODISCARD float SdfGridMinusSphere::sample_value(const float3& point) const
	{
		if (!contains(point))
		{
			return -FLT_MAX;
		}

		const float3 position_in_grid = (point + 1.0f - 0.5f * m_step) / m_step;
		const Grid::uint3 min_corner((uint32_t)position_in_grid.x, (uint32_t)position_in_grid.y, (uint32_t)position_in_grid.z);
		Grid::uint3 max_corner = {
			std::min(min_corner.x + 1u, m_size.x - 1u),
			std::min(min_corner.y + 1u, m_size.y - 1u),
			std::min(min_corner.z + 1u, m_size.z - 1u)
		};

		const float3 frac(
			position_in_grid.x - min_corner.x,
			position_in_grid.y - min_corner.y,
			position_in_grid.z - min_corner.z
		);

		float sdf_result = 0.0f;
		sdf_result += get(min_corner.x, min_corner.y, min_corner.z) * (1.0f - frac.x) * (1.0f - frac.y) * (1.0f - frac.z);
		sdf_result += get(min_corner.x, min_corner.y, max_corner.z) * (1.0f - frac.x) * (1.0f - frac.y) * frac.z;
		sdf_result += get(min_corner.x, max_corner.y, min_corner.z) * (1.0f - frac.x) * frac.y * (1.0f - frac.z);
		sdf_result += get(min_corner.x, max_corner.y, max_corner.z) * (1.0f - frac.x) * frac.y * frac.z;
		sdf_result += get(max_corner.x, min_corner.y, min_corner.z) * frac.x * (1.0f - frac.y) * (1.0f - frac.z);
		sdf_result += get(max_corner.x, min_corner.y, max_corner.z) * frac.x * (1.0f - frac.y) * frac.z;
		sdf_result += get(max_corner.x, max_corner.y, min_corner.z) * frac.x * frac.y * (1.0f - frac.z);
		sdf_result += get(max_corner.x, max_corner.y, max_corner.z) * frac.x * frac.y * frac.z;

		const float sphere_result = length(point - m_sphere_center) - m_sphere_radius;

		return std::max(-sphere_result, sdf_result);
	}

	Data::Data(const char* grid_path, const float3& center, float raduis): m_grid(center, raduis)
	{
		Grid::load_sdf_grid(m_grid, grid_path);
	}

	void draw_grid_minus_sphere(AppData& app_data, const Camera& camera)
	{
		const Data* const data = (const Data*)app_data.m_data.get();
		const SdfGridMinusSphere& grid = data->m_grid;

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

				float3 intersection_point;
				float3 intersection_normal;
				if (!ray_intersects_grid(grid, camera_pos, ray_dir, intersection_point, intersection_normal))
				{
					continue;
				}

				intersection_point += 0.01f * intersection_normal;
				const uint32_t dst_offset = i * app_data.m_width + j;
				calc_grid_color(app_data, grid, intersection_point, intersection_normal, frame_buffer[dst_offset]);
			}
		}
	}
}