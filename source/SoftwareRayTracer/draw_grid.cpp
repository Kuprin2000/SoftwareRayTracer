#include "draw_grid.h"
#include "bounding_box.h"

namespace
{
	_NODISCARD bool ray_intersects_grid(const Grid::SdfGrid& grid, const float3& ray_start, const float3& ray_dir, float3& point, float3& normal)
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
		while (step > 0.001f)
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

	void calc_grid_color(const AppData& app_data, const Grid::SdfGrid& grid, const float3& point, const float3& normal, uint32_t& result)
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

namespace DrawGrid
{
	Data::Data(const char* grid_path)
	{
		Grid::load_sdf_grid(m_grid, grid_path);
	}

	void draw_grid(AppData& app_data, const Camera& camera)
	{
		const Data* const data = (const Data*)app_data.m_data.get();
		const Grid::SdfGrid& grid = data->m_grid;

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