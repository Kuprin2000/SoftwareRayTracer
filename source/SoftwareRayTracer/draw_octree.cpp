#include "draw_octree.h"
#include "bounding_box.h"
#include <cassert>

namespace
{
	_NODISCARD bool ray_intersects_octree(const Octree::SdfOctree& octree, Octree::SamplingMode mode,
		const float3& ray_start, const float3& ray_dir, float3& point, float3& normal)
	{
		point = ray_start;
		Octree::SamplingResult status = Octree::SamplingResult::STEP;

		while (status == Octree::SamplingResult::STEP)
		{
			status = octree.sample(point, ray_dir, mode, point, normal);
		}

		if (status == Octree::SamplingResult::INTERSECTION)
		{
			return true;
		}

		return false;
	}

	void calc_octree_color(const AppData& app_data, const Octree::SdfOctree& octree, Octree::SamplingMode mode,
		const float3& point, const float3& normal, uint32_t& result)
	{
		float3 dummy_point;
		float3 dummy_normal;
		uint8_t light_intencity_int = 0u;

		if (!ray_intersects_octree(octree, mode, point, app_data.light_dir, dummy_point, dummy_normal))
		{
			float light_intencity =
				app_data.light_brightnes * std::max(0.0f, dot(app_data.light_dir, normal)) + app_data.ambient_brightnes;
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

namespace DrawOctree
{
	Data::Data(const char* octree_path, Octree::SamplingMode mode) : m_samplig_mode(mode)
	{
		Octree::load_sdf_octree(m_octree, octree_path);
	}

	void draw_octree(AppData& app_data, const Camera& camera)
	{
		const Data* const data = (const Data*)app_data.m_data.get();
		const Octree::SdfOctree& octree = data->m_octree;
		const Octree::SamplingMode mode = data->m_samplig_mode;

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
				if (!ray_intersects_octree(octree, mode, camera_pos, ray_dir, intersection_point, intersection_normal))
				{
					continue;
				}

				intersection_point += 0.01f * intersection_normal;
				const uint32_t dst_offset = i * app_data.m_width + j;
				calc_octree_color(app_data, octree, mode, intersection_point, intersection_normal, frame_buffer[dst_offset]);
			}
		}
	}
}