#include "draw_plane.h"
#include <vector>

using LiteMath::float3;
using LiteMath::float4;

namespace
{
	_NODISCARD std::tuple<float3, float3 > unpack_plane(float x, float y, float z, float d)
	{
		float3 normal(x, y, z);
		normal = normalize(normal);

		if (fabs(d) < FLT_EPSILON)
		{
			return std::make_tuple(normal, float3(0.0f, 0.0f, 0.0f));
		}

		if (fabs(z) > FLT_EPSILON)
		{
			return std::make_tuple(normal, normalize(float3(0.0f, 0.0f, -d / z)));
		}
		else if (fabs(y) > FLT_EPSILON)
		{
			return std::make_tuple(normal, normalize(float3(0.0f, -d / y, 0.0f)));
		}

		// else if (fabs(x) > FLT_EPSILON)
		return std::make_tuple(normal, normalize(float3(-d / x, 0.0f, 0.0f)));
	}

	_NODISCARD bool ray_intersects_plane(const float3& plane_normal, const float3& plane_point,
		const float3& ray_point, const float3& ray_dir, float3& intersection_point)
	{
		const float3 normalized_ray_point = normalize(ray_point);
		const float denom = dot(plane_normal, ray_dir);

		if (fabs(denom) > FLT_EPSILON)
		{
			const float3 plane_point_to_ray_point = plane_point - normalized_ray_point;
			const float t = dot(plane_point_to_ray_point, plane_normal) / denom;
			intersection_point = ray_point + t * ray_dir;
			return t >= 0.0f;
		}

		return false;
	}
}

namespace DrawPlane
{
	Data::Data(float x, float y, float z, float d): m_x(x), m_y(y), m_z(z), m_d(d)
	{
	}

	void draw_plane(AppData& app_data, const Camera& camera)
	{
		const Data* const data = (const Data*)app_data.m_data.get();
		const auto [plane_normal, plane_point] = unpack_plane(data->m_x, data->m_y, data->m_z, data->m_d);

		const float3 camera_pos = camera.m_position;
		const float3x3 camera_mat = camera.create_camera_rotation_mat();

		AlignedVector::AlignedVector<uint32_t>& frame_buffer = app_data.m_frame_buffer;

		float drak_side_intencity = app_data.ambient_brightnes;
		drak_side_intencity = std::clamp(drak_side_intencity, 0.0f, 255.0f);
		const uint8_t drak_side_intencity_int = (uint8_t)std::round(drak_side_intencity);

		float bright_side_intencity = app_data.light_brightnes * std::max(0.0f, dot(app_data.light_dir, plane_normal)) + app_data.ambient_brightnes;
		bright_side_intencity = std::clamp(bright_side_intencity, 0.0f, 255.0f);
		const uint8_t bright_intencity_int = (uint8_t)std::round(bright_side_intencity);

#pragma omp parallel for
		for (int i = 0; i < app_data.m_height; ++i)
		{
			for (int j = 0; j < app_data.m_width; ++j)
			{
				float3 ray_dir = camera.create_ray_dir((float)j, (float)i, app_data.m_width, app_data.m_height);
				ray_dir = camera_mat * ray_dir;

				float3 intersection_point;
				if (ray_intersects_plane(plane_normal, plane_point, camera_pos, ray_dir, intersection_point))
				{
					const uint32_t dst_offset = i * app_data.m_width + j;
					const float3 vec_to_camera = camera_pos - intersection_point;
					if (dot(vec_to_camera, plane_normal) * dot(app_data.light_dir, plane_normal) > 0.0f)
					{
						uint8_t* pixel = (uint8_t*)&frame_buffer[dst_offset];
						pixel[0] = bright_intencity_int;
						pixel[1] = bright_intencity_int;
						pixel[2] = bright_intencity_int;
						pixel[3] = 255u;
					}
					else
					{
						uint8_t* pixel = (uint8_t*)&frame_buffer[dst_offset];
						pixel[0] = drak_side_intencity_int;
						pixel[1] = drak_side_intencity_int;
						pixel[2] = drak_side_intencity_int;
						pixel[3] = 255u;
					}
				}
			}
		}
	}
}