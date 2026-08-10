#pragma once
#include "LiteMath/LiteMath.h"

using LiteMath::float3;
using LiteMath::float3x3;

static const float PI = 3.14159265359f;

struct Camera
{
	//	Camera coordinate system is:
	//	x - right
	//	y - top
	//	z - front

	_NODISCARD Camera(float aspect_ratio) : m_aspect_ratio(aspect_ratio)
	{
	}

	_NODISCARD float3x3 create_camera_rotation_mat() const
	{
		const float3 camera_front = normalize(
			float3(
				cosf(m_yaw) * cosf(m_pitch),
				sinf(m_pitch),
				sinf(m_yaw) * cosf(m_pitch)
			)
		);
		const float3 camera_right = normalize(cross(camera_front, m_up));
		const float3 camera_up = normalize(cross(camera_right, camera_front));

		return LiteMath::make_float3x3_by_columns(
			float3(camera_right.x, camera_right.y, camera_right.z),
			float3(camera_up.x, camera_up.y, camera_up.z),
			float3(camera_front.x, camera_front.y, camera_front.z)
		);
	}

	_NODISCARD float3 create_ray_dir(float x, float y, float width, float height) const
	{
		y = height - y - 1.0f;
		float3 result(
			x + 0.5f - 0.5f * width,
			y + 0.5f - 0.5f * height,
			width / tanf(m_fov * 0.5f)
		);

		return normalize(result);
	}

	float3 m_position = { 0.0f, 0.01f, -2.0f };
	float3 m_up = { 0.0f, 1.0f, 0.0f };
	float m_pitch = 0.0f;
	float m_yaw = PI * 0.5f;
	float m_fov = PI * 0.5f;
	float m_aspect_ratio = 16.0f / 9.0f;
};
