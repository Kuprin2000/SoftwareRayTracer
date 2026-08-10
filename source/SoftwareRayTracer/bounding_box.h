#pragma once
#include <cmath>
#include <algorithm>
#include "LiteMath/LiteMath.h"

using LiteMath::float3;

class BoundingBox
{
public:
	void setVoid()
	{
		m_data[0] = FLT_MAX;
		m_data[1] = -FLT_MAX;
		m_data[2] = FLT_MAX;
		m_data[3] = -FLT_MAX;
		m_data[4] = FLT_MAX;
		m_data[5] = -FLT_MAX;
	}

	void setTriangle(const float3* vertices_coords, float gap)
	{
		m_data[0] = std::min({ vertices_coords[0][0], vertices_coords[1][0], vertices_coords[2][0] }) - gap;
		m_data[1] = std::max({ vertices_coords[0][0], vertices_coords[1][0], vertices_coords[2][0] }) + gap;
		m_data[2] = std::min({ vertices_coords[0][1], vertices_coords[1][1], vertices_coords[2][1] }) - gap;
		m_data[3] = std::max({ vertices_coords[0][1], vertices_coords[1][1], vertices_coords[2][1] }) + gap;
		m_data[4] = std::min({ vertices_coords[0][2], vertices_coords[1][2], vertices_coords[2][2] }) - gap;
		m_data[5] = std::max({ vertices_coords[0][2], vertices_coords[1][2], vertices_coords[2][2] }) + gap;
	}

	void setBox(const float3& min_corner, const float3& max_corner)
	{
		m_data[0] = min_corner.x;
		m_data[1] = max_corner.x;
		m_data[2] = min_corner.y;
		m_data[3] = max_corner.y;
		m_data[4] = min_corner.z;
		m_data[5] = max_corner.z;
	}

	void addBndBox(const BoundingBox& bnd_box)
	{
		m_data[0] = (bnd_box.m_data[0] < m_data[0]) ? bnd_box.m_data[0] : m_data[0];
		m_data[1] = (bnd_box.m_data[1] > m_data[1]) ? bnd_box.m_data[1] : m_data[1];
		m_data[2] = (bnd_box.m_data[2] < m_data[2]) ? bnd_box.m_data[2] : m_data[2];
		m_data[3] = (bnd_box.m_data[3] > m_data[3]) ? bnd_box.m_data[3] : m_data[3];
		m_data[4] = (bnd_box.m_data[4] < m_data[4]) ? bnd_box.m_data[4] : m_data[4];
		m_data[5] = (bnd_box.m_data[5] > m_data[5]) ? bnd_box.m_data[5] : m_data[5];
	}

	_NODISCARD bool contains(const float3& point_coords) const
	{
		return point_coords[0] >= m_data[0] &&
			point_coords[0] <= m_data[1] &&
			point_coords[1] >= m_data[2] &&
			point_coords[1] <= m_data[3] &&
			point_coords[2] >= m_data[4] &&
			point_coords[2] <= m_data[5];
	}

	_NODISCARD bool intersectsRay(const float3& ray_start, const float3& ray_dir, float3& near_point, float3& far_point) const
	{
		float t_min = 0.0f;
		float t_max = FLT_MAX;

		for (int i = 0; i < 3; i++)
		{
			const float ood = 1.0f / ray_dir[i];
			const float min = m_data[2 * i];
			const float max = m_data[2 * i + 1];

			float t1 = (min - ray_start[i]) * ood;
			float t2 = (max - ray_start[i]) * ood;

			if (ood < 0.0f)
			{
				std::swap(t1, t2);
			}

			t_min = std::max(t_min, t1);
			t_max = std::min(t_max, t2);
			if (t_min > t_max)
			{
				return false;
			}
		}

		near_point = ray_start + t_min * ray_dir;
		far_point = ray_start + t_max * ray_dir;

		return true;
	}

	_NODISCARD float distanceFromPoint(const float3& point) const
	{
		float result_squared = 0.0f;

		for (int i = 0; i < 3; i++)
		{
			const float min = m_data[2 * i];
			const float max = m_data[2 * i + 1];
			const float v = point[i];

			if (v < min)
			{
				result_squared += (min - v) * (min - v);
			}
			if (v > max)
			{
				result_squared += (v - max) * (v - max);
			}
		}

		return sqrtf(result_squared);
	}

	_NODISCARD float getVolume() const
	{
		return (m_data[1] - m_data[0]) * (m_data[3] - m_data[2]) * (m_data[5] - m_data[4]);
	}

	_NODISCARD float3 getMinCorner() const
	{
		return float3(m_data[0], m_data[2], m_data[4]);
	}

	_NODISCARD float3 getMaxCorner() const
	{
		return float3(m_data[1], m_data[3], m_data[5]);
	}

	_NODISCARD BoundingBox getOctalPart(int sibling_order) const
	{
		float3 min_corner = getMinCorner();
		float3 max_corner = getMaxCorner();
		const float3 center = 0.5f * (min_corner + max_corner);
		const float3 size = 0.5f * (max_corner - min_corner);

		switch (sibling_order)
		{
		case 0:
			max_corner = min_corner + size;
			break;
		case 1:
			min_corner.z = center.z;
			max_corner = min_corner + size;
			break;
		case 2:
			min_corner.y = center.y;
			max_corner = min_corner + size;
			break;
		case 3:
			min_corner.y = center.y;
			min_corner.z = center.z;
			max_corner = min_corner + size;
			break;
		case 4:
			min_corner.x = center.x;
			max_corner = min_corner + size;
			break;
		case 5:
			min_corner.x = center.x;
			min_corner.z = center.z;
			max_corner = min_corner + size;
			break;
		case 6:
			min_corner.x = center.x;
			min_corner.y = center.y;
			max_corner = min_corner + size;
			break;
		case 7:
			min_corner = center;
			max_corner = min_corner + size;
			break;
		}

		BoundingBox result;
		result.setBox(min_corner, max_corner);

		return result;
	}

	_NODISCARD static float calcBndBoxesOverlap(const BoundingBox& box_a, const BoundingBox& box_b)
	{
		float overlap_dimensions[3] = { 0.0f, 0.0f, 0.0f };

		overlap_dimensions[0] = fmin(box_a.m_data[1], box_b.m_data[1]) - fmax(box_a.m_data[0], box_b.m_data[0]);
		if (overlap_dimensions[0] <= 0.0f)
		{
			return 0.0f;
		}

		overlap_dimensions[1] = fmin(box_a.m_data[3], box_b.m_data[3]) - fmax(box_a.m_data[2], box_b.m_data[2]);
		if (overlap_dimensions[1] <= 0.0f)
		{
			return 0.0f;
		}

		overlap_dimensions[2] = fmin(box_a.m_data[5], box_b.m_data[5]) - fmax(box_a.m_data[4], box_b.m_data[4]);
		if (overlap_dimensions[2] <= 0.0f)
		{
			return 0.0f;
		}

		return overlap_dimensions[0] * overlap_dimensions[1] * overlap_dimensions[2];
	}

	_NODISCARD static bool thereIsOVerlap(const BoundingBox& box_a, const BoundingBox& box_b)
	{
		if (box_a.m_data[1] < box_b.m_data[0] || box_a.m_data[0] > box_b.m_data[1])
		{
			return false;
		}

		if (box_a.m_data[3] < box_b.m_data[2] || box_a.m_data[2] > box_b.m_data[3])
		{
			return false;
		}

		if (box_a.m_data[5] < box_b.m_data[4] || box_a.m_data[4] > box_b.m_data[5])
		{
			return false;
		}

		return true;
	}

private:
	// x_min, x_max, y_min, y_max, z_min, z_max
	float m_data[6] = { FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX };
};