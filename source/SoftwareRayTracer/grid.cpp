#include "grid.h"

namespace Grid
{
	void load_sdf_grid(SdfGrid& scene, const std::string& path)
	{
		std::ifstream fs(path, std::ios::binary);
		fs.read((char*)&scene.m_size, 3 * sizeof(unsigned));

		scene.m_step = { 2.0f / (float)scene.m_size.x, 2.0f / (float)scene.m_size.y, 2.0f / (float)scene.m_size.z };

		scene.m_data.resize(scene.m_size.x * scene.m_size.y * scene.m_size.z);
		fs.read((char*)scene.m_data.data(), scene.m_size.x * scene.m_size.y * scene.m_size.z * sizeof(float));

		fs.close();
	}

	void save_sdf_grid(const SdfGrid& scene, const std::string& path)
	{
		std::ofstream fs(path, std::ios::binary);
		fs.write((const char*)&scene.m_size, 3 * sizeof(unsigned));
		fs.write((const char*)scene.m_data.data(), scene.m_size.x * scene.m_size.y * scene.m_size.z * sizeof(float));
		fs.flush();
		fs.close();
	}

	float SdfGrid::get(uint32_t x, uint32_t y, uint32_t z) const
	{
		return m_data[x * m_size.y * m_size.z + y * m_size.z + z];
	}

	void SdfGrid::set(uint32_t x, uint32_t y, uint32_t z, float value)
	{
		m_data[x * m_size.y * m_size.z + y * m_size.z + z] = value;
	}

	bool SdfGrid::contains(const float3& point) const
	{
		return std::min({ point.x, point.y, point.z }) >= -1.0f && std::max({ point.x, point.y, point.z }) <= 1.0f;
	}

	float SdfGrid::sample_value(const float3& point) const
	{
		if (!contains(point))
		{
			return -FLT_MAX;
		}

		const float3 position_in_grid = (point + 1.0f - 0.5f * m_step) / m_step;
		const uint3 min_corner((uint32_t)position_in_grid.x, (uint32_t)position_in_grid.y, (uint32_t)position_in_grid.z);
		uint3 max_corner = {
			std::min(min_corner.x + 1u, m_size.x - 1u),
			std::min(min_corner.y + 1u, m_size.y - 1u),
			std::min(min_corner.z + 1u, m_size.z - 1u)
		};

		const float3 frac(
			position_in_grid.x - min_corner.x,
			position_in_grid.y - min_corner.y,
			position_in_grid.z - min_corner.z
		);

		float result = 0.0f;
		result += get(min_corner.x, min_corner.y, min_corner.z) * (1.0f - frac.x) * (1.0f - frac.y) * (1.0f - frac.z);
		result += get(min_corner.x, min_corner.y, max_corner.z) * (1.0f - frac.x) * (1.0f - frac.y) * frac.z;
		result += get(min_corner.x, max_corner.y, min_corner.z) * (1.0f - frac.x) * frac.y * (1.0f - frac.z);
		result += get(min_corner.x, max_corner.y, max_corner.z) * (1.0f - frac.x) * frac.y * frac.z;
		result += get(max_corner.x, min_corner.y, min_corner.z) * frac.x * (1.0f - frac.y) * (1.0f - frac.z);
		result += get(max_corner.x, min_corner.y, max_corner.z) * frac.x * (1.0f - frac.y) * frac.z;
		result += get(max_corner.x, max_corner.y, min_corner.z) * frac.x * frac.y * (1.0f - frac.z);
		result += get(max_corner.x, max_corner.y, max_corner.z) * frac.x * frac.y * frac.z;

		return result;
	}

	float3 SdfGrid::sample_grad(const float3& point) const
	{
		if (!contains(point))
		{
			return float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		}

		float3 result;

		float3 step_forward_point = point + float3(m_step.x, 0.0f, 0.0f);
		float3 step_backward_point = point - float3(m_step.x, 0.0f, 0.0f);
		result.x = calc_diff(point, step_forward_point, step_backward_point, m_step.x);

		step_forward_point = point + float3(0.0, m_step.y, 0.0f);
		step_backward_point = point - float3(0.0f, m_step.y, 0.0f);
		result.y = calc_diff(point, step_forward_point, step_backward_point, m_step.y);

		step_forward_point = point + float3(0.0f, 0.0f, m_step.z);
		step_backward_point = point - float3(0.0f, 0.0f, m_step.z);
		result.z = calc_diff(point, step_forward_point, step_backward_point, m_step.z);

		return normalize(result);
	}

	float SdfGrid::calc_diff(const float3& point, const float3& step_forward_point, const float3& step_backward_point, const float step) const
	{
		const bool contains_forward_point = contains(step_forward_point);
		const bool contains_backward_point = contains(step_backward_point);

		if (contains_forward_point && contains_backward_point)
		{
			return (sample_value(step_forward_point) - sample_value(step_backward_point)) / (step * 2.0f);
		}

		if (contains_forward_point)
		{
			return (sample_value(step_forward_point) - sample_value(point)) / step;
		}

		if (contains_backward_point)
		{
			return (sample_value(point) - sample_value(step_backward_point)) / step;
		}

		return -FLT_MAX;
	}
}
