#pragma once
#include <fstream>
#include "aligned_vector.h"
#include "LiteMath/LiteMath.h"

namespace Grid
{
	using LiteMath::uint3;
	using LiteMath::float3;

	struct SdfGrid
	{
		uint3 m_size;
		float3 m_step;
		AlignedVector::AlignedVector<float> m_data;	// size.x*size.y*size.z values

		SdfGrid() = default;

		SdfGrid(int size) : m_size(size)
		{
			m_step = { 2.0f / (float)m_size.x, 2.0f / (float)m_size.y, 2.0f / (float)m_size.z };
			m_data.resize(m_size.x * m_size.y * m_size.z, 0.0f);
		}

		_NODISCARD bool contains(const float3& point) const;

		_NODISCARD virtual float sample_value(const float3& point) const;

		_NODISCARD float3 sample_grad(const float3& point) const;

		void set(uint32_t x, uint32_t y, uint32_t z, float value);

	protected:
		_NODISCARD float get(uint32_t x, uint32_t y, uint32_t z) const;

	private:
		_NODISCARD float calc_diff(const float3& point, const float3& step_forward_point,
			const float3& step_backward_point, const float step) const;
	};

	void load_sdf_grid(SdfGrid& scene, const std::string& path);

	void save_sdf_grid(const SdfGrid& scene, const std::string& path);
}
