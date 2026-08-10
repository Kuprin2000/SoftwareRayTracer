#pragma once
#include <vector>
#include <memory>
#include "LiteMath/LiteMath.h"
#include "aligned_vector.h"

using LiteMath::float3;

enum class Mode
{
	DRAW_PLANE,
	DRAW_MESH_NO_BVH,
	DRAW_MESH_BVH,
	DRAW_GRID,
	MESH_TO_GRID,
	DRAW_GRID_MINUS_SPHERE,
	DRAW_OCTREE_SPHERE_TRACING,
	DRAW_OCTREE_SPHERE_ANALYTIC,
	MESH_TO_OCTREE,
	NONE
};

struct AppData
{
	Mode m_mode = Mode::NONE;
	bool m_single_frame = false;
	int m_width = 0;
	int m_height = 0;
	AlignedVector::AlignedVector<uint32_t> m_frame_buffer;
	std::unique_ptr<void, void(*)(void*)> m_data;

	float3 light_dir = normalize(float3(0.5f, 0.5f, -0.5f));
	float light_brightnes = 255.0f;
	float ambient_brightnes = 20.0f;

	AppData(Mode mode, bool single_frame, std::unique_ptr<void, void(*)(void*)>&& data, int width, int height) :
		m_mode(mode), m_single_frame(single_frame), m_data(std::move(data)),
		m_width(width), m_height(height), m_frame_buffer(m_width* m_height, 0)
	{
	}
};