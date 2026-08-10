#pragma once
#include "app_data.h"
#include "camera.h"
#include "octree.h"

namespace DrawOctree
{
	struct Data
	{
		Data(const char* octree_path, Octree::SamplingMode mode);

		Octree::SdfOctree m_octree;
		Octree::SamplingMode m_samplig_mode = Octree::SamplingMode::SPHERE_TRACING;
	};

	void draw_octree(AppData& app_data, const Camera& camera);
}