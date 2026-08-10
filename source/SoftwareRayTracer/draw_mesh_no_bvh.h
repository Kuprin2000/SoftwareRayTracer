#pragma once
#include"mesh.h"
#include "app_data.h"
#include "camera.h"


using LiteMath::float4;

namespace DrawMeshNoBVH
{
	struct Data
	{
		Data(const char* mesh_path, float4 plane_parameters);

		cmesh4::SimpleMesh m_mesh;
		float4 m_plane_parameters;

	private:
		void getMeshExtent(float4& min_corner, float4& max_corner) const;

		void resizeMesh();
	};


	void draw_mesh_no_bvh(AppData& app_data, const Camera& camera);
}