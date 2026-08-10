#include <vector>
#include <iostream>
#include <string>
#include <tuple>
#include <iomanip>
#include "SDL.h"
#include "LiteMath/LiteMath.h"
#include "LiteMath/Image2d.h"
#include "mesh.h"
#include "grid.h"
#include "draw_plane.h"
#include "draw_mesh_no_bvh.h"
#include "draw_mesh_bvh.h"
#include "draw_grid.h"
#include "mesh_to_grid.h"
#include "draw_grid_minus_sphere.h"
#include "draw_octree.h"
#include "mesh_to_octree.h"

using LiteMath::float4;

static const int SCREEN_WIDTH = 512;
static const int SCREEN_HEIGHT = 512;
static const bool SINGLE_FRAME_MODE = false;
static const char* FRAME_NAME = "frame.bmp";
static const float4 PLANE_PARAMETERS = { 0.0f, 1.0f, 0.0f, 1.0f };

// need for parallel processing
static_assert(SCREEN_WIDTH % 16 == 0);

_NODISCARD std::tuple<Mode, std::unique_ptr<void, void(*)(void*)>> parce_command_line(int num_args, char** args)
{
	// My code

	std::unique_ptr<void, void(*)(void*)> empty_result(nullptr, [](void* data) {});

	if (num_args < 2)
	{
		return { Mode::NONE, std::move(empty_result) };
	}

	std::string command = args[1];

	// plane x y z d
	if (command == "plane")
	{
		if (num_args != 6)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		const float x = std::stof(args[2]);
		const float y = std::stof(args[3]);
		const float z = std::stof(args[4]);
		const float d = std::stof(args[5]);

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawPlane::Data(x, y, z, d),
			[](void* data) { delete (DrawPlane::Data*)data; }
		);

		return { Mode::DRAW_PLANE, std::move(result) };
	}

	// mesh_no_bvh file_name
	if (command == "mesh_no_bvh")
	{
		if (num_args != 3)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawMeshNoBVH::Data(args[2], PLANE_PARAMETERS),
			[](void* data) { delete (DrawMeshBVH::Data*)data; }
		);

		cmesh4::SimpleMesh* mesh_ptr = (cmesh4::SimpleMesh*)result.get();
		if (!mesh_ptr->VerticesNum())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_MESH_NO_BVH, std::move(result) };
	}

	// mesh_bvh file_name
	if (command == "mesh_bvh")
	{
		if (num_args != 3)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawMeshBVH::Data(args[2], PLANE_PARAMETERS),
			[](void* data) { delete (DrawMeshBVH::Data*)data; }
		);

		cmesh4::SimpleMesh* mesh_ptr = &((DrawMeshBVH::Data*)result.get())->m_mesh;
		if (!mesh_ptr->VerticesNum())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_MESH_BVH, std::move(result) };
	}

	// grid file_name
	if (command == "grid")
	{
		if (num_args != 3)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawGrid::Data(args[2]),
			[](void* data) { delete (DrawGrid::Data*)data; }
		);

		const Grid::SdfGrid* grid_ptr = &((DrawGrid::Data*)result.get())->m_grid;
		if (grid_ptr->m_data.empty())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_GRID, std::move(result) };
	}

	// mesh_to_grid size input output
	if (command == "mesh_to_grid")
	{
		if (num_args != 5)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new MeshToGrid::Data(std::stoi(args[2]), args[3], args[4]),
			[](void* data) { delete (MeshToGrid::Data*)data; }
		);

		cmesh4::SimpleMesh* mesh_ptr = &((MeshToGrid::Data*)result.get())->m_mesh;
		if (!mesh_ptr->VerticesNum())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::MESH_TO_GRID, std::move(result) };
	}

	// grid_minus_sphere file_name x y z radius
	if (command == "grid_minus_sphere")
	{
		if (num_args != 7)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		const float3 center = {
			std::stof(args[3]),
			std::stof(args[4]),
			std::stof(args[5])
		};
		const float radius = std::stof(args[6]);

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawGridMinusSphere::Data(args[2], center, radius),
			[](void* data) { delete (DrawGridMinusSphere::Data*)data; }
		);

		const Grid::SdfGrid* grid_ptr = &((DrawGridMinusSphere::Data*)result.get())->m_grid;
		if (grid_ptr->m_data.empty())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_GRID_MINUS_SPHERE, std::move(result) };
	}

	// octree_sphere_tracing file_name
	if (command == "octree_sphere_tracing")
	{
		if (num_args != 3)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawOctree::Data(args[2], Octree::SamplingMode::SPHERE_TRACING),
			[](void* data) { delete (DrawOctree::Data*)data; }
		);

		const Octree::SdfOctree* tree_ptr = &((DrawOctree::Data*)result.get())->m_octree;
		if (tree_ptr->m_nodes.empty())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_OCTREE_SPHERE_TRACING, std::move(result) };
	}

	// octree_analytic file_name
	if (command == "octree_analytic")
	{
		if (num_args != 3)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new DrawOctree::Data(args[2], Octree::SamplingMode::ANALYTIC),
			[](void* data) { delete (DrawOctree::Data*)data; }
		);

		const Octree::SdfOctree* tree_ptr = &((DrawOctree::Data*)result.get())->m_octree;
		if (tree_ptr->m_nodes.empty())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::DRAW_OCTREE_SPHERE_ANALYTIC, std::move(result) };
	}

	// mesh_to_octree d input output
	if (command == "mesh_to_octree")
	{
		if (num_args != 5)
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		std::unique_ptr<void, void(*)(void*)> result(
			new MeshToOctree::Data(std::stoi(args[2]), args[3], args[4]),
			[](void* data) { delete (MeshToOctree::Data*)data; }
		);

		cmesh4::SimpleMesh* mesh_ptr = &((MeshToOctree::Data*)result.get())->m_mesh;
		if (!mesh_ptr->VerticesNum())
		{
			return { Mode::NONE, std::move(empty_result) };
		}

		return { Mode::MESH_TO_OCTREE, std::move(result) };
	}

	// TODO: new modes

	return { Mode::NONE, std::move(empty_result) };
}

void move_camera(const SDL_Event& event, Camera& camera)
{
	// My code

	const float movement_speed = 0.01f;
	const float rotation_speed = 0.01f;

	const float3 camera_front = normalize(
		float3(
			cosf(camera.m_yaw) * cosf(camera.m_pitch),
			sinf(camera.m_pitch),
			sinf(camera.m_yaw) * cosf(camera.m_pitch)
		)
	);
	const float3 camera_right = normalize(cross(camera_front, camera.m_up));
	const float3 camera_up = normalize(cross(camera_right, camera_front));

	switch (event.key.keysym.sym)
	{
	case SDLK_w:
		camera.m_position += camera_front * movement_speed;
		break;
	case SDLK_a:
		camera.m_position -= camera_right * movement_speed;
		break;
	case SDLK_s:
		camera.m_position -= camera_front * movement_speed;
		break;
	case SDLK_d:
		camera.m_position += camera_right * movement_speed;
		break;
	case SDLK_q:
		camera.m_position += camera_up * movement_speed;
		break;
	case SDLK_z:
		camera.m_position -= camera_up * movement_speed;
		break;
	case SDLK_LEFT:
		camera.m_yaw -= rotation_speed;
		camera.m_yaw = std::fmod(camera.m_yaw, PI);
		break;
	case SDLK_RIGHT:
		camera.m_yaw += rotation_speed;
		camera.m_yaw = std::fmod(camera.m_yaw, PI);
		break;
	case SDLK_UP:
		camera.m_pitch += rotation_speed;
		camera.m_pitch = std::fmod(camera.m_pitch, PI);
		break;
	case SDLK_DOWN:
		camera.m_pitch -= rotation_speed;
		camera.m_pitch = std::fmod(camera.m_pitch, PI);
		break;
	}
}

_NODISCARD bool execute(AppData& app_data, const Camera& camera)
{
	// My code

	std::fill_n(app_data.m_frame_buffer.data(), app_data.m_width * app_data.m_height, 0u);

	// TODO: new modes

	switch (app_data.m_mode)
	{
	case Mode::DRAW_PLANE:
		DrawPlane::draw_plane(app_data, camera);
		return true;
	case Mode::DRAW_MESH_NO_BVH:
		DrawMeshNoBVH::draw_mesh_no_bvh(app_data, camera);
		return true;
	case Mode::DRAW_MESH_BVH:
		DrawMeshBVH::draw_mesh_bvh(app_data, camera);
		return true;
	case Mode::DRAW_GRID:
		DrawGrid::draw_grid(app_data, camera);
		return true;
	case Mode::MESH_TO_GRID:
		MeshToGrid::mesh_to_grid(app_data);
		return false;
	case Mode::DRAW_GRID_MINUS_SPHERE:
		DrawGridMinusSphere::draw_grid_minus_sphere(app_data, camera);
		return true;
	case Mode::DRAW_OCTREE_SPHERE_TRACING:
	case Mode::DRAW_OCTREE_SPHERE_ANALYTIC:
		DrawOctree::draw_octree(app_data, camera);
		return true;
	case Mode::MESH_TO_OCTREE:
		MeshToOctree::mesh_to_octree(app_data);
		return false;
	default:
		break;
	}
}

void save_frame(const char* filename, const AlignedVector::AlignedVector<uint32_t>& frame, uint32_t width, uint32_t height)
{
	LiteImage::Image2D<uint32_t> image(width, height, frame.data());

	// Convert from ARGB to ABGR
	for (uint32_t i = 0; i < width * height; i++)
	{
		uint32_t& pixel = image.data()[i];
		auto a = (pixel & 0xFF000000);
		auto r = (pixel & 0x00FF0000) >> 16;
		auto g = (pixel & 0x0000FF00);
		auto b = (pixel & 0x000000FF) << 16;
		pixel = a | b | g | r;
	}

	if (LiteImage::SaveImage(filename, image))
	{
		std::cout << "Image saved to " << filename << std::endl;
	}
	else
	{
		std::cout << "Image could not be saved to " << filename << std::endl;
	}
}

int main(int argc, char** args)
{
	// Create window

	if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
	{
		std::cerr << "Error initializing SDL: " << SDL_GetError() << std::endl;
		return 1;
	}

	SDL_Window* window = SDL_CreateWindow("SDF Viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

	if (!window)
	{
		std::cerr << "Error creating window: " << SDL_GetError() << std::endl;
		return 1;
	}

	// Create render

	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer)
	{
		std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	// Create a texture

	SDL_Texture* texture = SDL_CreateTexture(
		renderer,
		SDL_PIXELFORMAT_ARGB8888,    // 32-bit RGBA format
		SDL_TEXTUREACCESS_STREAMING, // Allows us to update the texture
		SCREEN_WIDTH,
		SCREEN_HEIGHT);

	if (!texture)
	{
		std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError() << std::endl;
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	// Parce command line

	auto [mode, data] = parce_command_line(argc, args);
	if (mode == Mode::NONE)
	{
		std::cout << "Arguments are wrong" << std::endl;
		std::cout << "Supported arguments are:" << std::endl;
		std::cout << "to draw plane:                              plane x y z d" << std::endl;
		std::cout << "to draw mesh with mirror floor:             mesh_no_bvh file_name" << std::endl;
		std::cout << "to draw mesh with mirror floor using BVH:   mesh_bvh file_name" << std::endl;
		std::cout << "to draw grid:                               grid file_name" << std::endl;
		std::cout << "to convert mesh to grid:                    mesh_to_grid size input_file output_file" << std::endl;
		std::cout << "to draw grid - sphere:                      grid_minus_sphere file_name x y z radius" << std::endl;
		std::cout << "to draw octree using sphere tracing:        octree_sphere_tracing file_name" << std::endl;
		std::cout << "to draw octree suing analituc solution:     octree_analytic file_name" << std::endl;
		std::cout << "to convert mesh to octree:                  mesh_to_octree d input output" << std::endl;

		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	AppData app_data(mode, SINGLE_FRAME_MODE, std::move(data), SCREEN_WIDTH, SCREEN_HEIGHT);

	// Create camera

	Camera camera((float)SCREEN_WIDTH / (float)SCREEN_HEIGHT);

	// Main loop

	SDL_Event ev;
	bool running = true;

	uint32_t last_time = SDL_GetTicks();
	uint32_t frame_count = 0u;

	while (running)
	{
		while (SDL_PollEvent(&ev) != 0)
		{
			if (ev.type == SDL_QUIT || ev.key.keysym.sym == SDLK_ESCAPE)
			{
				running = false;
				break;
			}

			move_camera(ev, camera);
		}

		if (running)
		{
			running = execute(app_data, camera);
		}

		SDL_UpdateTexture(texture, nullptr, app_data.m_frame_buffer.data(), app_data.m_width * sizeof(uint32_t));

		SDL_RenderClear(renderer);

		SDL_RenderCopy(renderer, texture, nullptr, nullptr);

		SDL_RenderPresent(renderer);

		frame_count++;

		const uint32_t current_time = SDL_GetTicks();
		const uint32_t delta = current_time - last_time;
		if (delta >= 1000 || app_data.m_single_frame) {
			std::cout << "FPS: " << std::setprecision(3) << (float)frame_count * 1000.0f / float(delta);
			std::cout << " (" << (int)round((float)delta / (float)frame_count) << "ms)" << std::endl;
			frame_count = 0u;
			last_time = current_time;
		}

		if (app_data.m_single_frame)
		{
			running = false;
		}
	}

	if (app_data.m_single_frame)
	{
		save_frame(FRAME_NAME, app_data.m_frame_buffer, app_data.m_width, app_data.m_height);
	}

	SDL_DestroyWindow(window);

	SDL_Quit();

	return 0;
}
