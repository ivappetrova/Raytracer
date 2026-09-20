#define PARALLEL_EXECUTION
//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <execution>

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float INV_WIDTH = 1.0f / static_cast<float>(m_Width);
	const float INV_HEIGHT = 1.0f / static_cast<float>(m_Height);
	const float ASPECT_RATIO = static_cast<float>(m_Width) / static_cast<float>(m_Height);
	const float FOV_ANGLE = camera.fovAngle * TO_RADIANS;
	const float FOV = tan(FOV_ANGLE / 2.f);

#if defined (PARALLEL_EXECUTION)
	// Parallel logic
	const uint32_t NUM_PIXELS{ static_cast<uint32_t>(m_Width * m_Height) };
	std::vector<uint32_t>pixelIndices{};
	pixelIndices.reserve(NUM_PIXELS);
	for (uint32_t pixelIndex{}; pixelIndex < NUM_PIXELS; ++pixelIndex) pixelIndices.emplace_back(pixelIndex);

	std::for_each(std::execution::par, pixelIndices.begin(), pixelIndices.end(), [&](const uint32_t i) {
		RenderPixel(pScene, i, FOV, ASPECT_RATIO, camera, INV_WIDTH, INV_HEIGHT, materials);
		});

#else 
	// Synchronous logic (no threading)
	const uint32_t NUM_PIXELS{ static_cast<uint32_t>(m_Width * m_Height) };
	for (uint32_t pixelIndex{}; pixelIndex < NUM_PIXELS; ++pixelIndex)
	{
		RenderPixel(pScene, pixelIndex, FOV, ASPECT_RATIO, camera, INV_WIDTH, INV_HEIGHT, materials);
	}
#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(const Scene* pScene, const uint32_t pixelIndex, const float fov, 
								const float aspectRatio, const Camera& camera, const float invertedWidth, 
								const float invertedHeight, const std::vector<Material*>& materials) const
{
	// calculate pixel
	const uint32_t PX{ pixelIndex % m_Width };
	const uint32_t PY{ pixelIndex / m_Width };

	// calulate view ray
	const Vector3 ORIGIN{ camera.origin };
	const Vector3 RAY_DIRECTION{ CalculateDirectionRayFromCamera(PX, PY, camera, invertedWidth, invertedHeight, aspectRatio) };
	const Ray VIEW_RAY{ ORIGIN, RAY_DIRECTION };

	ColorRGB finalColor{};

	// does the view ray hit something?
	HitRecord closestHit{};
	pScene->GetClosestHit(VIEW_RAY, closestHit);

	if (closestHit.didHit)
	{
		for (const auto& light : pScene->GetLights())
		{
			//calculate light ray
			Vector3 directionToLight = LightUtils::GetDirectionToLight(light, closestHit.origin);
			const float DISTANCE_TO_LIGHT = directionToLight.Normalize();

			// shadow check
			if (m_ShadowsEnabled)
			{
				Ray lightRay{ closestHit.origin + closestHit.normal * 0.0005f, directionToLight };
				lightRay.max = DISTANCE_TO_LIGHT;

				bool inShadow = pScene->DoesHit(lightRay);
				if (inShadow) continue;
			}

			// lighting mode logic
			const float OBSERVED_AREA = std::max(0.f, Vector3::Dot(closestHit.normal, directionToLight));
			if (m_CurrentLightingMode != LightingMode::Radiance && m_CurrentLightingMode != LightingMode::BRDF && OBSERVED_AREA <= 0) continue;

			const ColorRGB RADIANCE = LightUtils::GetRadiance(light, closestHit.origin);
			const ColorRGB BRDF = materials[closestHit.materialIndex]->Shade(closestHit, directionToLight, -RAY_DIRECTION);

			switch (m_CurrentLightingMode)
			{
			case LightingMode::ObservedArea:
				finalColor += ColorRGB{ OBSERVED_AREA, OBSERVED_AREA, OBSERVED_AREA };
				break;
			case LightingMode::Radiance:
				finalColor += RADIANCE /** OBSERVED_AREA*/;
				break;
			case LightingMode::BRDF:
				finalColor += BRDF;
				break;
			case LightingMode::Combined:
				finalColor += RADIANCE * BRDF * OBSERVED_AREA;
				break;
			}
		}
	}
	else
	{
		finalColor = ColorRGB{ 0.f , 0.f , 0.f }; //black
	}

	finalColor.MaxToOne();

	m_pBufferPixels[PX + (PY * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));

}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	size_t const NUM_MODES{ 4 };
	int current{ static_cast<int>(m_CurrentLightingMode) };
	current = (current + 1) % NUM_MODES;
	m_CurrentLightingMode = static_cast<LightingMode>(current);
	std::cout << "changed mode to " << static_cast<int>(m_CurrentLightingMode) << std::endl;

}

void dae::Renderer::ToggleShadows()
{
	m_ShadowsEnabled = !m_ShadowsEnabled;
	std::cout << "shadows are " << std::boolalpha << m_ShadowsEnabled << std::endl;
}

Vector3 Renderer::CalculateDirectionRayFromCamera(const int px, const int py, const Camera& camera,
											      const float invWidth, const float invHeight, const float aspectRatio) const
{
	const float VIEW_PLANE_HALF_SIZE = camera.fovScale;

	const float NDC_X{ (2.f * ((px + 0.5f) *invWidth) - 1.f) * aspectRatio * VIEW_PLANE_HALF_SIZE };
	const float NDC_Y{ (1.f - (2.f * ((py + 0.5f) *invHeight))) * VIEW_PLANE_HALF_SIZE };

	Vector3 rayDirection{ NDC_X, NDC_Y, 1.f };
	rayDirection = rayDirection.Normalized();
	rayDirection = camera.cameraToWorld.TransformVector(rayDirection).Normalized();

	return rayDirection;
}