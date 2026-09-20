#pragma once
#include <cstdint>
#include <vector>

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Camera;
	class Scene;
	class Material;

	class Vector3;
	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;
		void RenderPixel(const Scene* pScene, const uint32_t pixelIndex, const float fov, const float aspectRatio,
						 const Camera& camera, const float invertedWidth, const float invertedHeight,
						 const std::vector<Material*>& materials) const;
		bool SaveBufferToImage() const;

		void CycleLightingMode();
		void ToggleShadows();

	private:
		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};

		Vector3 CalculateDirectionRayFromCamera(const int px, const int py, const Camera& camera, const float invWidth, 
												const float invHeight, const float aspectRatio) const;

		enum class LightingMode
		{
			ObservedArea, // Lambert Cos Law
			Radiance, // Incident Radiance
			BRDF, // Scattering of the light
			Combined // ObservedArea*Radiance*BRDF	
		};

		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
		bool m_ShadowsEnabled{ true };

	};
}
