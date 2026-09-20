#pragma once
#include "Math.h"
#include "DataTypes.h"
#include "BRDFs.h"
#include <iostream>

namespace dae
{
#pragma region Material BASE
	class Material
	{
	public:
		Material() = default;
		virtual ~Material() = default;

		Material(const Material&) = delete;
		Material(Material&&) noexcept = delete;
		Material& operator=(const Material&) = delete;
		Material& operator=(Material&&) noexcept = delete;

		/**
		 * \brief Function used to calculate the correct color for the specific material and its parameters
		 * \param hitRecord current hitrecord
		 * \param l light direction
		 * \param v view direction
		 * \return color
		 */
		virtual ColorRGB Shade(const HitRecord& hitRecord = {}, const Vector3& l = {}, const Vector3& v = {}) = 0;
	};
#pragma endregion

#pragma region Material SOLID COLOR
	//SOLID COLOR
	//===========
	class Material_SolidColor final : public Material
	{
	public:
		Material_SolidColor(const ColorRGB& color) : m_Color(color) {}

		ColorRGB Shade(const HitRecord& hitRecord, const Vector3& l, const Vector3& v) override
		{
			return m_Color;
		}

	private:
		ColorRGB m_Color{ colors::White };
	};
#pragma endregion

#pragma region Material LAMBERT
	//LAMBERT
	//=======
	class Material_Lambert final : public Material
	{
	public:
		Material_Lambert(const ColorRGB& diffuseColor, float diffuseReflectance) :
			m_DiffuseColor(diffuseColor), m_DiffuseReflectance(diffuseReflectance) {}

		ColorRGB Shade(const HitRecord& hitRecord = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			return BRDF::Lambert(m_DiffuseReflectance, m_DiffuseColor);
		}

	private:
		ColorRGB m_DiffuseColor{ colors::White };
		float m_DiffuseReflectance{ 1.f }; //kd
	};
#pragma endregion

#pragma region Material LAMBERT PHONG
	//LAMBERT-PHONG
	//=============
	class Material_LambertPhong final : public Material
	{
	public:
		Material_LambertPhong(const ColorRGB& diffuseColor, float kd, float ks, float phongExponent) :
			m_DiffuseColor(diffuseColor), m_DiffuseReflectance(kd), m_SpecularReflectance(ks),
			m_PhongExponent(phongExponent) {}

		ColorRGB Shade(const HitRecord& hitRecord = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			return BRDF::Lambert(m_DiffuseReflectance, m_DiffuseColor) 
				+ BRDF::Phong(m_SpecularReflectance, m_PhongExponent, l, v, hitRecord.normal);
		}

	private:
		ColorRGB m_DiffuseColor;
		float m_DiffuseReflectance; //kd
		float m_SpecularReflectance; //ks
		float m_PhongExponent; //Phong Exponent
	};
#pragma endregion

#pragma region Material COOK TORRENCE
	//COOK TORRENCE
	class Material_CookTorrence final : public Material
	{
	public:
		Material_CookTorrence(const ColorRGB& albedo, float metalness, float roughness) :
			m_Albedo(albedo), m_Metalness(metalness), m_Roughness(roughness) {}

		ColorRGB Shade(const HitRecord& hitRecord = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			const ColorRGB F0 = (m_Metalness == 0.f) ? ColorRGB{ 0.04f, 0.04f, 0.04f } : m_Albedo;
			const float DENOM = std::max((v + l).Magnitude(), 0.00001f);
			const Vector3 HALF_V = (v + l) / DENOM;
			const ColorRGB F = BRDF::FresnelFunction_Schlick(HALF_V, v, F0);
			const float D = BRDF::NormalDistribution_GGX(hitRecord.normal, HALF_V, m_Roughness);
			const float G = BRDF::GeometryFunction_Smith(hitRecord.normal, v, l, m_Roughness);
			const float VdotN = std::max(Vector3::Dot(v, hitRecord.normal), 0.f);
			const float LdotN = std::max(Vector3::Dot(l, hitRecord.normal), 0.f);
			const float DENOM2 = std::max(4 * (VdotN * LdotN), 0.00001f);
			const ColorRGB SPECULAR = (D * F * G) / DENOM2;

			const ColorRGB KD = (ColorRGB{ 1.f, 1.f, 1.f } - F) * (1.f - m_Metalness);
			const ColorRGB DIFFUSE = BRDF::Lambert(KD, m_Albedo);

			return DIFFUSE + SPECULAR;
		}

	private:
		ColorRGB m_Albedo{ 0.955f, 0.637f, 0.538f }; //Copper
		float m_Metalness{ 1.0f };
		float m_Roughness{ 0.1f }; // [1.0 > 0.0] >> [ROUGH > SMOOTH]
	};
#pragma endregion
}
