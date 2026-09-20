#pragma once
#include "Math.h"
#include <iostream>

namespace dae
{
	namespace BRDF
	{
		/**
		 * \param kd Diffuse Reflection Coefficient
		 * \param cd Diffuse Color
		 * \return Lambert Diffuse Color
		 */
		static ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			return (cd * kd)/ PI;
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{
			return (cd * kd) / PI;
		}

		/**
		 * \brief todo
		 * \param ks Specular Reflection Coefficient
		 * \param exp Phong Exponent
		 * \param l Incoming (incident) Light Direction
		 * \param v View Direction
		 * \param n Normal of the Surface
		 * \return Phong Specular Color
		 */
		static ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			const float LdotN = std::max(Vector3::Dot(l, n), 0.f);
			const Vector3 REFLECT = l - 2.f * LdotN * n;
			const float COS_A = std::max(Vector3::Dot(REFLECT, v), 0.f);
			const float SPECULAR = ks * std::pow(COS_A, exp);
			return ColorRGB{ SPECULAR, SPECULAR, SPECULAR };
		}

		/**
		 * \brief BRDF Fresnel Function >> Schlick
		 * \param h Normalized Halfvector between View and Light directions
		 * \param v Normalized View direction
		 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
		 * \return
		 */
		static ColorRGB FresnelFunction_Schlick(const Vector3& h, const Vector3& v, const ColorRGB& f0)
		{
			const ColorRGB COLOR_1{ 1.f, 1.f, 1.f };
			const float HdotV = std::max(Vector3::Dot(h, v), 0.f);
			return f0 + (COLOR_1 - f0) * (std::pow(1.f - HdotV, 5));
		}

		/**
		 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation - squared(roughness))
		 * \param n Surface normal
		 * \param h Normalized half vector
		 * \param roughness Roughness of the material
		 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
		 */
		static float NormalDistribution_GGX(const Vector3& n, const Vector3& h, float roughness)
		{
			const float A = roughness * roughness;
			const float A2 = A * A;
			const float NdotH = std::max(Vector3::Dot(n, h), 0.f);
			const float DENOM = std::max(((NdotH * NdotH) * (A2 - 1.0f) + 1.0f), 0.00001f);
			return A2 / (PI * (DENOM * DENOM));
		}


		/**
		 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using SchlickGGX
		 */
		static float GeometryFunction_SchlickGGX(const Vector3& n, const Vector3& v, float roughness)
		{
			const float A = roughness * roughness;
			const float K = ((A + 1) * (A + 1)) /8;
			const float NdotV = std::max(Vector3::Dot(n, v), 0.f);
			return NdotV / ((NdotV * (1 - K)) + K);
		}

		/**
		 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param l Normalized light direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
		 */
		static float GeometryFunction_Smith(const Vector3& n, const Vector3& v, const Vector3& l, float roughness)
		{
			return GeometryFunction_SchlickGGX(n,v,roughness) * GeometryFunction_SchlickGGX(n,l,roughness);
		}

	}
}