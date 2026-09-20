#pragma once
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 oc = ray.origin - sphere.origin;

			// quadratic equation
			const float A = Vector3::Dot(ray.direction, ray.direction);
			const float B = 2.0f * Vector3::Dot(ray.direction, oc);
			const float C = Vector3::Dot(oc, oc) - sphere.radius * sphere.radius;

			const float DISCRIMINANT = B * B - 4 * A * C;
			if (DISCRIMINANT < 0) return false;

			const float SQRT_DIST = sqrtf(DISCRIMINANT);

			// calculate distance along the ray to the intersection point
			float t = (-B - SQRT_DIST) / (2 * A);

			// if first intersection is behind us or too close- try the other
			if (t <= ray.min)
			{
				t = (-B + SQRT_DIST) / (2 * A);
			}

			if (t < ray.min || t > ray.max)	return false;

			if (!ignoreHitRecord)
			{
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
				hitRecord.materialIndex = sphere.materialIndex;
			}

			return true;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float DENOM = Vector3::Dot(ray.direction, plane.normal);
			if (DENOM > -0.00001f && DENOM < 0.00001f) return false;

			// calculate distance along the ray to the intersection point
			float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / DENOM;

			if (t > ray.min && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.origin = ray.origin + ray.direction * t;
					hitRecord.normal = plane.normal;
				}
				return true;
			}

			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}

#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3& N {triangle.normal};
			const Vector3& V { ray.direction };

			// does the ray intersect with the plane?
			const float NdotV = Vector3::Dot(N, V);
			if (AreEqual(NdotV, 0)) return false;

			// culling
			bool isShadowRay = ignoreHitRecord;

			TriangleCullMode effectiveCull = triangle.cullMode;
			if (isShadowRay)
			{
				// invert cull mode for shadow rays
				if (effectiveCull == TriangleCullMode::BackFaceCulling) effectiveCull = TriangleCullMode::FrontFaceCulling;
				else if (effectiveCull == TriangleCullMode::FrontFaceCulling) effectiveCull = TriangleCullMode::BackFaceCulling;
			}

			if (effectiveCull == TriangleCullMode::BackFaceCulling && NdotV > 0.f) return false;
			if (effectiveCull == TriangleCullMode::FrontFaceCulling && NdotV < 0.f) return false;

			// calculate distance along the ray to the intersection point
			const Vector3 L{ triangle.v0 - ray.origin };
			const float LdotN = Vector3::Dot(L, N);
			float t = LdotN / NdotV;

			if (t  < ray.min || t > ray.max) return false;

			Vector3 P = ray.origin + V * t;

			//is the intersection point inside the triangle?
			Vector3 p1{P - triangle.v0};
			Vector3 e1{triangle.v1 - triangle.v0};
			Vector3 p2{ P - triangle.v1 };
			Vector3 e2{ triangle.v2 - triangle.v1 };
			Vector3 p3{ P - triangle.v2 };
			Vector3 e3{ triangle.v0 - triangle.v2 };

			if (Vector3::Dot(Vector3::Cross(e1, p1), N) < 0 ||
				Vector3::Dot(Vector3::Cross(e2, p2), N) < 0 ||
				Vector3::Dot(Vector3::Cross(e3, p3), N) < 0) return false;

			if (!ignoreHitRecord)
			{
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = ray.origin + ray.direction * t;
				
				// check if its hitting backface
				bool isBackFace = Vector3::Dot(ray.direction, triangle.normal) > 0.0f;
				hitRecord.normal = isBackFace ? -triangle.normal : triangle.normal;
			}
			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);

		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			float tx1 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// slabtest
			if (!SlabTest_TriangleMesh(mesh, ray))
			{
				return false;
			}
			if (mesh.transformedPositions.empty() || mesh.indices.empty()) return false;


			HitRecord temp{};

			const size_t NUM_INDICES = mesh.indices.size();
			for (size_t triIndex{}; triIndex < NUM_INDICES; triIndex+=3)
			{
				const int i0 = mesh.indices[triIndex+0];
				const int i1 = mesh.indices[triIndex+1];
				const int i2 = mesh.indices[triIndex+2];

				Triangle triangle{};
				triangle.v0 = mesh.transformedPositions[i0];
				triangle.v1 = mesh.transformedPositions[i1];
				triangle.v2 = mesh.transformedPositions[i2];

				if (triIndex/3 < mesh.transformedNormals.size())
				{
					triangle.normal = mesh.transformedNormals[triIndex/3];
				}
				else
				{
					triangle.normal = Vector3::Cross(triangle.v1 - triangle.v0, triangle.v2 - triangle.v0).Normalized();
				}
				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;


				if (ignoreHitRecord)
				{
					if (HitTest_Triangle(triangle, ray))
					{
						return true;
					}
				}
				else
				{
					if (HitTest_Triangle(triangle, ray, temp) && hitRecord.t > temp.t)
					{
						hitRecord = temp;
					}
				}
			}

			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			switch (light.type)
			{
			case LightType::Point:
				return light.origin - origin;

			case LightType::Directional:
				return -light.direction * FLT_MAX;

			default:
				return Vector3{};
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB radiance{};

			if (light.type == LightType::Point)
			{
				const Vector3 LIGHT_DIR = light.origin - target;
				const float DISTANCE_SQUARED = std::max(LIGHT_DIR.SqrMagnitude(), 0.00001f);
				radiance = light.color * (light.intensity / DISTANCE_SQUARED);
			}
			else 
			{
				radiance = light.color * light.intensity;
			}

			return radiance;
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof())
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if (std::isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (std::isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}