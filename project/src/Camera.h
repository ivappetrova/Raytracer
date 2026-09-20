#pragma once
#include <SDL_keyboard.h>
#include <SDL_mouse.h>
#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera final
	{
		Camera() = default;
		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{}

		Vector3 origin;
		float fovAngle{};

		float fovScale{};       
		float m_PrevFovAngle{};

	    Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{};
		float totalYaw{};

		Matrix cameraToWorld{};
		//Mouse Input
		int mouseX{0}, mouseY{0};

		void Update(Timer* pTimer)
		{
			const float DELTA_TIME = pTimer->GetElapsed();
		
			Move(DELTA_TIME);
			Rotate(DELTA_TIME);

			// only calculate fovScale it if it changed
			if (fovAngle != m_PrevFovAngle)
			{
				const float FIELD_OF_VIEW_IN_RAD = fovAngle * TO_RADIANS;
				fovScale = tanf(FIELD_OF_VIEW_IN_RAD * 0.5f);
				m_PrevFovAngle = fovAngle;
			}

			cameraToWorld = CalculateCameraToWorld();
		}

		void Move(float deltaTime)
		{
			const uint8_t* pKEYBOARD_STATE = SDL_GetKeyboardState(nullptr);
			const float MOVE_SPEED = 7.0f;

			// Move Forward/Backward 
			if (pKEYBOARD_STATE[SDL_SCANCODE_W]) origin += forward * MOVE_SPEED * deltaTime;
			if (pKEYBOARD_STATE[SDL_SCANCODE_S]) origin -= forward * MOVE_SPEED * deltaTime;

			// Move Left/Right 
			if (pKEYBOARD_STATE[SDL_SCANCODE_A]) origin -= right * MOVE_SPEED * deltaTime;
			if (pKEYBOARD_STATE[SDL_SCANCODE_D]) origin += right * MOVE_SPEED * deltaTime;

			// Move Up/Down 
			if (pKEYBOARD_STATE[SDL_SCANCODE_E]) origin += Vector3::UnitY * MOVE_SPEED * deltaTime;
			if (pKEYBOARD_STATE[SDL_SCANCODE_Q]) origin -= Vector3::UnitY * MOVE_SPEED * deltaTime;

		}

		void Rotate(float deltaTime)
		{
			const uint32_t MOUSE_STATE = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			const float ROTATION_SPEED = 0.6f; // mouse sensitivity

			if (MOUSE_STATE & SDL_BUTTON(SDL_BUTTON_LEFT))
			{
				totalYaw += mouseX * ROTATION_SPEED * deltaTime * TO_RADIANS;
				totalPitch -= mouseY * ROTATION_SPEED * deltaTime * TO_RADIANS;
				
				Matrix rotation =Matrix::CreateRotation(totalPitch,totalYaw,0.f);

				forward = rotation.TransformVector(Vector3::UnitZ); 
				forward.Normalize();					
			}
		}

		Matrix CalculateCameraToWorld() 
		{
			//todo: W2
			right = (Vector3::Cross(Vector3::UnitY, forward)).Normalized();
			up = (Vector3::Cross(forward, right)).Normalized();

			cameraToWorld = Matrix{ right, up, forward, origin };
			return cameraToWorld;
		}
	};
}
