/**
 * Delta X Firmware
 * Copyright (c) 2020 DeltaXFirmware [https://github.com/deltaxrobot/Delta-X-Firmware]
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "config.h"

typedef enum  // 枚举
{
	THETA_1 = 0,
	THETA_2,
	THETA_3,
#ifdef DELTA_AXIS_4
	AXIS_4,
#endif // DELTA_AXIS_4

#ifdef DELTA_AXIS_5
	AXIS_5,
#endif // DELTA_AXIS_5

#ifdef CLIP
	CLAMP
#endif // CLIP
}AXIS;

#ifdef REVERSE_DIRECTION
typedef enum   //枚举
{
	DECREASE = 0,
	INCREASE
}VARIATION;

#else
typedef enum
{
	INCREASE = 0,
	DECREASE
}VARIATION;

#endif // reverse direction

typedef enum  //枚举
{
	USE_STEPPER = 0,
	USE_SERVO
}MOTIVE;

typedef enum  //枚举   // 打印头 
{
	USE_VACUUM = 0,
	USE_CLIP,
	USE_PEN,
	USE_LASER,
	USE_PRINTER,
	USE_CUSTOM
}END_EFFECTOR;   // 打印头

typedef enum  //枚举
{
	BEGIN_SEG = 0,
	END_SEG,
	CHANGE_VELOCITY_SEG,
	FIXED_VELICITY_SEG,
	MOVING_HOME
}TYPE_SEGMENT;

typedef enum //枚举
{
	RED = 0,
	GREEN,
	BLUE,
}COLOR;    // 颜色

//#define A(CODE) " " CODE "\n\t"
