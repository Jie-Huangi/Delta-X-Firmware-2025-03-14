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

#include "Constants.h"

void Constants::init() //定义类成员函数
{
	RD_F = (float)DEFAULT_RD_F;  // 机械臂几何参数 在geometry.h中
	RD_E = (float)DEFAULT_RD_E;  // 机械臂几何参数 在geometry.h中
	RD_RF = (float)DEFAULT_RD_RF;  // 机械臂几何参数 在geometry.h中
	RD_RE = (float)DEFAULT_RD_RE;  // 机械臂几何参数 在geometry.h中

	RD_W = (float)DEFAULT_RD_W;  // 机械臂几何参数 在geometry.h中
	RD_U = (float)DEFAULT_RD_U;  // 机械臂几何参数 在geometry.h中
	RD_V = (float)DEFAULT_RD_V;  // 机械臂几何参数 在geometry.h中

	MOVING_AREA_X = (float)DEFAULT_MOVING_AREA_X;  // 机械臂几何参数 在geometry.h中
	MOVING_AREA_Y = (float)DEFAULT_MOVING_AREA_Y;  // 机械臂几何参数 在geometry.h中
	MOVING_AREA_Z = (float)DEFAULT_MOVING_AREA_Z;  // 机械臂几何参数 在geometry.h中

	MOVING_AREA_LARGEST_DIAMETER = (float)DEFAULT_MOVING_AREA_LARGEST_DIAMETER;   // 最大移动直径
	// 机械臂几何参数 在geometry.h中

	Velocity = (float)DEFAULT_VELOCITY;			//mm/s   // 默认速度 在config.h中
	Acceleration = (float)DEFAULT_ACCELERATION;         // 默认加速度 在config.h中
	
	MovingHomeSpeed = (float)DEFAULT_MOVING_HOME_SPEED;  //回到初始位置（home）速度，在config.h中

	BeginEndVelocity = (float)DEFAULT_BEGIN_VELOCITY;  //默认开始速度，在config.h中

	EntryVelocity = (float)DEFAULT_ENTRY_VELOCITY;  //默认entry速度，在config.h中
	ExitVelocity = (float)DEFAULT_EXIT_VELOCITY;   //默认exit速度，在config.h中

	IsMoveWithTheAbsoluteCoordinates = true;   // constant.h 中的bool值，在constants.h中
	
	IsExecutedGcode = false;   // constant.h 中的bool值，在constants.h中

	End_Effector = USE_VACUUM;  // enum.h（枚举）中的新定义类型，在enum.h中

	// USE_VACUUM = 0,
	// USE_CLIP,
	// USE_PEN,
	// USE_LASER,
	// USE_PRINTER,
	// USE_CUSTOM

	MMPerLinearSegment = (float)MM_PER_LINEAR_SEGMENT; //线性插值，在config.h中

	MMPerArcSegment = (float)MM_PER_ARC_SEGMENT;  // 圆弧插值，在config.h中
}

void Constants::ResetData() //定义类成员函数
{
	init();
}


Constants Data;  //创建一个名为Data的Constants对象。

