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
//robot geometry
//
//
//
//                                          / \             RD_RF    
//                                         /   \             /
//                                        /     \ Theta2    /                                   ^ Y   ^ Z
//                                       /       \ ________/____                                |    /
//                           Theta3     /         \            /                                |   /
//                                     /     *O    \          /                                 |  /
//                        RD_F_______ /             \        /                                  | /
//            ^                      /               \      /                                 __|/_____________>
//            |                      -----------------     /                                    |O             X
//            |                           Theta1          /
//            |                                          /_______RD_RE
//            |                                         /
//            |                                        /
//            |                                       /
//            |                                 / \  /
//            |                      RD_E_____ /   \/
//     RD_B   |                               /  *  \     RD_U
//            |                               -------    /
//            |                                  |      /
//            |                     RD_W________ |_____/_
//            |                                  |
//            |                                  | _______RD_V
//            |                                  ^
//____________|__________________________________________________________________________________________________________

// 此代码用于定义机械臂各个结构的参数及尺寸。

// #ifdef DELTA_X_PULY_S
// #define DEFAULT_RD_F	259.04   // SB			//size S  258.64 + 0.4
// #define DEFAULT_RD_E	120.0	// SP = RD_E / 2
// #define DEFAULT_RD_RF	130.0	// L
// #define DEFAULT_RD_RE	315.0   // l

// new delta
#ifdef DELTA_X_PULY_S
#define DEFAULT_RD_F	259.81   // SB			//size S  258.64 + 0.4
#define DEFAULT_RD_E	120.0	// SP = RD_E / 2
#define DEFAULT_RD_RF	120.0	// L
#define DEFAULT_RD_RE	306.0   // l

#define DEFAULT_RD_W	0.0
#define DEFAULT_RD_U	0.0
#define DEFAULT_RD_V	0.0

#define DEFAULT_MOVING_AREA_X		170
#define DEFAULT_MOVING_AREA_Y		170
#define DEFAULT_MOVING_AREA_Z	    300    // confined max z axis

#define DEFAULT_MOVING_AREA_LARGEST_DIAMETER	170			//mm

#define THETA1_HOME_POSITION -38.5      //theta_1原点角度			//deg -41.2	
#define THETA2_HOME_POSITION -38.5      //theta_2原点角度
#define THETA3_HOME_POSITION -38.5      //theta_3原点角度

// 200 * 8 * 5 = 8000，脉冲200一圈，细分8，传动比5，精度低
// 200 * 16 * 5 = 16000，脉冲200一圈，细分16，传动比5，精度高

#define THETA1_STEPS_PER_2PI  8000		// 这个值感觉像是步进电机的初始脉冲	// 

// 200*16*3，16细分，200个脉冲，传动比3，9600个脉冲，电机转一周。
#define THETA2_STEPS_PER_2PI  8000
#define THETA3_STEPS_PER_2PI  8000

// #define THETA1_STEPS_PER_2PI  9600		// 这个值感觉像是步进电机的初始脉冲	//200*16*3，16细分，200个脉冲，传动比3，9600个脉冲，电机转一周。
// #define THETA2_STEPS_PER_2PI  9600
// #define THETA3_STEPS_PER_2PI  9600

#endif // DELTA_X_PULY_S

#ifdef DELTA_X_V2
#define DEFAULT_RD_F	260.0   // SB			//size S  258.64 + 0.4
#define DEFAULT_RD_E	120.0	// SP = RD_E / 2
#define DEFAULT_RD_RF	120.0	// L
#define DEFAULT_RD_RE	305.0   // l

#define DEFAULT_RD_W	0.0
#define DEFAULT_RD_U	0.0
#define DEFAULT_RD_V	0.0

#define DEFAULT_MOVING_AREA_X		170
#define DEFAULT_MOVING_AREA_Y		170
#define DEFAULT_MOVING_AREA_Z	    402

#define DEFAULT_MOVING_AREA_LARGEST_DIAMETER	170			//mm

#define THETA1_HOME_POSITION -34.25				//deg -41.2	
#define THETA2_HOME_POSITION -34.25
#define THETA3_HOME_POSITION -34.25

#define THETA1_STEPS_PER_2PI  9600				//200*16*3
#define THETA2_STEPS_PER_2PI  9600
#define THETA3_STEPS_PER_2PI  9600

#endif
