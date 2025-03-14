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

#include "Planner.h"

void PlannerClass::init(vector<Segment>* SegmentQueue)  //初始化
{
	StepsPerDeg[0] = (float)THETA1_STEPS_PER_2PI / 360.0;
	StepsPerDeg[1] = (float)THETA2_STEPS_PER_2PI / 360.0;
	StepsPerDeg[2] = (float)THETA3_STEPS_PER_2PI / 360.0;

	this->SegmentQueue = SegmentQueue;

	RecalculateBeginEndIntCycle();    // 重新计算开始和结束的intcycle
	RecalculateTimeForChangeVelocity();  // 重新计算 改变后速度的时间

	LastError[0] = 0;
	LastError[1] = 0;
	LastError[2] = 0;
}

void PlannerClass::AddHomeSegment()   // 添加原点插值
{
	float timeForOneStep = 1 / (Data.MovingHomeSpeed * StepsPerDeg[0]);

	HomingIntCycle = timeForOneStep * TIME_TO_US;

	Segment segbuffer;     // segbuffer 这个变量与Segment
	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer.StepperArray[i].StepsToJump = STEP_NULL;   // #define STEP_NULL 65530
		segbuffer.StepperArray[i].Direction = DECREASE;
	}
	SegmentQueue->push_back(segbuffer);

	Segment segbuffer2;
	float offset[3];
	offset[0] = 10;
	offset[1] = 10;
	offset[2] = 10;
	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer2.StepperArray[i] = ChangeToStep(offset, i);
	}
	SegmentQueue->push_back(segbuffer2);

	Segment segbuffer1;
	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer1.StepperArray[i].StepsToJump = STEP_NULL;   // #define STEP_NULL 65530
		segbuffer1.StepperArray[i].Direction = DECREASE;
	}
	SegmentQueue->push_back(segbuffer1);

	LastError[0] = 0;
	LastError[1] = 0;
	LastError[2] = 0;
}

void PlannerClass::AddBeginSegment(float* offset, float lengthOfRoad)  // 添加开始运动插值
{
	Segment segbuffer;

	NumberIntRoad = 0;

	float timeMove = lengthOfRoad / Data.Velocity;

	segbuffer.NumberINT = roundf(timeMove * TIME_TO_NUMINT);   // 取整  ；这里计算了 segbuffer.NumberINT 的值

	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer.StepperArray[i] = ChangeToStep(offset, i);  

		// 这里直接计算了segbuffer.StepperArray[i]的值，然后因为StepperArray是一个结构体，所以需要将里面的值赋值给StepsToJump，
		// 相当于在计算segbuffer.StepperArray[i].StepsToJump 的值。

		if (segbuffer.NumberINT < segbuffer.StepperArray[i].StepsToJump)
		{
			segbuffer.NumberINT = segbuffer.StepperArray[i].StepsToJump;
		}
		//Serial.println(segbuffer.StepperArray[i].StepsToJump);
	}

	NumberIntRoad += segbuffer.NumberINT;   // 然后将 segbuffer.NumberINT 这个值 赋给 NumberIntRoad

	SegmentQueue->push_back(segbuffer);
}

void PlannerClass::AddEndSegment(float* offset, float lengthOfRoad)  // 添加 结束运动插值
{
	Segment segbuffer;

	float timeMove = lengthOfRoad / Data.Velocity;
	segbuffer.NumberINT = roundf(timeMove * TIME_TO_NUMINT);

	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer.StepperArray[i] = ChangeToStep(offset, i);
		if (segbuffer.NumberINT < segbuffer.StepperArray[i].StepsToJump)
		{
			segbuffer.NumberINT = segbuffer.StepperArray[i].StepsToJump;
		}
		//Serial.println(segbuffer.StepperArray[i].StepsToJump);
	}

	NumberIntRoad += segbuffer.NumberINT;

	if (NumberIntRoad / 2 < TimeForChangeVelocity)
	{
		AccelerationUntil = NumberIntRoad / 2;
		DecelerateAfter = AccelerationUntil;
	}
	else
	{
		AccelerationUntil = TimeForChangeVelocity;
		DecelerateAfter = NumberIntRoad - AccelerationUntil;
	}

	SegmentQueue->push_back(segbuffer);
}

void PlannerClass::AddChangeSegment(float* offset, float lengthOfRoad)   // 添加变速插值
{
	Segment segbuffer;
	NumberIntRoad = 0;

	float timeMove = lengthOfRoad / Data.Velocity;
	segbuffer.NumberINT = roundf(timeMove * TIME_TO_NUMINT);

	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer.StepperArray[i] = ChangeToStep(offset, i);
		if (segbuffer.NumberINT < segbuffer.StepperArray[i].StepsToJump)
		{
			segbuffer.NumberINT = segbuffer.StepperArray[i].StepsToJump;
		}
	}

	NumberIntRoad += segbuffer.NumberINT;

	if (NumberIntRoad / 2 < TimeForChangeVelocity)
	{
		AccelerationUntil = NumberIntRoad / 2;
		DecelerateAfter = AccelerationUntil;
	}
	else
	{
		AccelerationUntil = TimeForChangeVelocity;
		DecelerateAfter = NumberIntRoad - AccelerationUntil;
	}

	SegmentQueue->push_back(segbuffer);  // 插入到 全局队列 SegmentQueue
}

void PlannerClass::AddFixedSegment(float* offset, float lengthOfRoad)   // 添加固定Fixed插值
{
	Segment segbuffer;

	float timeMove = lengthOfRoad / Data.Velocity;
	segbuffer.NumberINT = roundf(timeMove * TIME_TO_NUMINT);

	for (uint8_t i = 0; i < 3; i++)
	{
		segbuffer.StepperArray[i] = ChangeToStep(offset, i);
		if (segbuffer.NumberINT < segbuffer.StepperArray[i].StepsToJump)
		{
			segbuffer.NumberINT = segbuffer.StepperArray[i].StepsToJump;
		}
		//Serial.println(segbuffer.StepperArray[i].StepsToJump);
	}

	NumberIntRoad += segbuffer.NumberINT;

	SegmentQueue->push_back(segbuffer);
}




void PlannerClass::AddExtrustionSegment(float offset)   // 添加挤出运动插值
{
	if (offset == 0)
	{
		return;
	}

	if (offset > 0)          // 这里就是在设置 挤出步进电机的偏移量
	{
		ExtrustionStepsDirection = INCREASE;   // 这里使用的枚举 
	}
	else
	{
		ExtrustionStepsDirection = DECREASE;   /// DECREASE = 0
		offset = -offset;
	}

	ExtrustionStepsToJump = roundf(offset * EXTRUSTION_STEPS_PER_MM);   // 取整

	if (NumberIntRoad == 0)
	{
		float timeMove = offset / Data.Velocity;

		NumberIntRoad = roundf(timeMove * TIME_TO_NUMINT);   // 向下取整

		Segment segbuffer;
		segbuffer.NumberINT = NumberIntRoad;

		segbuffer.StepperArray[0].StepsToJump = 0;
		segbuffer.StepperArray[1].StepsToJump = 0;
		segbuffer.StepperArray[2].StepsToJump = 0;

		AccelerationUntil = 0;
		DecelerateAfter = NumberIntRoad;

		SegmentQueue->push_back(segbuffer);   // 传入 SegmentQueue 这个队列
	}
}

Step PlannerClass::ChangeToStep(float* offset, uint8_t index)  // change to step
{
	Step stepBuffer;

	float realStepsToJump = offset[index] * StepsPerDeg[index];   // 计算真实的 移动步数

	realStepsToJump -= LastError[index];

	if (offset[index] >= 0)
	{
		stepBuffer.Direction = INCREASE;
		realStepsToJump = realStepsToJump;
	}
	else
	{
		stepBuffer.Direction = DECREASE;
		realStepsToJump = -realStepsToJump;
	}

	stepBuffer.StepsToJump = roundf(realStepsToJump);   // 向下取整

	LastError[index] = stepBuffer.StepsToJump - realStepsToJump;
	
	if (offset[index] < 0)
	{
		LastError[index] = -LastError[index];
	}

	return stepBuffer;  // 返回stepBuffer
}

void PlannerClass::SetVelocity(float velocity)  // 设置速度
{
	if (velocity > DEFAULT_MAX_VELOCITY)  // 判断，if 设置的速度 小于 默认速度则还是默认速度
	{
		Data.Velocity = DEFAULT_MAX_VELOCITY;
	}
	else
	{
		Data.Velocity = velocity;     // 否则的话 就重新赋值为 Velocity 速度
	}

	if (Data.End_Effector == USE_PRINTER)
	{
		Data.Velocity = velocity / 60;
	}

	RecalculateBeginEndIntCycle();
	RecalculateTimeForChangeVelocity();
}

void PlannerClass::SetMaxVelocity(float velocity)  // 设置最大速度
{

}

void PlannerClass::SetAcceleration(float acceleration)  // 设置加速度
{
	if (acceleration > DEFAULT_MAX_ACCELERATION)
	{
		Data.Acceleration = DEFAULT_MAX_ACCELERATION;
	}
	else
	{
		Data.Acceleration = acceleration;
	}

	RecalculateTimeForChangeVelocity();
}

void PlannerClass::SetHomeSpeed(float velocity)  // 设置回原点速度
{
	Data.MovingHomeSpeed = velocity;
	float timeForOneStep = 1 / (Data.MovingHomeSpeed * StepsPerDeg[0]);
	HomingIntCycle = timeForOneStep * TIME_TO_US;
}

void PlannerClass::SetBeginEndVelocity(float velocity)   // 设置开始和结束速度
{
	Data.BeginEndVelocity = velocity;
	RecalculateBeginEndIntCycle();
	RecalculateTimeForChangeVelocity();
}

void PlannerClass::RecalculateTimeForChangeVelocity()   // 计算时间，变速的时间
{
	float accelTime = (Data.Velocity - Data.BeginEndVelocity) / Data.Acceleration;
	TimeForChangeVelocity = roundf(accelTime * TIME_TO_NUMINT);
	a = (BeginEndIntCycle - INTERRUPT_CYCLE_MIN) / (accelTime * TIME_TO_US);
	if (a < 0)
	{
		a = 0;
	}
}

void PlannerClass::RecalculateBeginEndIntCycle()   // 再次计算开始和结束的intcycle
{
	BeginEndIntCycle =  Data.Velocity * INTERRUPT_CYCLE_MIN / Data.BeginEndVelocity;
	
	if (BeginEndIntCycle < INTERRUPT_CYCLE_MIN)
	{
		BeginEndIntCycle = INTERRUPT_CYCLE_MIN;
	}
}

PlannerClass Planner;