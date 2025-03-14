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

// 执行G代码函数
#include "GCodeExecute.h"

void GCodeExecuteClass::Init(vector<String>* gCodeQueue)  // 初始化 G代码命令
{
	this->GCodeQueue = gCodeQueue;  // 将 gCodeQueue 的值 赋给 GCodeQueue

	ResetValue();  // 重置 Value 的值
 
	IsRunning = false;
}

void GCodeExecuteClass::Run()  // 开始执行 G 代码
{
	WhenFinishMove(); // 判断机器是否在移动

	if (GCodeQueue->size() == 0 || IsRunning == true)  // 判断 GCodeQueue 是否有指令
		return;

	String GcodeInProcessing = GCodeQueue->operator[](0);   // 取最上面的代码给  GcodeInProcessing 这个变量
	// String 将字符串作为一种数据类型的表示方法

	vector<KeyValue> keyValues = getKeyValues(GcodeInProcessing);   // GcodeInProcessing 经过 getKeyValues 函数处理得到 keyValues ； 分离key 和 values
	// getKeyValues函数 得到 keyValues 的值。

	for (uint8_t i = 1; i < keyValues.size(); i++)    // 从 1 开始，直接读取 x y z …… 的数据
	// 读取 key 关键字（G 或者 M）中 x y z A F …… 的值。
	{
		switch (keyValues[i].Key)
		{
		case 'X':
			// Serial.println("X = keyValues[i].Value");
			// Serial.println(X);
			X = keyValues[i].Value;
			break;
		case 'Y':
			Y = keyValues[i].Value;
			break;
		case 'Z':
			Z = keyValues[i].Value;
			break;
		case 'E':
			E = keyValues[i].Value;
			break;
		case 'S':
			S = keyValues[i].Value;
			break;
		case 'A':
			A = keyValues[i].Value;
			break;
		case 'I':
			I = keyValues[i].Value;
			break;
		case 'J':
			J = keyValues[i].Value;
			break;
		case 'F':
			F = keyValues[i].Value;
			break;
		case 'P':
			P = keyValues[i].Value;
			break;
		case 'R':
			R = keyValues[i].Value;
			break;
		case 'Q':
			Q = keyValues[i].Value;
			break;
		case 'W':
			W = keyValues[i].Value;
			break;
		default:
			break;
		}
	}

	// 获取前面 G 代码中的值，赋给 checkAndRunFunction

	checkAndRunFunction(keyValues[0]);    //  keyValues[0] 是获取 第一组数据；    G代码 的第一个 字母
	// 这行代码就是在 执行 G 代码命令了

	GCodeQueue->erase(GCodeQueue->begin());   // erase means removes the element at the position pointed to by the iterator.
	IsRunning = true;
}

void GCodeExecuteClass::WhenFinishMove()   //  判断是否执行到位的函数  打印 执行完成后的结果
{
	if (Stepper.GetStateMotor() == true)
	{
		if (IsRunning == true)
		{
			if (Data.IsExecutedGcode == true)   // 判断 IsExecutedGcode 的值
			{
				SERIAL_PORT.println("Ok");	
			}
			else
			{
				if (Tool.IsWait == true)
				{
					if (!Tool.Wait())
					{
						return;
					}
					SERIAL_PORT.println("Ok");
					Tool.IsWait = false;
				}
				else
				{
					SERIAL_PORT.println("Unknown");
				}				
			}
			ConnectionState.ResetTimeDisconnect();
			IsRunning = false;
			Data.IsExecutedGcode = false;
		}
	}
}

void GCodeExecuteClass::checkAndRunFunction(KeyValue keyValue)   //  执行 G 代码函数
{
	switch (keyValue.Key)  // 等待 Key 的值 G 01 中的 G
	{
	case 'G':   // G 代码
		switch ((int)keyValue.Value)  // 等待 Value 的值 G 01 中的 1
		{
		case 0:
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;    // 这里相当于直接赋值给 CurrentPoint 当前点
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (Z == NULL_NUMBER) Z = Data.CurrentPoint.Z;
			if (W == NULL_NUMBER) W = Data.WPosition;
			
			Motion.G0(X, Y, Z, W);		// 这里就已经拿到了   X, Y, Z, W …… 的值，使用motion函数去执行指令
			break;
		case 1:
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (Z == NULL_NUMBER) Z = Data.CurrentPoint.Z;
			if (W == NULL_NUMBER) W = Data.WPosition;

			Motion.G1(X, Y, Z, W);

			if (S >= 0 && S <= 255) Control.M03(S);
			break;
		case 2:
			if (I == NULL_NUMBER) break;
			if (J == NULL_NUMBER) break;
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (W == NULL_NUMBER) W = Data.WPosition;

			Motion.G2(I, J, X, Y, W);

			if (S >= 0 && S <= 255) Control.M03(S);
			break;
		case 3:
			if (I == NULL_NUMBER) break;
			if (J == NULL_NUMBER) break;
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (W == NULL_NUMBER) W = Data.WPosition;

			Motion.G3(I, J, X, Y, W);

			if (S >= 0 && S <= 255) Control.M03(S);
			break;
		case 4:
			if (P == NULL_NUMBER) break;
			Motion.G4(P);
			break;
		case 5:
			if (I == NULL_NUMBER) break;
			if (J == NULL_NUMBER) break;
			if (P == NULL_NUMBER) break;
			if (Q == NULL_NUMBER) break;
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (W == NULL_NUMBER) W = Data.WPosition;

			Motion.G5(I, J, P, Q, X, Y, W);

			if (S >= 0 && S <= 255) Control.M03(S);
			break;
		case 6:
			if (F != NULL_NUMBER) Planner.SetVelocity(F);
			if (X == NULL_NUMBER) X = Data.CurrentPoint.X;
			if (Y == NULL_NUMBER) Y = Data.CurrentPoint.Y;
			if (Z == NULL_NUMBER) Z = Data.CurrentPoint.Z;
			if (W == NULL_NUMBER) W = Data.WPosition;
			if (P == NULL_NUMBER) break;

			Motion.G6(X, Y, Z, P);

			if (S >= 0 && S <= 255) Control.M03(S);
			break;
		case 28:
			Motion.G28();
			break;
		case 90:
			Control.G90();
			break;
		case 91:
			Control.G91();
			break;
		case 93:
			Control.G93();
			break;
		default:
			break;
		}
		break;
	case 'M':     // M 代码
		switch ((int)keyValue.Value)
		{
		case 3:
			if (S == NULL_NUMBER) S = 255;
			Control.M03(S);
			break;
		case 4:
			if (S == NULL_NUMBER) S = 255;
			Control.M04(S);
			break;
		case 5:
			Control.M05();
			break;
		case 84:
			Control.M84();
			break;
		case 104:
			if (S > 240) S = 240;
			Control.M104(S);
			break;
		case  105:
			Control.M105();
			break;
		case 109:
			Control.M109();
			break;
		case 203:
			Control.M203(S);
			break;
		case 204:
			Control.M204(A);
			break;
		case 206:
			Control.M206(Z);
			break;
		case 360:
			if (E > USE_CUSTOM) break;
			Control.M360(E);
			break;
		case 361:
			if (P == NULL_NUMBER) break;
			Control.M361(P);
			break;
		case 362:
			if (P == NULL_NUMBER) break;
			Control.M362(P);
			break;
		case 500:
			Control.M500();
			break;
		case 501:
			Control.M501();
			break;
		case 502:
			Control.M502();
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	ResetValue();
}

// G01 X80 Y50 Z-200  //此函数以空格为一段，分别处理每一段的数据到 Key 和 Value
vector<KeyValue> GCodeExecuteClass::getKeyValues(String code)   // 得到 关键字 和 值
{
	String splitWord = "";
	vector<KeyValue> keyValues;
	code += " ";   // code = code + " "
	for (uint16_t i = 0; i < code.length(); i++)
	{
		if (code[i] == ' ')
		{
			if (splitWord == "")
				continue;
			KeyValue keyValue;

			keyValue.Key = splitWord[0];    // 获取 关键字 

			keyValue.Value = splitWord.substring(1).toFloat();  // 获取 G 代码中的数值

			keyValues.push_back(keyValue);   // 将 keyValu 值，push到keyValu中去
			
			splitWord = "";
			continue;
		}
		splitWord += String(code[i]);
	}
	return keyValues;
}

void GCodeExecuteClass::ResetValue()  // 重置 Valude 的值。
{
	X = NULL_NUMBER;   // x 轴
	Y = NULL_NUMBER;   // y 轴
	Z = NULL_NUMBER;   // z 轴

	E = NULL_NUMBER;   // X 轴

	S = NULL_NUMBER;   // 主轴转速
	A = NULL_NUMBER;   // 加速度

	I = NULL_NUMBER;   // x轴偏移量（第一个点）
	J = NULL_NUMBER;   // y轴偏移量（第一个点）

	F = NULL_NUMBER;   // 速度

	P = NULL_NUMBER;   // x轴偏移量（第二个点）

	R = NULL_NUMBER;   

	Q = NULL_NUMBER;   // y轴偏移量（第二个点）

	W = NULL_NUMBER;   // X 轴
}