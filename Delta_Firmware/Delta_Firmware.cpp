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

#include "config.h"
#include "Constants.h"

#include "fastio.h"

#include "Storage.h"
#include <ArduinoSTL.h>
#include <vector>
#include "GCodeReceiver.h"
#include "GCodeExecute.h"
#include "Planner.h"
#include "Motion.h"
#include "Tool.h"
#include "EndStops.h"
#include "EndEffector.h"
#include "MultiServo.h"
#include "Temperature.h"
#include "ConnectionState.h"

using namespace std;

GCodeReceiverClass GcodeReceiver;  // 接收G代码
GCodeExecuteClass GcodeExecute;   // 执行G代码命令

vector<String> GCodeQueue;  //全局变量，接收G代码，每一行代码放到这个队列里面，依次压入队列（G代码队列）
vector<Segment> SegmentQueue; // 全局变量，插值队列？

// 这个相当于 C++ 中的main 函数

void setup() { // only once

	SERIAL_PORT.begin(BAUDRATE); // connect the board using  baudrate // 波特率值

	Data.init();   //  这句话感觉就像是对数据(机械臂结构的数据)进行初始化

	Storage.init(); // 这句话感觉还是在初始化，这个时候开始涉及到板子的信息，比如EEPROM.put()，EEPROM.get()写数据和读数据

	DeltaKinematics.init();  // 正解逆解初始化？

	EndEffector.init();  // 给对应引脚写入 PWM 脉冲

	EndStops.init();  // 初始化机械臂限位装置处的状态

	Planner.init(&SegmentQueue);   // 这个 函数主要用来设置 速度 等

	Stepper.init(&SegmentQueue);  //  这个 函数主要用来 启动步进电机

	MultiServo.init();  //舵机的初始化程序？

	ConnectionState.Init();  // 这是在初始化连接状态

	Motion.init();  // 初始化机械臂运动函数

	Temperature.init();  //初始化加=热床

	GcodeReceiver.Init(&GCodeQueue, &SERIAL_PORT, BAUDRATE); //负责接收G代码
	GcodeExecute.Init(&GCodeQueue); //负责执行G代码

	Serial.println("Init Success!");  //打印初始化成功
	
	// 初始化完成后，执行G28指令，回到原点。

	// Motion.G28();
}

void loop() { // continuously repeated

	// ****************  很重要***************//
	
	GcodeReceiver.Execute();  // 执行 接收G代码指令
	GcodeExecute.Run();    // 执行 G 代码指令

	// ****************  很重要***************//


	ConnectionState.Execute();   // 显示连接状态

	Temperature.ISR_EXECUTE();  // 显示加热床温度
}
 