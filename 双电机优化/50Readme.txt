update:2021.8.18 H3AA30-C1版本
40机器调试完成
修复：
1.无刷电机使能报HALL错误，位CW和CCW值为0
新增：
1.推杆电机上电标定
2.推杆电机位置控制
3.新增对象字典 （全局错误、当前错误、推杆给定位置、推杆实际位置、推杆标定标志位、位置环使能标志）

updata:2021.8.20
新增状态指示灯
1.错误报警同时显示两个错误只显示最后一个发生的错误

updata:2021.8.28
1.禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
2.将主函数的PID计算移到嘀嗒定时器
3.将霍尔两个霍尔学习分开，Motorstate 8 五号电机 9 六号电机
4.新增堵转保护（未完善）
5.ADC模拟看门狗和看门狗1S喂狗（模拟看门狗当中未处理，由于温度读取上拉信号未ADC采样最大值阈值没法设置）

updata:2021.9.2
修复BUG：1.断线保护功能失效导致代码跑飞（优先级问题）
          2.CAN通信异常终止（旧版本的CAN库存在的BUG）
          3.更改为外部时钟
          4.将三路ADC的温度采样改为与采上一个通道（并打开模拟看门狗）

updata:2021.9.13
1.新增未接霍尔报错
2.打开堵转功能
3.新增四个推杆上电收回标定（标定在INIT状态机当中执行），以及4个推杆位置控制
4.优化过压欠压错误报警功能（门限值低于电压限制值2V在计数值开始--）
5.将主函数所有的具有时间效应的功能都移到嘀嗒定时器当中（包括获取速度、状态机、过压保护）


updata: 2021 .9.15
1.新增两个无刷BRAKE功能（brake报错指示灯未做）
2.新增过流保护四个阶段
3.新增电容充电
4.将速度环计算加入MotorSpeedSet case当中
5.解决使能电机5边刷开启的BUG为换向CCER值问题
updata:9.22
1.修复误触发brake中断BUG
2.新增BLDC电容充电，打开下管
H4AA30-C1_S线_ 该版本与9.15版本的差异为使能电机5边刷动或者不动，9.13版本为S线车上的代码，该版本为9-13版本升级版，需要上车测试

updata:10.10  H3AA30-C2版本
1.更改与C1版本差异的引脚
2.新增4路推杆和4单向电机硬件过流保护
3.修改无刷霍尔外部中断优先级2——>1（解决两个位置的换向冲突问题）
4.修复无刷5、无刷6电机运行过程中的电流尖峰（霍尔中断当中新增判断条件MotorControl[6].Direction == 1 || MotorControl[6].Direction == -1，在2msPID运算周期换向前关闭外部中断）
存在问题：无刷电机电流干扰问题

updata:10.11
1.CANopen新增值更改主动报错功能
2.TPDO1为触发方式为01，收到单个同步帧响应数据

updata:10.18
1.修改OVR和UVR
2.新增硬件过流指示灯

updata:10.22
1.修改推杆霍尔外部中断 else当中pwmduty为0时受到干扰霍尔值自增导致推杆不能到位置，PWM值一直存在

uapdata:10.30
1.修改推杆加速过程从PWM1000开始加速
2.推杆停机充电
3.缩短推杆的反方向停机时间，以及CCW\CW\STOP的宏定义去掉其中的for延迟
4.打开推杆BRAKE
5.（40机器修改边刷的选装方向）

updata:11.4
1.修改风机上电报brake问题

update:11.5			author ：diamond
1.修改24V上电延时100ms->500ms以保证风机上电不报brake

update:11.8
1.增加风机故障报警，根据报警信息不同报警灯闪烁6次

update:11.15
1.S线————修改电机1（吸水趴推杆）方向

updata:11.22
1.修复硬件短路报警的bug
	#1检测到电机3，4在报警后的PWM仍然会有低电平跳变，原因是定时器的ccr需要和8400做比较（大于8400）。
	因此需要定义常量
	PWM_PeriodOFFSET以保证输出恒为高电平(目前电机3，4，7，8，10，11均存在此问题，已修正)
	#2在频率较低的情况下，通过修改PWM来调整定时器输出的方法存在延时，
	例：50hz频率，需要进3-10次硬件中断才能调整PWM为高电平。
	因此改为修改TIMx->EGR=1的方法来调整定时器输出以达到最快的速度。
	
update:11.24
1.增加上位机相关文件Agreement.c和flash.c
startup_stm32f407xx.s的line33和line44改为0x1000否则会造成栈溢出，程序跑飞
2.增加过流最大次数限制，当累计过流超过BREAK_CNT_MAX后，相关电机禁止使能。
	且报警灯一直闪烁，直到软件清除报警次数
3.增加canRPDO，可以通过can进行过流最大次数的读取和清零，以及flash的读写
增加历史故障记录，可记录最近10次产生的报警，可通过SDO读取错误报警码以及错误报警码产生时的嘀嗒
4.看门狗5s

update:11.30
1.增加无刷三相检测，电机相线接错后，电机不动或来回振动会产生报警码MOTOR5_PHASE_ERROR和MOTOR6_PHASE_ERROR

upadata:12.1
1.新增推杆霍尔保护和断线保护 ，4路推杆短线保护强制打开，霍尔保护选择打开（wGlobal_Flags只提示推杆错误并且报警（霍尔或者断线），
  具体哪个推杆电机错误需要读取wGlobal_Flags1，并且wGlobal_Flags1不会自动TPDO发送）
2.新增无刷缺相保护    （wGlobal_Flags只提示5号无刷缺相或者6号无刷缺相，具体的错误需要读取wGlobal_Flags1，wGlobal_Flags1不会自动TPDO发送）
3.新增三四号电机断线保护 （wGlobal_Flags中提示三号四号电机错误）
4.修改错误报警指示灯，新增第二个32位全局错误，不会自动上传
5.新增电压偏置上电修正

update :12.07
1.修改LED灯的函数，现在支持慢闪，快闪N次，之前只支持慢闪1次

update:12.9	
1.修正了无刷电机使能后，速度为0时会抖动的bug 将定时器1和8的周期改为8400.不能将EGR置1会重置定时器计数

update:12.13
1.无刷霍尔错误以后现在不能使能，现在PID计算处的PWM最大由8400改为8000。
2.增加电流环，电流环和速度环的数组定义修改，初始化中的PID初始化放在无刷电流初始化后面
3. S线的3档保护电流值及时间改为Cur3A 5A 7A  time3S,1S,200ms
4.推杆电机0的位置现在可以根据电机5的电流调整高低。
5.无刷相序错误来回摆动的情况做了错误累计，10次后才会报警，防止恶劣工况下误报
6.V2.1的板子无刷采集5.6有区别，代码内5号电机约100代表1A，6号电机约137代表1A
7.修改tpdo对象0x3003的内容，现在8个内容，分别是设定的电流值以及3段报警值

update :12.16
1.新增边刷异常报警，累计2秒无动作后报警		错误码未定
2.去掉推杆位置环做高度自适应，用三段式速度很稳定，PID改了效果不够好，还在测试
3.BLDC失速报警		4S连续速度异常后报警

update:12.21
1、新增变量MotorControl[5].Current.CurOffset，表示上位机下发的电流误差范围，对象字典为0x3003
2、将推杆的位置环加了一个条件，即设置自适应电流为0时才启用位置环
3、推杆自适应通过PWM来调控，而不是位置
4、给6号无刷的深度滤波加了个参数1.37，现在深度滤波表示的实际电流均为深度滤波除以100，例如100代表1A
5、推杆自适应PID版本测试ok
6、新版错误报警灯OK

update：12.22
1、更新了对象字典

update:12.27
1、推杆自适应只要给限制电流推杆就动作，不需要滚刷也转起来
2、推杆行程最大限制在36，

update：1.7
1、修改了失速报警的条件，去掉了一个会导致误报的条件
2、推杆自适应最小高度限制在5，低于5不往回调整
3、增加了代码版本的sdo，对象字典0x3000，子索引02


【50版本开始】
updata:1.28
1.BLDC5_Phase_Check修改该函数永不执行的BUG
2.重新调整缺相保护功能
3.屏蔽第一阶段自动调整PWM

updata:2.11
1.调整缺相保护功能，改为先排序再用最大值减去最小值再除以最大值，得出0.5该值符合高速中速低速情况下的50机器和S线机器，该功能稳定完成
2.调整无刷电机的电流保护为7A8A9A10A
3.调整推杆标定功能，修改参数初始化标定hall给定值，新增对象字典3017：分别控制4路推杆电机的标定，该指令由下位机发送，推杆标定按照全部收回或者
中途堵转的情况做为0位（该需求为50提出不合理，由于结构可能存在不能完全收回去的情况）
4.调整BLDC_Stuck_Chk该函数，区分电流情况，有电流为相序错误（堵转），没有电流将错误归类为缺相
5.调整新版电流采样系数，理论值为116，实际值为108和114
6.修复推杆电机进入启动文件.B当中，是由于移植过程中导致串口中断开启，但是没有中断函数的入口
7.去掉过流第一阶段的PWM自动调整
8.屏蔽FLASH保存标志位以及CurrentLimit函数当中的未完成堵转保护
9.修改34号电机的频率为10K(之前移植S线为50HZ),降低34号电机的过流保护值过滤点击 4567A，过滤电机3456A，延长其过流保护时间
10.调整代码运行指示灯闪烁频率
11.修改推杆位置控制函数，需要位置控制的需要先标定否则不能控制
12.修改推杆标定修改偏置电压的BUG
13.修改8号接口风机测速失效
14.修改深度滤波算法

upadata:2.18
1.修改滚刷推杆标定方向
2.新增推杆标定时，PWM减小的最小值限制 
3.新增推杆自适应，将电流系数定义为宏定义，并修改推杆自适应推杆运动方向
4.修改极对数2-->4

upadata:2.22
1.修改过流保护值和过流时间
2.修改高速切换到低速时失控问题，高速切换到低速时清掉积分项
3.修改BLDC控制方式PWM>4500时下管全开

updata:2.24
1.修改缺相保护判断值0.5->0.8 0.5为S线滚刷适用值，0.8适用于50机器
2.修改PID参数，速度环 700 256 2 1024  电流环 800 256 1 1024 
3.新增最低速度限制300RPM
4.修改34号电机的加减速度3->10
upadata:2.26
1.修改报告缺相的bug
2.修改限制最大速度为1000
3.新增对象字典3004深度滤波后的值
4.新增50机器版本
5.修改缺相保护判断值0.8->0.9，并且新增电流值大于50时，检测缺相

updata:3.1
1.修改自适应参数：调整范围0.3->0.5 推杆1加速度10->100 PID运算周期100ms ->10ms PID输出 -1000-1000之间都等于0
2.新增清除错误函数（先报错再置错误标志）

upadata:3.4
1.新增硬件版本号
2.修改硬件过流flash指针改为定义的变量
3.新增推杆超时保护
4.修改缺相报错检测时间

upadata:3.8
1.将10号电机11和12PWM开关量改为rampPWM方式（上机代码不含本次修改3.9）
