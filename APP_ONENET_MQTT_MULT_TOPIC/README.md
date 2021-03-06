# APP+STM32+ESP8266+MQTT协议上云OneNET

**简介：** STM32+ESP8266通过MQTT协议将多传感器数据传输至OnenNet云平台，加入操作系统FreeRTOS进行多任务管理，增删模块和功能简单方便，提高开发效率，可以根据自己的需求快速增加其他传感器模块。基于上个版本改进，加入APP进行远程控制以及查看相关数据。

> 1.**基于上一个freeRTOS版本改进**，利用了mqtt协议的轻量性以及topic（主题）发布与订阅的特性。（上个版本没有topic订阅的功能，只是OneNET官方给的数据流接口，无法通过OneNET转发至个人APP）**主要修改传感器数据上传报文的组织形式并加入订阅topic功能，设备控制部分简单修改**
>
> 3.**加入安卓手机APP**进行远程控制，不再使用OneNET自带的应用管理，**APP采用图形化编程，对只想快速制作APP的人来说非常友好（有APP制作简单教学）**
>
> 4.**两个版本**，一个版本是多topic版（稍微稳定），一个是单topic版本（随便看看）

**已知缺陷：**

1. **APP存在缺陷，有时候会收不到数据（云平台转发丢包占一定比例，程序应该也有问题，但具体问题暂时不清楚，仅供参考，欢迎各位指出问题）**

2. 偶尔出现esp8266连接不上网络的问题（包括路由器连接，STA设置），大概率接线问题，多复位几次；也可能已经连接上但是串口显示未连接，可以试试去掉wifi指令控制的返回值判定。

**注：部分功能采用他人开源程序或在他人开源程序的基础上修改。**