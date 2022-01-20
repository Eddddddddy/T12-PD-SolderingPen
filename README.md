# T12-PD-SolderingPen

使用atmega328p作为主控芯片，128x64 OLED屏幕，内置陀螺仪和蜂鸣器。能够读取控制部分电路温度和输入电压，主控电压，主控温度。具备自动启停的功能。

![image](3.images/9CA4EE73-B2C1-42C1-BB1C-226CC02446D5.png)

可以保存多组不同的烙铁头信息。烙铁主控部分代码基于<https://github.com/wagiminator/ATmega-Soldering-Station>.在屏幕驱动、按键驱动、休眠检测、陀螺仪和内部温度检测等方面做出调整与改进。

## 制作信息

- 手柄部分请使用1.6mm双层板打样，主控部分请使用0.8mm四层板。

- 屏幕较为特殊，为0.96寸64x128分辨率sh1107驱动OLED屏幕，0.3mm脚距插接13pin接口。

- FS312协议芯片请使用H版，以支持20v电压。

- 建议开一张钢片用锡膏焊接，大大降低难度。

- 焊接完成之后需要飞线到预留的焊盘，进行程序烧录。

## 注意事项

- 支架与主控需共地
- 如果主控不是原厂的芯片，在information页面的主控温度可能会不正常。但不影响使用
- 建议通过<https://github.com/carlosefr/atmega>页面的步骤对芯片进行烧写。
- 简单来说，安装上面连接中的开发板到Arduino IDE，并在开发板中选中"ATmega328/328p" - "ATmega328p" - "16 MHz" - 编程器："Arduino as ISP"
- 准备一个Arduino UNO，烧写Arduino示例程序中的ArduinoISP。将Arduino UNO的相应管脚与主控板上相应的飞线焊盘连接。在Arduino IDE中点击“烧录引导程序”，完成之后再点击“使用编程器上传”，即可完成烧录工作。

更多信息稍后更新。
