# nrf_ble_remote_control
Using nRF52 series SoC to develop a BLE remote control by a custom GATT service other than HOGP

# change log
2021-07-26
1. 初步实现Client和Server的自动连接

2021-07-27 午
1. 初步调通按键

2021-07-27 晚
1. 调好按键和使能notification, 而且只有完成绑定的peer才能写CCCD

2021-07-28 下午
1. Server 增加电量检测
2. Client 增加UART

2021-07-29 上午
1. 遥控器增加adv idle时进入 power down 模式，按键唤醒(会reset)
2. Notify 命令特征值时加限定，防止连接未绑定的设备时notify引起SoftDevice错误
3. 增加长按Mode键擦除 BLE Bonds 信息的功能

2021-07-29 晚
1. Client 也增加擦除 BLE Bonds 信息的函数，并且改进了Server 侧擦除BLE Bonds的函数，在连接状态也可以擦除了
2. 按键唤醒会记录按键的事件类型(button release)，如果白名单中有peer，在连接到peer后会notify对应命令(延时未测)

2021-07-30 晚
1. 修复连接未绑定情况下发送命令是 Server SoftDevice ERROR 的问题
