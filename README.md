# MatKit

Development Kit for Matrix Sensor. 阵列传感器开发工具集。



- 基本数据流：信号采集 → 信号处理 → 上层应用
  - 信号采集：硬件采集信号通过串口与计算机通信
    - Arduino固件
  - 信号处理：计算机服务端程序，基于datagram socket（UNIX domain 或 UDP）
    - 有C++版本和Python版本
  - 上层应用：计算机客户端程序
    - Python
- 其他工具



## Arduino代码

目前存在多种版本代码，针对不同（传感器-Arduino开发板-多路选择器）组合。

如需设置Arduino传递数据的波特率，则使用Arduino的IDE打开`ino`文件并修改`void setup()`函数中的这一行：

```c++
Serial.begin(500000);
```

修改之后烧写到Arduino板子上。



## C++代码：信号处理服务程序

可通过串口读取数据，或从文件读取数据；可输出数据到指定文件。可指定解析数据的方式，并提供数据服务通信。数据通信使用进程间通信（IPC）常用的UNIX domain datagram socket。

运行方式：

```
READ and PROCESS Arduino pressure sensor data.

Data flow:
  Default: connect to Arduino board using serial port.
  Alternative: read from && output to file.
               input: positional argument
               output: '-o' flag

Usage options:
  -h            print this help message and exit
  -e            enumerate serial ports and exit
  -r            raw data mode
                  default: false (processed)
  -s            run service on demand using UNIX domain datagram socket
                  default: false
  -p PORT       sepcify port name
                  default: last port i.e. /dev/cu.usbmodem1424301
  -b BAUDRATE   specify baudrate
                  default: 500000
  -t TIMEOUT    specify timeout in milliseconds
                  default: 1000
  -n SIZE       specify side size of sensor
                  default: 16
  -u SOCK_FILE  specify UNIX domain socket address
                  default: /var/tmp/unix.socket.server
  -o OUT_FILE   specify output filename
                  default: output.csv

Options for processed data mode (i.e. ignored when raw data mode):
  -i INIT_NUM   specify number of frames for initiating calibration
                  0 for no calibration
                  default: 100, value(integer) >= 0
  -w WINDOW     specify window size for dynamic calibration,
                  0 for static calibration
                  default: 10000, 0 <= value(integer) <= 10000
  -fs S_FILTER  specify spatial filter window type to smooth data,
                  0: None
                  1: Ideal
                  2: Butterworth
                  3: Gaussian
                  default: 3
  --d D0        specify spatial filter window radius D0
                  default: 3.500000, value(float) > 0

  for Butterworth:
  --o ORDER     specify order of butterworth window
                  default: 2, value(int) >= 1

  -ft T_FILTER  specify temporal filter to smooth data,
                  0: None
                  1: Exponential Smoothing
                  2: Moving Average
                  3: SINC Low-Pass filter
                  default: 3

  for Exponential Smoothing:
  --a ALPHA     specify ALPHA for first-order exponential smoothing
                  default: 0.110000, 0 <= value(float) <= 1
  --b BETA      specify BETA for second-order exponential smoothing
                  default: 0.000000, 0 <= value(float) <= 1

  for Moving Average:
  --m MA_SIZE   specify kernel size for moving average
                  default: 15, 0 < value(integer) <= 100

  for SINC Low-Pass filter:
  --ls LP_SIZE  specify kernel size for sinc low-pass filter
                  default: 16, 0 < value(integer) <= 100
  --lw LP_W      specify cut-off normalized frequency
                  default: 0.040000, value(float) > 0

Positional argument:
  INPUT_FILE    (optional) specify input filename(s), reading each line
                  as a raw frame and output processed data to OUT_FILE;
                  multiple input files will be concatenated.
                  NOTE: this argument will disable serial port reading
```

UNIX domain socket通信地址：`/var/tmp/unix.socket.server`

通信API：

| 简称     | 请求内容   | 值            | 请求格式         | 返回内容             | 返回格式         |
| -------- | ---------- | ------------- | ---------------- | -------------------- | ---------------- |
| CLOSE    | 关闭服务   | 0             | 1byte            | 操作状态             | 1byte            |
| DATA     | 获取处理帧 | 1             | 1byte            | 帧数据+帧号          | 256double+1int   |
| RAW      | 获取原始帧 | 2             | 1byte            | 帧数据+帧号          | 256double+1int   |
| REC_DATA | 存储处理帧 | 3(+filename)  | 1byte+string     | 操作状态+文件名      | 1byte+string     |
| REC_RAW  | 存储原始帧 | 4(+filename)  | 1byte+string     | 操作状态+文件名      | 1byte+string     |
| REC_STOP | 停止存储   | 5             | 1byte            | 操作状态             | 1byte            |
| RESTART  | 重启服务   | 6+i+w+f(+...) | 1byte+3int(+...) | 操作状态+i+w+f(+...) | 1byte+3int(+...) |
| PARAS    | 获取参数   | 7             | 1byte            | 操作状态+i+w+f(+...) | 1byte+3int(+...) |

- 操作状态为1byte，0表示成功，255表示失败
- filename文件名为空时，默认输出到可执行文件路径下的`output.csv`中
- 重启服务各参数，-1表示参数保持不变：
  - i（int）：指initiating frame number，用于校准的帧数，应非负，0表示无校准
  - w（int）：指calibration window size，动态校准窗口大小，应介于0和最大窗口之间，0表示校准零点不变（静态校准）
  - f（int）：指filter，使用何种滤波器，可选值有
    - 0：无滤波
    - 1：二阶指数平滑
    - 2：移动平均
    - 3：sinc低通滤波
    - 如果传入-1，则维持原有滤波器及参数，下面的其他选项将被忽略
  - 其他：根据f指定的滤波器的不同，其他参数格式也不同，见下；如果指定了滤波器但未指定其他参数，则使用该指定滤波器的前一次参数
    - 当f = 1时，二次指数平滑：
      - α（double）：一次平滑参数，介于0和1之间，1表示无平滑
      - β（double）：二次平滑参数，介于0和1之间，0表示仅为一次指数平滑
    - 当f = 2时，移动平均：
      - m（int）：窗口大小，应大于0
    - 当f = 3时，sinc低通滤波：
      - l（int）：窗口大小，应大于0
      - c（double）：归一化的截止频率，应大于0



### 环境配置

- 安装`serial`
  - 使用serial库的精简版`serial-lite`安装，见陈伟浩的私有仓库
  - 文件夹中已放置了macOS平台下的动态库`libserial.so`和静态库`libserial.a`，以及头文件夹`include`，保证未安装时可运行
- 安装`fftw3`
- 需要pthread、socket支持



## Python代码

### 环境配置

需要使用python3，并安装如下python库：

- `numpy`，数值计算

  ```sh
  pip3 install numpy
  ```

- `scipy`，插值

  ```sh
  pip3 install scipy
  ```

串口：

- `pyserial`

  ```sh
  pip3 install pyserial
  ```

可视化相关：

- `matplotlib`

  ```sh
  pip3 install matplotlib
  ```

- `pyqtgraph`

  ```sh
  pip3 install pyqtgraph
  ```

  - Mac上需要用如下方式安装以确保3D显示正常（采用上面的方式其实也可，不过3D显示会只占用窗口的1/4面积）：

    ```sh
    pip3 install git+https://github.com/pyqtgraph/pyqtgraph
    ```

- `pyopengl`，`pyqtgraph`的3D可视化依赖

  ```sh
  pip3 install pyopengl
  ```

- `pyqt5`，`pyqtgraph`的3D可视化依赖

  ```sh
  pip3 install pyqt5
  ```

可选：

- Windows平台可安装`pyreadline`，方便程序中命令行文本输入时按上下键切换到历史输入

  ```sh
  pip3 install pyreadline
  ```



### 后端信号处理服务

C++后端服务程序的Python迁移版，代码入口是`main_server.py`。

运行方式：

```
usage: main_server.py [-h] [-e] [-p PORT] [-b BAUDRATE] [-t TIMEOUT] [-n N] [-s]
                      [-a ADDRESS] [-u] [-r] [-nc] [-v] [-f FPS] [--pyqtgraph]

optional arguments:
  -h, --help            show this help message and exit
  -e                    enumerate all serial ports (default: False)
  -p PORT               specify serial port (default: None)
  -b BAUDRATE           specify baudrate (default: 500000)
  -t TIMEOUT            specify timeout in seconds (default: 1)
  -n N                  specify sensor size (default: 16)
  -s, --service         run service (default: False)
  -a ADDRESS, --address ADDRESS
                        specify server socket address (default: None)
  -u, --udp             use UDP protocol (default: False)
  -r, --raw             raw data mode (default: False)
  -nc, --no_convert     do not apply voltage-resistance conversion (default:
                        False)
  -v, --visualize       enable visualization (default: False)
  -f FPS                frames per second (default: 100)
  --pyqtgraph           use pyqtgraph to plot (default: False)
```



### 接Arduino串口并可视化

方便调试，快速查看串口连接情况和数据图，代码的主入口是`main_server.py`，需要加上`-v`选项（否则是后端模式）。数据处理采用了和C++后端一致的流程。



### 接C++后端服务并可视化

代码主入口是`main_client.py`，运行方式：

```
usage: main_client.py [-h] [-s SERVER_ADDR] [-c CLIENT_ADDR] [-u] [-r] [-n N]
                      [--interp INTERP] [--noblob] [--th THRESHOLD] [-i]
                      [-f FPS] [-m]

optional arguments:
  -h, --help         show this help message and exit
  -s SERVER_ADDR     specify server socket address (default: None)
  -c CLIENT_ADDR     specify client socket address (default: None)
  -u, --udp          use UDP protocol (default: False)
  -r, --raw          plot raw data (default: False)
  -n N               sensor side size (default: 16)
  --interp INTERP    interpolated side size (default: None)
  --noblob           do not filter out blob (default: False)
  --th THRESHOLD     blob filter threshold (default: 0.15)
  -i, --interactive  interactive mode (default: False)
  -f FPS             frames per second (default: 194)
  -m, --matplot      use mathplotlib to plot (default: False)
```

注意在不指定CLIENT_ADDR的情况下，客户端地址默认为`/var/tmp/unix.socket.client.`+6位随机数字，以免启动多个客户端时地址冲突。当随机生成的地址冲突时，会再次重新生成可用地址。

两种模式：

- 可视化模式：默认，且在server断开情况下可以自动停止，直至server上线
  - 空格：播放/暂停
  - `q`：退出
- 交互模式：加`-i`选项进入，可以与后端服务通信，执行数据记录、获取帧、关闭服务等命令
  - 输入数据通信的数字及相应的命令，与server通信
  - 退出：输入`quit`的任何前缀、`exit`，或者使用按键中断`ctrl+c`、`ctrl+d`



### 读取文件数据并可视化

代码入口是`main_data.py`，运行方式：

```
usage: main_data.py [-h] [-n N] [-f FPS] [-m] [-o OUTPUT] [--interp INTERP]
                    [--noblob] [--th THRESHOLD]
                    filename

Visualize data, or process data via -o flag

positional arguments:
  filename

optional arguments:
  -h, --help       show this help message and exit
  -n N             sensor side size (default: 16)
  -f FPS           frames per second (default: 194)
  -m, --matplot    use mathplotlib to plot (default: False)
  -o OUTPUT        output processed data to file (default: None)
  --interp INTERP  interpolated side size (default: 16)
  --noblob         do not filter out blob (default: False)
  --th THRESHOLD   blob filter threshold (default: 0.15)
```

- 交互模式：实际上是一个简易播放器，可在可视化界面播放、暂停、快进、快退、逐帧调整、重播等操作，具体交互方式是：
  - 空格：播放/暂停
  - 左/右键：快退/快进若干帧
  - `,`：退一帧，`.`：进一帧
  - `a`：跳到开头（保持播放状态）
  - `e`：跳到结尾（暂停）
  - `j`：跳到指定帧（暂停），应切换到命令行窗口根据提示输入帧号，空串或不合法输入会自动跳出提示界面
  - `q`：退出



### 离线数据处理

将服务端记录的文件数据进行插值、滤出触摸区处理，并保存到文件，代码入口是`main_data.py`，需要加上`-o`选项及输出文件名，否则是可视化模式。



## 数据格式

默认采用英文逗号`,`分隔的CSV文本文件存储数据。每帧数据占据一行，每行的内容依次是：按顺序N*N个压力值，帧号（从运行时第一帧0算起），时间戳（从UNIX Epoch算起的时间长度，以微秒即10e-6秒为单位）。

可能采集的数据类型包括：处理后数据，原始数据。


