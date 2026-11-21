# Linux 构建（GCC + CMake）

本目录除原有的 Keil uVision5 工程（`MDK-ARM/`）外，附带一套等价的 **GCC + CMake** 构建配置：
同一份源码、同样的存储地址，可在 Linux / macOS / Windows(MSYS2) 下编译，不依赖 Keil。

## 依赖

- `arm-none-eabi-gcc` >= 11（链接脚本用到了 `READONLY` 段属性）以及 `arm-none-eabi-objcopy`、`arm-none-eabi-size`
- `cmake` >= 3.22、`ninja`
- `openocd`（仅下载 / 擦除时需要）

装好 STM32CubeCLT 或 STM32CubeIDE 后这些工具都在其 bundle 里，把 `bin` 目录加入 `PATH` 即可；
Arch Linux 也可以 `pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake ninja openocd`。

## 编译

```bash
make            # = make artifacts：configure + 编译 + 生成 hex/bin + 打印尺寸
make release    # Release 预设（-Os），固件更小
make size       # 只编译并打印 size
make flash      # 编译并下载（J-Link，见 openocd.cfg）
make erase      # 整片擦除（会清掉 Flash 里保存的电机参数与 ID，需重新标定）
make reset      # 复位运行
make clean
```

也可以直接用 cmake：

```bash
cmake --preset Debug          # 或 Release
cmake --build --preset Debug
arm-none-eabi-size build/Debug/C6T6SimpleFoc.elf
```

产物在 `build/Debug/` 或 `build/Release/` 下：`C6T6SimpleFoc.elf` / `.hex` / `.bin` / `.map`。

## 资源占用（实测）

| 预设 | 标志 | FLASH | RAM |
| --- | --- | --- | --- |
| Debug | `-Og -g3` | 26000 B / 32 KB (79.4%) | 3496 B / 10 KB (34.1%) |
| Release | `-Os -g0` | 24420 B / 32 KB (74.5%) | 3496 B / 10 KB (34.1%) |

32KB Flash 偏紧，**Debug 预设用 `-Og` 而不是 `-O0`**：`-O0` 会溢出约 844 字节。

## 目录内的构建文件

| 文件 | 说明 |
| --- | --- |
| `CMakeLists.txt`、`CMakePresets.json` | 顶层 CMake 工程，风格与 STM32CubeMX 的 CMake 转换器一致 |
| `cmake/gcc-arm-none-eabi.cmake` | 工具链文件：`-mcpu=cortex-m3`、无 FPU（软件浮点）、`--specs=nano.specs` |
| `cmake/starm-clang.cmake` | 备用的 ArmClang 工具链文件（未在本机验证） |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX 生成的源文件 / HAL 驱动清单 |
| `STM32F103C6TX_FLASH.ld` | 链接脚本：32KB FLASH / 10KB RAM，`_Min_Stack_Size=0x800`、`_Min_Heap_Size=0x200` |
| `startup_stm32f103x6.s` | GCC 启动文件（取自 `Drivers/CMSIS/.../Templates/gcc/`） |
| `Src/syscalls.c`、`Src/sysmem.c` | newlib 系统调用桩，`_sbrk` 使用链接脚本中的 `_end` / `_Min_Stack_Size` |
| `Makefile`、`openocd.cfg` | 命令行封装与调试器配置（J-Link SWD，`reset_config none`） |
| `.vscode/`、`.gitignore` | 编辑器索引（`build/Debug/compile_commands.json`）与构建产物忽略规则 |

## 与 Keil 工程的关系

- 源码目录仍是 `Src` / `Inc` / `USER`；`.ioc` 里的工具链设置保持 **MDK-ARM**，
  因此用 CubeMX 重新生成代码不会覆盖上面的 CMake 文件，也不会移动源码路径。
- 两套构建共用同一份源码，只有下列几处为兼容 GCC 做的**等价**调整：

| 文件 | 调整 | 原因 |
| --- | --- | --- |
| `Src/main.c`、`USER/BLDCMotor.c`、`USER/FOCMotor.c` | `#include "BLDCmotor.h"` → `"BLDCMotor.h"` | Linux 文件系统区分大小写 |
| `Src/main.c` | `const uint8_t speedLpfLen = 5;` → `enum { speedLpfLen = 5 };` | C 标准不允许用变量作数组长度初始化 |
| `Src/main.c` | 新增空实现 `__io_putchar()` | Keil 下靠空 `fputc()` 丢弃 `printf`；newlib 走 `_write()`，不补会调用空指针 |

- 工具链文件里额外加了 `-fno-strict-aliasing`：`USER/foc_utils.c` 的快速平方根倒数、
  `USER/FlashStorage.c` 的字打包都通过指针强转做类型双关（type punning），
  关掉严格别名假设才能保证 `-O2/-Os` 下不被优化坏（CAN_TEST / twitch_gimbal 的模板没有这项，属于本工程特有）。

## 已知上游遗留问题（未改动）

`USER/FlashStorage.c:38`：

```c
for(uint8_t i=0; i<sizeof(buf); i++)      // buf 是 uint32_t[5]，sizeof = 20
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_PARAM_ADDR+i*4, buf[i]);
```

应为 `i < sizeof(buf)/sizeof(buf[0])`。当前会多迭代 15 次擦写并读到数组外的栈数据，
GCC 在 `-Waggressive-loop-optimizations` 下报警（Keil 不报）；因为读回只用到前 4 个字，
所以暂不影响功能，只是多写了 60 字节无效数据。修不修由你决定，改一处循环条件即可。

## 其它说明

- `openocd.cfg` 默认走 **J-Link**（`interface/jlink.cfg`，SWD 1MHz）。若用 ST-Link，
  把第一行换成 `source [find interface/stlink.cfg]` 即可（`reset_config none` 保持不变，
  因为板上 SWD 排针通常没接 NRST，复位靠 Cortex-M 的 SYSRESETREQ）。
- 串口没接出来也用不到：本工程不使用 `printf` 输出（`fputc` / `__io_putchar` 都是空实现）。
- 编译时出现 GBK 编码的源文件不会报错，GCC 只在字符串字面量里有非 ASCII 字符时才有影响；本工程没有。
