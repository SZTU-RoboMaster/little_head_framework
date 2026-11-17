# USB BSP

USB Device CDC 接收回调转发模块。它保存一个上层回调，并由 CubeMX 生成的 CDC 接收函数调用 `BspUsb_RxCallback()` 完成分发。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/usb/bsp_usb.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/usb
)
```

依赖 CubeMX 生成的 USB Device CDC 工程、`usbd_cdc_if.h` 和 STM32 USB Device Middleware。

## CubeMX 配置

启用 `USB_OTG_FS` Device Only 和 USB Device `CDC` Class，并生成 `USB_DEVICE` 目录。当前工程 OTG FS 中断优先级为 `5`。

## 接入 CDC 接收函数

在 `USB_DEVICE/App/usbd_cdc_if.c` 的 `CDC_Receive_FS()` 用户代码区调用 BSP：

```c
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    BspUsb_RxCallback(Buf, *Len);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}
```

## 使用方法

```c
#include "bsp_usb.h"
#include "vision.h"

void vision_rx_callback(uint8_t *buffer, uint32_t length)
{
    Vision_UsbRxCallback(buffer, length);
}

void device_start(void)
{
    BspUsb_Init(vision_rx_callback);
}
```

发送接口不再额外封装，直接调用：

```c
CDC_Transmit_FS(data, length);
```

调用者需要处理 `USBD_BUSY`，避免上一帧尚未发送完成时覆盖发送缓冲区。

## 注意事项

- 接收回调只有在全局 `initialized != 0` 后才会分发。
- `BspUsb_RxCallback()` 是 C 函数；头文件在 C++ 文件中通过 `extern "C"` 声明，供 C 与 C++ 调用。
- USB 回调处于中断/协议栈回调上下文，不要阻塞。
- CubeMX 重新生成代码后，确认 `CDC_Receive_FS()` 中的用户代码仍然存在。

