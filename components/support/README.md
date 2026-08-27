# Support

当前目录提供 RoboMaster 常用 CRC8/CRC16 查表实现，可用于裁判系统、VT13 和视觉通信协议。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/support/CRC8_CRC16.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/support
)
```

头文件提供 `extern "C"` 兼容，可同时被 C 和 C++ 调用。

## API

```c
uint8_t crc8 = get_CRC8_check_sum(data, data_length, 0xff);
uint32_t crc8_ok = verify_CRC8_check_sum(frame, frame_length);
append_CRC8_check_sum(frame, frame_length);

uint16_t crc16 = get_CRC16_check_sum(data, data_length, 0xffff);
uint32_t crc16_ok = verify_CRC16_check_sum(frame, frame_length);
append_CRC16_check_sum(frame, frame_length);
```

`append_*` 的长度包含尾部校验字段：CRC8 占最后 1 字节，CRC16 按低字节在前写入最后 2 字节。`verify_*` 同样要求传入完整帧长度。

## 使用示例

```cpp
uint8_t frame[8] = {0xA5, 0, 0, 0, 0, 0, 0, 0};
append_CRC16_check_sum(frame, sizeof(frame));

if (verify_CRC16_check_sum(frame, sizeof(frame)))
{
    // 校验通过
}
```

## 注意事项

- CRC 参数与 RoboMaster 常用协议一致；接入其他协议前先确认初值、多项式、反射和字节序。
- 输入指针不能为空，长度必须包含校验字段并满足函数的最小长度要求。

