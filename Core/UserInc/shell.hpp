#ifndef SHELL_HPP
#define SHELL_HPP

#include <cstdint>
#include "usart.h"
#include "utils.h"

/**
 * @file    shell.hpp
 * @brief   串口命令行交互模块 (Shell)
 */
#define SHELL_VOFA_MAX_FLOAT_SIZE       (DMA_UNION_BUF_SIZE_FLOATS - 1)  /*!< VOFA JustFloat 单帧最大浮点数 (留一个给帧尾) */

/* 命令处理函数指针类型 */
typedef void (*ShellCmdHandler)(void *context, int argc, char **argv);

/* 命令表条目：保存命令名、处理函数与上下文 */
struct ShellCmdEntry {
    const char *cmd_name;       /*!< 命令名字符串 (区分大小写) */
    ShellCmdHandler handler;    /*!< 命令对应的处理函数 */
    void *context;              /*!< 传递给处理函数的上下文指针 (通常为 Shell 实例自身) */
};

/* 可读写参数的类型枚举 */
enum class ShellRwParamType : uint8_t
{
    kFloat,     /*!< 单精度浮点数 */
    kInt,       /*!< 有符号整数 */
    kBool       /*!< 布尔值 (非0即真) */
};

/* 可读写参数表条目：保存参数名、类型与变量地址 */
struct ShellRwParamEntry
{
    const char *name;           /*!< 参数名称 (write/read 命令匹配用) */
    ShellRwParamType type;      /*!< 参数类型 */
    void *ptr;                  /*!< 指向参数变量的指针 */
    bool has_float_range = false;
    float float_min = 0.0f;
    float float_max = 0.0f;
};

/**
 * @class Shell
 * @brief 串口命令行交互与波形显示终端
 */
class Shell
{
public:
    explicit Shell(UART_HandleTypeDef &huart);
    virtual ~Shell() = default;

    /* 启动 UART DMA 空闲中断接收，准备接收命令帧 */
    void UartReceiveDma(void);

    /* 格式化输出 (类 printf)，将结果写入 DMA 缓冲区后自动启动发送 */
    void Printf(const char *format, ...);
    /* 设置 VOFA JustFloat 帧中第 index 个浮点值，调用 SendVofaJustFloatFrame 后统一发送 */
    void SetVofaJustFloatData(uint16_t index, float value);
    /* 发送 VOFA JustFloat 帧：4字节小端浮点数 * float_size + 4字节帧尾 (0x00 0x00 0x80 0x7F) */
    void SendVofaJustFloatFrame(uint16_t float_size);
    /* 发送纯字符串 (不含格式化开销) */
    void SendString(const char *str);
    /* 启动 DMA 发送 txbuffer_ 中前 data_size 字节 */
    void SendData(uint16_t data_size);

    /* 注册一条命令到命令表 */
    bool RegisterCommand(const char *cmd_name, ShellCmdHandler handler, void *context);
    /* 将收到的原始 ASCII 数据推入待处理缓冲区 (丢弃正在处理中的命令) */
    void PushPendingCommand(const uint8_t *rx_data, uint16_t len);
    /* 解析并执行待处理缓冲区中的命令 (主循环中调用) */
    bool ProcessPendingCommand();

    /* 注册一个可读写浮点参数 (write/read 命令将修改/查询该变量) */
    void RegisterRwParam(const char *name, float *ptr);
    void RegisterRwParam(const char *name, float *ptr, float min_value, float max_value);
    /* 注册一个可读写整数参数 */
    void RegisterRwParam(const char *name, int *ptr);
    /* 注册一个可读写布尔参数 */
    void RegisterRwParam(const char *name, bool *ptr);

    /* 从 argv 中安全获取整数参数 */
    static inline int GetInt(int argc, char *argv[], int index, int default_val = 0)
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return atoi(argv[index]);
    }
    /* 从 argv 中安全获取浮点参数 */
    static inline float GetFloat(int argc, char *argv[], int index, float default_val = 0.0f)
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return strtof(argv[index], nullptr);
    }
    /* 从 argv 中安全获取字符串参数 */
    static inline const char *GetString(int argc, char *argv[], int index, const char *default_val = "")
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return argv[index];
    }

protected:
    DmaUnionBuffer &txbuffer_;          /*!< DMA 发送缓冲区引用 (指向外部静态 g_uart_tx_dma_buf) */
    UART_HandleTypeDef &huart_;         /*!< Shell 使用的串口句柄引用 */

    static constexpr uint16_t kMaxPendingCmdLen = 256U;       /*!< 待处理命令的最大字节长度 */
    static constexpr uint16_t kMaxNumCmds = 20U;              /*!< 支持的最大命令个数 */
    static constexpr uint16_t kMaxNumSupportedArgvs = 10;     /*!< 每个命令支持的最大参数个数 */
    static constexpr uint16_t kMaxNumRwParams = 50;           /*!< 可读写参数的最大个数 */

    /* 将成员函数包装为普通函数指针，适配 ShellCmdHandler 类型 */
    template <typename T, void (T::*MemFn)(int, char **)>
    static inline void CmdWrapper(void *context, int argc, char *argv[])
    {
        if (context != nullptr)
        {
            (static_cast<T *>(context)->*MemFn)(argc, argv);
        }
    }
private:
    void OnCmdHelp(int argc, char **argv);         /*!< help 命令：列出所有已注册命令 */
    void OnCmdWriteParam(int argc, char **argv);   /*!< write 命令：修改指定参数的值 */
    void OnCmdReadParam(int argc, char **argv);    /*!< read 命令：查询参数值 */

    ShellRwParamEntry param_table_[kMaxNumRwParams];  /*!< 参数表：保存所有已注册的可读写参数 */
    ShellCmdEntry cmd_table_[kMaxNumCmds];            /*!< 命令表：保存所有已注册的命令 */
    uint8_t pending_cmd_buf_[kMaxPendingCmdLen] = {0};/*!< 待处理命令的原始数据缓冲区 */
    uint16_t param_count_ = 0;                        /*!< 当前已注册的参数个数 */
    uint16_t cmd_count_ = 0;                          /*!< 当前已注册的命令个数 */
    volatile bool is_cmd_pending_ = false;            /*!< 标记是否有命令等待处理 */
};

#endif
