/**
 * @file    shell.cpp
 * @brief   Shell 模块实现：命令解析、参数读写、VOFA 波形 & DMA 传输
 */
#include "shell.hpp"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usart.h"
#include <cstring>
#include <cstdarg>

#define SHELL_UART_RX_BUF_SIZE              256     /*!< UART DMA 接收缓冲区大小 (字节) */

/* 全局 DMA 发送缓冲区，位于 DTCM 的 .dma_buf 段，32 字节对齐以满足 DMA 对齐要求 */
__attribute__((section(".dma_buf"), aligned(32))) DmaUnionBuffer g_uart_tx_dma_buf;
/* 全局 UART 接收缓冲区，供 DMA 空闲中断写入 */
__attribute__((section(".dma_buf"), aligned(32))) uint8_t shell_uart_rx_buffer[SHELL_UART_RX_BUF_SIZE];

Shell::Shell(UART_HandleTypeDef &huart): txbuffer_(g_uart_tx_dma_buf), huart_(huart)
{
    is_cmd_pending_ = false;        /* 初始无待处理命令 */
    param_count_ = 0;               /* 参数表为空 */

    /* 注册 3 条内置命令，通过 CmdWrapper 将成员函数转为普通函数指针 */
    RegisterCommand("help", CmdWrapper<Shell, &Shell::OnCmdHelp>, this);
    RegisterCommand("write", CmdWrapper<Shell, &Shell::OnCmdWriteParam>, this);
    RegisterCommand("read", CmdWrapper<Shell, &Shell::OnCmdReadParam>, this);
}

/**
 * @brief 启动 UART DMA 空闲中断接收
 */
void Shell::UartReceiveDma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_, shell_uart_rx_buffer, SHELL_UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart_.hdmarx, DMA_IT_HT);   /* 仅需要 IDLE 中断，半传输中断无意义 */
}

/**
 * @brief 格式化输出到串口 (DMA 方式)
 */
void Shell::Printf(const char *format, ...)
{
    if (!format) return;
    if (huart_.gState != HAL_UART_STATE_READY) return;   /* DMA 忙则丢弃，避免破坏正在发送的数据 */

    va_list args;
    va_start(args, format);
    int len = vsnprintf(txbuffer_.c_data, DMA_UNION_BUF_SIZE_BYTES, format, args);
    va_end(args);

    if (len > 0)
    {
        /* 限制发送长度不超过 DMA 缓冲区容量 */
        uint16_t send_len = ((uint32_t)len < DMA_UNION_BUF_SIZE_BYTES) ? static_cast<uint16_t>(len) : static_cast<uint16_t>(DMA_UNION_BUF_SIZE_BYTES - 1);
        SendData(send_len);
    }
}

/**
 * @brief 设置 VOFA JustFloat 帧中指定通道的浮点值
 * @param index  通道索引 (0 ~ SHELL_VOFA_MAX_FLOAT_SIZE-1)
 * @param value  浮点值
 */
void Shell::SetVofaJustFloatData(uint16_t index, float value)
{
    if (index >= SHELL_VOFA_MAX_FLOAT_SIZE) return;

    if (index < SHELL_VOFA_MAX_FLOAT_SIZE)
    {
        txbuffer_.f_data[index] = value;
    }
}

/**
 * @brief 发送 VOFA JustFloat 帧
 * @param float_size  本帧包含的浮点数个数
 */
void Shell::SendVofaJustFloatFrame(uint16_t float_size)
{
    if (float_size == 0 || float_size > SHELL_VOFA_MAX_FLOAT_SIZE) return;

    uint16_t count = 4 * float_size;
    /* 追加 JUST FLOAT 协议帧尾：VOFA+ 以此检测帧边界并区分协议类型 */
    txbuffer_.u8_data[count++] = 0x00;
    txbuffer_.u8_data[count++] = 0x00;
    txbuffer_.u8_data[count++] = 0x80;
    txbuffer_.u8_data[count++] = 0x7f;
    SendData(count);
}

/**
 * @brief 发送纯字符串 (无格式化)
 */
void Shell::SendString(const char *str)
{
    if (str == nullptr) return;
    if (huart_.gState != HAL_UART_STATE_READY) return;

    const uint16_t max_len = DMA_UNION_BUF_SIZE_BYTES - 1;   /* 保留一个字节给 '\0' */
    const size_t len = strnlen(str, max_len);
    memcpy(txbuffer_.u8_data, str, len);
    txbuffer_.u8_data[len] = '\0';

    SendData(static_cast<uint16_t>(len));
}

/**
 * @brief 启动 DMA 发送 txbuffer_ 中前 data_size 字节
 *
 * 所有输出函数的最终出口。调用 HAL_UART_Transmit_DMA 后立即返回，
 * DMA 控制器在后台逐个字节送入 USART 数据寄存器，CPU 不等待。
 * 发送前再次检查 gState == READY 作为最终防护。
 */
void Shell::SendData(uint16_t data_size)
{
    if (data_size == 0 || data_size > DMA_UNION_BUF_SIZE_BYTES) return;

    if (huart_.gState == HAL_UART_STATE_READY && data_size < DMA_UNION_BUF_SIZE_BYTES)
    {
        HAL_UART_Transmit_DMA(&huart_, txbuffer_.u8_data, data_size);
    }
}

/**
 * @brief 注册命令到命令表
 * @param cmd_name  命令名字符串 (区分大小写，通常不含空格)
 * @param handler   命令处理函数
 * @param context   传递给 handler 的上下文指针
 * @return true 注册成功，false 命令表已满
 */
bool Shell::RegisterCommand(const char *cmd_name, ShellCmdHandler handler, void *context)
{
    if (cmd_count_ >= kMaxNumCmds) return false;   /* 命令表已满 */

    cmd_table_[cmd_count_].cmd_name = cmd_name;
    cmd_table_[cmd_count_].handler = handler;
    cmd_table_[cmd_count_].context = context;
    cmd_count_++;
    return true;
}

/**
 * @brief 将收到的原始数据推入待处理命令缓冲区
 * @param rx_data  原始字节数据起始地址
 * @param len      数据长度
 */
void Shell::PushPendingCommand(const uint8_t *rx_data, uint16_t len)
{
    /* 上一条命令尚未处理则丢弃新命令，避免覆盖 */
    if (is_cmd_pending_) return;

    /* 拷贝到待处理缓冲区，超长截断 */
    uint16_t copy_len = (len < sizeof(pending_cmd_buf_) - 1) ? len : (sizeof(pending_cmd_buf_) - 1);
    memcpy(pending_cmd_buf_, rx_data, copy_len);
    pending_cmd_buf_[copy_len] = '\0';    /* 确保 C 字符串终止 */
    is_cmd_pending_ = true;

    UartReceiveDma();   /* 重启 DMA 接收，准备接收下一条命令 */
}

/**
 * @brief 处理待处理命令 (主循环中调用)
 * @return true  有待处理命令且已执行 (或已提示未找到)
 * @return false 无待处理命令
 */
bool Shell::ProcessPendingCommand()
{
    if (!is_cmd_pending_) return false;

    char *cmd_line = reinterpret_cast<char *>(pending_cmd_buf_);
    char *argv[kMaxNumSupportedArgvs];
    int argc = 0;
    char *saveptr;

    /* 以空格、\r、\n 为分隔符切词 */
    char *token = strtok_r(cmd_line, " \r\n", &saveptr);
    while (token != nullptr && argc < kMaxNumSupportedArgvs)
    {
        argv[argc++] = token;
        token = strtok_r(nullptr, " \r\n", &saveptr);
    }

    if (argc == 0)
    {
        is_cmd_pending_ = false;
        return false;
    }

    /* 在命令表中进行线性查找 */
    for (uint16_t i = 0; i < cmd_count_; i++)
    {
        if (strcmp(argv[0], cmd_table_[i].cmd_name) == 0)
        {
            cmd_table_[i].handler(cmd_table_[i].context, argc, argv);
            is_cmd_pending_ = false;
            return true;
        }
    }

    SendString(" Command not found, Type 'help' to list commands.\r\n");
    is_cmd_pending_ = false;

    /* 返回 true 让调用方知道 Shell 已处理过此输入 (即使未找到命令) */
    return true;
}


/**
 * @brief 注册浮点型可读写参数
 */
void Shell::RegisterRwParam(const char *name, float *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kFloat, static_cast<void *>(ptr)};
}
/**
 * @brief 注册整型可读写参数
 */
void Shell::RegisterRwParam(const char *name, int *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kInt, static_cast<void *>(ptr)};
}
/**
 * @brief 注册布尔型可读写参数
 */
void Shell::RegisterRwParam(const char *name, bool *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kBool, static_cast<void *>(ptr)};
}

/**
 * @brief help 命令处理：列举所有已注册命令
 */
void Shell::OnCmdHelp(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (huart_.gState != HAL_UART_STATE_READY) return;

    char *buf = txbuffer_.c_data;
    const size_t max_len = DMA_UNION_BUF_SIZE_BYTES;
    size_t offset = 0;

    offset += snprintf(buf + offset, max_len - offset, "\r\nAvailable Commands\r\n");

    for (uint16_t i = 0; i < cmd_count_; i++)
    {
        if (offset + 32 >= max_len) break;   /* 接近缓冲区末尾则截断 */
        offset += snprintf(buf + offset, max_len - offset, "-%s\r\n", cmd_table_[i].cmd_name);
    }
    SendData(static_cast<uint16_t>(offset));
}

/**
 * @brief write 命令处理：修改指定参数的值
 */
void Shell::OnCmdWriteParam(int argc, char **argv)
{
    if (argc < 3) {
        Printf("Usage: write <param_name> <value>\r\n");
        return;
    }

    const char *target_name = argv[1];

    for (uint16_t i = 0; i < param_count_; i++)
    {
        if (strcmp(param_table_[i].name, target_name) == 0)
        {
            switch (param_table_[i].type)
            {
                case ShellRwParamType::kFloat: {
                    float val = strtof(argv[2], nullptr);
                    *(static_cast<float *>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %.3f\r\n", target_name, val);
                    break;
                }
                case ShellRwParamType::kInt: {
                    int val = atoi(argv[2]);
                    *(static_cast<int *>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %d\r\n", target_name, val);
                    break;
                }
                case ShellRwParamType::kBool: {
                    bool val = (atoi(argv[2]) > 0);
                    *(static_cast<bool *>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %s\r\n", target_name, val ? "true" : "false");
                    break;
                }
            }
            return;
        }
    }

    Printf("Error: Parameter '%s' not found.\r\n", target_name);
}

/**
 * @brief read 命令处理：查询参数值
 *
 * 无参数 (argc < 2)：列出所有已注册参数及当前值
 * 有参数：read <param_name>，仅查询指定参数
 */
void Shell::OnCmdReadParam(int argc, char **argv)
{
    if (argc < 2)
    {
        /* 列出全部参数 */
        char out[DMA_UNION_BUF_SIZE_BYTES];
        size_t used = 0;

        int n = snprintf(out + used, sizeof(out) - used, "Params(%u):\r\n", static_cast<unsigned>(param_count_));
        if (n < 0)
        {
            return;
        }
        used += static_cast<size_t>(n);

        for (uint16_t i = 0; i < param_count_; ++i)
        {
            if (used >= sizeof(out) - 1)
            {
                break;   /* 缓冲区将满，截断剩余参数 */
            }

            switch (param_table_[i].type)
            {
            case ShellRwParamType::kFloat:
                n = snprintf(out + used, sizeof(out) - used, "%s=%.3f\r\n", (param_table_[i].name != nullptr) ? param_table_[i].name : "(null)",
                             *(static_cast<float *>(param_table_[i].ptr)));
                break;
            case ShellRwParamType::kInt:
                n = snprintf(out + used, sizeof(out) - used, "%s=%d\r\n", (param_table_[i].name != nullptr) ? param_table_[i].name : "(null)", *(static_cast<int *>(param_table_[i].ptr)));
                break;
            case ShellRwParamType::kBool:
                n = snprintf(out + used, sizeof(out) - used, "%s=%c\r\n", (param_table_[i].name != nullptr) ? param_table_[i].name : "(null)",
                             *(static_cast<bool *>(param_table_[i].ptr)) ? '1' : '0');
                break;
            default:
                n = snprintf(out + used, sizeof(out) - used, "%s=?\r\n", (param_table_[i].name != nullptr) ? param_table_[i].name : "(null)");
                break;
            }

            if (n < 0)
            {
                break;
            }
            used += static_cast<size_t>(n);
        }

        SendString(out);
        return;
    }

    /* 查询单个参数 */
    const char *target_name = argv[1];
    for (uint16_t i = 0; i < param_count_; i++)
    {
        if (strcmp(param_table_[i].name, target_name) == 0)
        {
            switch (param_table_[i].type) {
                case ShellRwParamType::kFloat:
                    Printf("%s = %.3f\r\n", target_name, *(static_cast<float *>(param_table_[i].ptr)));
                    break;
                case ShellRwParamType::kInt:
                    Printf("%s = %d\r\n", target_name, *(static_cast<int *>(param_table_[i].ptr)));
                    break;
                case ShellRwParamType::kBool:
                    Printf("%s = %s\r\n", target_name, *(static_cast<bool *>(param_table_[i].ptr)) ? "true" : "false");
                    break;
            }
            return;
        }
    }
    Printf("Error: Parameter '%s' not found.\r\n", target_name);
}