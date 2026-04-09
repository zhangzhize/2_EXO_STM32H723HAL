#include "shell.hpp"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usart.h"
#include <cstring>
#include <cstdarg>

__attribute__((section(".dma_buf"), aligned(32))) DmaBuffer g_uart_tx_dma_buf;

Shell::Shell(UART_HandleTypeDef* huart)
{
    ptr_txbuffer_ = &g_uart_tx_dma_buf;
    ptr_huart_ = huart;
    is_cmd_pending_ = false;
    param_count_ = 0;
    
    RegisterCommand("help", CmdWrapper<Shell, &Shell::OnCmdHelp>, this);
    RegisterCommand("write", CmdWrapper<Shell, &Shell::OnCmdWriteParam>, this);
    RegisterCommand("read", CmdWrapper<Shell, &Shell::OnCmdReadParam>, this);
}

void Shell::Printf(const char* format, ...)
{
    if (!format || !ptr_txbuffer_ || !ptr_huart_) return;
    if (ptr_huart_->gState != HAL_UART_STATE_READY) return;

    va_list args;
    va_start(args, format);
    int len = vsnprintf(ptr_txbuffer_->c_data, SHELL_TX_BUF_SIZE_BYTES, format, args);
    va_end(args);

    if (len > 0)
    {
        uint16_t send_len = ((uint32_t)len < SHELL_TX_BUF_SIZE_BYTES) ? static_cast<uint16_t>(len) : static_cast<uint16_t>(SHELL_TX_BUF_SIZE_BYTES - 1);
        SendData(send_len);
    }
}

void Shell::SetVofaJustFloatData(uint16_t index, float value)
{
    if (index >= SHELL_VOFA_MAX_FLOAT_SIZE || ptr_txbuffer_ == nullptr) return;

    if (index < SHELL_VOFA_MAX_FLOAT_SIZE && ptr_txbuffer_ != nullptr)
    {
        ptr_txbuffer_->f_data[index] = value; 
    }
}

void Shell::SendVofaJustFloatFrame(uint16_t float_size)
{
    if (ptr_txbuffer_ == nullptr || float_size == 0 || float_size > SHELL_VOFA_MAX_FLOAT_SIZE)
    {
        return;
    }

    uint16_t count = 4 * float_size;
    /* 补上JUST FLOAT格式 帧尾 */
    ptr_txbuffer_->u8_data[count++] = 0x00;
    ptr_txbuffer_->u8_data[count++] = 0x00;
    ptr_txbuffer_->u8_data[count++] = 0x80;
    ptr_txbuffer_->u8_data[count++] = 0x7f;
    SendData(count);
}

void Shell::SendString(const char *str)
{
    if (str == nullptr || ptr_txbuffer_ == nullptr) return;
    if (ptr_huart_->gState != HAL_UART_STATE_READY) return;

    const uint16_t max_len = SHELL_TX_BUF_SIZE_BYTES - 1;
    const size_t len = strnlen(str, max_len);
    memcpy(ptr_txbuffer_->u8_data, str, len);
    ptr_txbuffer_->u8_data[len] = '\0';

    SendData(static_cast<uint16_t>(len));
}

void Shell::SendData(uint16_t data_size)
{
    if (ptr_huart_ == nullptr || ptr_txbuffer_ == nullptr || data_size == 0 || data_size > SHELL_TX_BUF_SIZE_BYTES)
    {
        return;
    }
    if (ptr_huart_->gState == HAL_UART_STATE_READY && data_size < SHELL_TX_BUF_SIZE_BYTES)
    {
        HAL_UART_Transmit_DMA(ptr_huart_, ptr_txbuffer_->u8_data, data_size);
	    // CDC_Transmit_HS(ptr_txbuffer_->u8_data, data_size);   //zzz: just for debug
    }
}

bool Shell::RegisterCommand(const char* cmd_name, ShellCmdHandler handler, void* context)
{
    if (cmd_count_ >= kMaxNumCmds)
    {
        return false;
    }

    cmd_table_[cmd_count_].cmd_name = cmd_name;
    cmd_table_[cmd_count_].handler = handler;
    cmd_table_[cmd_count_].context = context;
    cmd_count_++;
    return true;
}

void Shell::PushPendingCommand(const uint8_t* rx_data, uint16_t len)
{
    /** Throw away new command if previous one is still pending */
    if (is_cmd_pending_)  
    {
        return; 
    }

    /** Copy new command to pending buffer */
    uint16_t copy_len = (len < sizeof(pending_cmd_buf_) - 1) ? len : (sizeof(pending_cmd_buf_) - 1);
    memcpy(pending_cmd_buf_, rx_data, copy_len);
    pending_cmd_buf_[copy_len] = '\0';
    is_cmd_pending_ = true;
}

bool Shell::ProcessPendingCommand()
{
    if (!is_cmd_pending_)
    {
        return false;
    }

    char* cmd_line = reinterpret_cast<char*>(pending_cmd_buf_);
    char* argv[kMaxNumSupportedArgvs];
    int argc = 0;
    char* saveptr;

    char* token = strtok_r(cmd_line, " \r\n", &saveptr);
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

    /** 返回真单纯是为了能让上位机看到: Command not found */
    return true; 
}


void Shell::RegisterRwParam(const char *name, float *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kFloat, static_cast<void *>(ptr)};
}
void Shell::RegisterRwParam(const char *name, int *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kInt, static_cast<void *>(ptr)};
}
void Shell::RegisterRwParam(const char *name, bool *ptr)
{
    if (param_count_ < kMaxNumRwParams && ptr != nullptr)
        param_table_[param_count_++] = {name, ShellRwParamType::kBool, static_cast<void *>(ptr)};
}

void Shell::OnCmdHelp(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    if (ptr_huart_->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    char* buf = ptr_txbuffer_->c_data;
    const size_t max_len = SHELL_TX_BUF_SIZE_BYTES;
    size_t offset = 0;

    offset += snprintf(buf + offset, max_len - offset, "\r\nAvailable Commands\r\n");

    for (uint16_t i = 0; i < cmd_count_; i++)
    {
        if (offset + 32 >= max_len) break; 
        offset += snprintf(buf + offset, max_len - offset, "-%s\r\n", cmd_table_[i].cmd_name);
    }
    SendData(static_cast<uint16_t>(offset));
}

void Shell::OnCmdWriteParam(int argc, char **argv)
{
    if (argc < 3) {
        Printf("Usage: write <param_name> <value>\r\n");
        return;
    }

    const char* target_name = argv[1];
    
    for (uint16_t i = 0; i < param_count_; i++) 
    {
        if (strcmp(param_table_[i].name, target_name) == 0) 
        {
            switch (param_table_[i].type) 
            {
                case ShellRwParamType::kFloat: {
                    float val = strtof(argv[2], nullptr);
                    *(static_cast<float*>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %.3f\r\n", target_name, val);
                    break;
                }
                case ShellRwParamType::kInt: {
                    int val = atoi(argv[2]);
                    *(static_cast<int*>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %d\r\n", target_name, val);
                    break;
                }
                case ShellRwParamType::kBool: {
                    bool val = (atoi(argv[2]) > 0);
                    *(static_cast<bool*>(param_table_[i].ptr)) = val;
                    Printf("OK: %s = %s\r\n", target_name, val ? "true" : "false");
                    break;
                }
            }
            return;
        }
    }
    
    Printf("Error: Parameter '%s' not found.\r\n", target_name);
}

void Shell::OnCmdReadParam(int argc, char **argv)
{
    if (argc < 2)
    {
        char out[SHELL_TX_BUF_SIZE_BYTES];
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
                break;
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

    const char* target_name = argv[1];
    for (uint16_t i = 0; i < param_count_; i++) 
    {
        if (strcmp(param_table_[i].name, target_name) == 0) 
        {
            switch (param_table_[i].type) {
                case ShellRwParamType::kFloat:
                    Printf("%s = %.3f\r\n", target_name, *(static_cast<float*>(param_table_[i].ptr)));
                    break;
                case ShellRwParamType::kInt:
                    Printf("%s = %d\r\n", target_name, *(static_cast<int*>(param_table_[i].ptr)));
                    break;
                case ShellRwParamType::kBool:
                    Printf("%s = %s\r\n", target_name, *(static_cast<bool*>(param_table_[i].ptr)) ? "true" : "false");
                    break;
            }
            return;
        }
    }
    Printf("Error: Parameter '%s' not found.\r\n", target_name);
}