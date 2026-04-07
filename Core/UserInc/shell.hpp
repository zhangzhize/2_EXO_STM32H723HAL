#ifndef SHELL_HPP
#define SHELL_HPP

#include <cstdint>
#include "usart.h"

#define SHELL_TX_BUF_SIZE_BYTES         (512U)
#define SHELL_TX_BUF_SIZE_FLOATS        (SHELL_TX_BUF_SIZE_BYTES/4)
#define SHELL_VOFA_MAX_FLOAT_SIZE       (SHELL_TX_BUF_SIZE_FLOATS - 1)

union DmaBuffer {
    float f_data[SHELL_TX_BUF_SIZE_FLOATS];
    uint32_t u32_data[SHELL_TX_BUF_SIZE_FLOATS];
    char c_data[SHELL_TX_BUF_SIZE_BYTES];
    uint8_t u8_data[SHELL_TX_BUF_SIZE_BYTES];
};

typedef void (*ShellCmdHandler)(void* context, int argc, char** argv);

struct ShellCmdEntry {
    const char* cmd_name;
    ShellCmdHandler handler;
    void* context;
};

enum class ShellRwParamType : uint8_t
{ 
    kFloat, 
    kInt, 
    kBool 
};

struct ShellRwParamEntry
{
    const char* name; 
    ShellRwParamType type;   
    void* ptr;        
};

class Shell
{
public:
    Shell(UART_HandleTypeDef* huart = &huart9);
    virtual ~Shell() = default;

    void Printf(const char* format, ...);
    void SetVofaJustFloatData(uint16_t index, float value);
    void SendVofaJustFloatFrame(uint16_t float_size);
    void SendString(const char* str);
    void SendData(uint16_t data_size);

    bool RegisterCommand(const char* cmd_name, ShellCmdHandler handler, void* context);
    void PushPendingCommand(const uint8_t* rx_data, uint16_t len);
    bool ProcessPendingCommand();

    void RegisterRwParam(const char* name, float* ptr);
    void RegisterRwParam(const char* name, int* ptr);
    void RegisterRwParam(const char* name, bool* ptr);

    static int GetInt(int argc, char *argv[], int index, int default_val = 0)
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return atoi(argv[index]);
    }
    static float GetFloat(int argc, char *argv[], int index, float default_val = 0.0f)
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return strtof(argv[index], nullptr);
    }
    static const char *GetString(int argc, char *argv[], int index, const char *default_val = "")
    {
        if (index >= argc || argv[index] == nullptr)
            return default_val;
        return argv[index];
    }

protected:
    void OnCmdHelp(int argc, char** argv);
    void OnCmdWriteParam(int argc, char** argv);
    void OnCmdReadParam(int argc, char** argv);
    DmaBuffer* ptr_txbuffer_ = nullptr;
    UART_HandleTypeDef *ptr_huart_ = nullptr;
    static constexpr uint16_t kMaxNumCmds = 20U;  /** 支持20个命令 */
    ShellCmdEntry cmd_table_[kMaxNumCmds];
    uint16_t cmd_count_ = 0;
    uint8_t pending_cmd_buf_[UART9_RX_BUF_SIZE] = {0};
    volatile bool is_cmd_pending_ = false;
    static constexpr uint8_t kMaxNumSupportedArgvs = 10;    /** 每个命令支持的最大参数个数 */
    static constexpr uint16_t kMaxNumRwParams = 50;     /** 可读写参数的最大个数 */
    ShellRwParamEntry param_table_[kMaxNumRwParams];
    uint16_t param_count_ = 0;

    template <typename T, void (T::*MemFn)(int, char**)>
    static void CmdWrapper(void* context, int argc, char* argv[])
    {
        if (context != nullptr)
        {
            (static_cast<T*>(context)->*MemFn)(argc, argv);
        }
    }
};

#endif
