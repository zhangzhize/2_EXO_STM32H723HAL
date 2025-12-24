#ifndef VOFA_HPP
#define VOFA_HPP

#include <cstdint>

#define VOFA_MAX_FLOAT_SIZE (32)

class Vofa
{
public:
    Vofa();
    ~Vofa() = default;

    float *ptr_vofa_data_ = nullptr;
    void SendOneFrame(uint16_t float_size);
private:
    uint8_t vofa_data_[(VOFA_MAX_FLOAT_SIZE + 1) * 4] = {0};
};

extern "C" {

struct Vofa *CallVofaCreate(void);
void CallVofaDestroy(struct Vofa **pptr_vofa);
void CallVofaSendOneFrame(struct Vofa *ptr_vofa, uint16_t float_size);

}


#endif
