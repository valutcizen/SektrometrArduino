#include "SPTPP.h"

SPTPP::SPTPP(uint8_t* obj, int size, unsigned long timeout)
{
    this->obj = obj;
    this->size = size;
    this->timeout = timeout;
    buffer = new uint8_t[size + 5];
    pos = 0;
}

void SPTPP::GetedByte(uint8_t b)
{
    unsigned long act = millis();
    if (pos >= size + 5 || act - lastTime > timeout)
        pos = 0;
    lastTime = act;

    if (pos == 0)
    {
        if (b == 0xf0 || b == 0xaa)
        {
            buffer[0] = b;
            pos = 1;
        }
    }
    else if (buffer[0] == 0xaa)
    {
        buffer[pos++] = b;
        if (pos >= 5)
            RunRead();
    }
    else if (buffer[0] == 0xf0)
    {
        buffer[pos++] = b;

        if (pos == 2)
            actSize = b + 5;
        else if (pos >= actSize)
            RunWrite();
    }
}

int SPTPP::CheckFletcher16( uint8_t *data, int count )
{
    uint8_t* cs = Fletcher16(data, count);
    return cs[0] == data[count] && cs[1] == data[count + 1];
}

uint8_t* SPTPP::Fletcher16( uint8_t *data, int count )
{
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;

    for( index = 0; index < count; ++index )
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    uint8_t* res = new uint8_t[2];
    res[0] = (uint8_t)sum1;
    res[1] = (uint8_t)sum2;
    return res;
}

void SPTPP::RunRead()
{
    if (CheckFletcher16(buffer, 3))
    {
        int adr = buffer[1];
        int count = buffer[2];

        if (adr + count < size)
        {
            uint8_t* buf = new uint8_t[count + 4];

            buf[0] = 0x55;
            buf[1] = buffer[2];

            for (int i = 0; i < count; ++i)
                buf[2 + i] = obj[adr + i];
            
            uint8_t* cs = Fletcher16(buf, count + 2);
            buf[count+2] = cs[0];
            buf[count+3] = cs[1];
            Serial.write(buf, count + 4);
            Serial.flush();
        }
        else
            SendReadError();
        SetNewBuffer(5);
    }
    else
        CheckNextBuffPosition();
}

void SPTPP::RunWrite()
{
    if (CheckFletcher16(buffer, buffer[1] + 3))
    {
        int adr = buffer[1];
        int count = buffer[2];

        if (adr + count < size)
        {
            for (int i = 0; i < count; ++i)
                obj[adr + i] = buffer[3 + i];

            buffer[0] = 0x0f;
            buffer[1] = buffer[2];
            uint8_t* cs = Fletcher16(buffer, 2);
            buffer[2] = cs[0];
            buffer[3] = cs[1];

            Serial.write(buffer, 4);
            Serial.flush();
        }
        else
            SendWriteError();

        SetNewBuffer(buffer[1] + 5);
    }
    else
        CheckNextBuffPosition();
}

void SPTPP::CheckNextBuffPosition()
{
    for (int i = 1; i < pos; ++i)
    {
        if (buffer[i] == 0xf0)
        {
            SetNewBuffer(i);
            if (pos > 1 && buffer[1] + 5 >= pos)
                RunWrite();
            return;
        }
        else if (i >= pos - 5 && buffer[i] == 0xaa)
        {
            SetNewBuffer(i);
            if (pos == 5)
                RunRead();
            return;
        }
    }
}

void SPTPP::SetNewBuffer(int p)
{
    for (int i = 0; i < pos - p; ++i)
        buffer[i] = buffer[p + i];
    pos -= p;
    
    if (pos < 0)
        pos = 0;
}

void SPTPP::SendReadError()
{
    uint8_t buf[4] = {0x55, 0x00, 0x55, 0xAA};
    Serial.write(buf, 4);
    Serial.flush();
}

void SPTPP::SendWriteError()
{
    uint8_t buf[4] = {0xf0, 0x00, 0x0F, 0x1E};
    Serial.write(buf, 4);
    Serial.flush();
}
