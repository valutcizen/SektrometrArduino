/*
Simple PointToPoint Protocol
master/slave protocol for easy byte communication

Frame: 
0xf0 - Write Command
0xaa - Read Command
0x55 - Response

Write:
0xf0 0x{adr} 0x{size} {... bytes ...} 0x{ CS1 } 0x{ CS2 }

Response:
0x0f 0x{size} 0x{ CS1 } 0x{ CS2 }

Read:
0xaa 0x{adr} 0x{size} 0x{ CS1 } 0x{ CS2 }

Response:
0x55 0x{size} {... bytes ...} 0x{ CS1 } 0x{ CS2 }
size = 0 when any error

*/
#include <Arduino.h>

class SPTPP {
    public:
        SPTPP(uint8_t* obj, int size, unsigned long timeout);
        void GetedByte(uint8_t b);
    private:
        uint8_t* obj;
        int size, pos, actSize;
        uint8_t* buffer;
        uint8_t* writeBuffer;
        unsigned long lastTime, timeout;

        int CheckFletcher16( uint8_t *data, int count );
        uint8_t* Fletcher16( uint8_t *data, int count );
        void RunRead();
        void RunWrite();
        void CheckNextBuffPosition();
        void SetNewBuffer(int p);
        void SendReadError();
        void SendWriteError();
};
