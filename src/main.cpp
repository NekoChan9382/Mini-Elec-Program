#include <mbed.h>
#include <cstdint>
#include <string>
#include <PID_new.hpp>

CAN can(PA_11, PA_12, 1e6);
CANMessage msg;
CANMessage msg_read;
CANMessage msg_noid;
BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial esp(PA_9, PA_10, 115200);
int16_t output[4] = {0}; // 0: left, 1: right, 2: extract
uint8_t noid[8] = {0};
Pid pid({{150.0, 3.0, 0.01}, 20000, -20000});

DigitalOut led_eb(PC_3);
DigitalOut led_conn(PC_2);
DigitalOut eb_vcc(PC_1);
DigitalIn eb_gnd(PC_0);

int max_speed = 24000;
int extract_speed = 10000;
int trash_speed = 18000;
int arm_height[4] = {1500, 1000, 500, 0};
bool is_pid = false;
bool is_eb_push = true;
bool is_conn = false;

int stick_output_left[9] = {0, -max_speed, -max_speed, -max_speed * 0.5, -max_speed, max_speed * 0.5, max_speed, max_speed, max_speed};
int stick_output_right[9] = {0, -max_speed * 0.5, -max_speed, -max_speed, max_speed, max_speed, max_speed, max_speed * 0.5, -max_speed};

string readlines(BufferedSerial &serial, bool is_only_number = false)
{
    int i = 0;        // 繰り返し変数
    char buff = '0';  // シリアル受信
    string data = ""; // 受信データ保存

    while ((buff != '\n') and i < 10)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信

        if (buff != '\n' && buff != '\r')
        {
            data += buff; // 受信データ保存

            if (is_only_number)
            {

                if ((buff < '0' || buff > '9'))
                {
                    printf("error\n");
                    return "";
                }
            }
        }
        i++;
    }
    return data;
}

int main()
{
    int stickXY = 0;
    int goal = 0;
    int deg = 0;
    bool is_limit_push = 0;
    led_conn = 0;
    led_eb = 0;
    eb_vcc = 1;

    printf("main\n");
    while (1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        if (esp.readable())
        {
            string data = readlines(esp);

            if (data.compare("stick") == 0)
            {

                string datas = readlines(esp, true);
                if (datas == "")
                {
                    continue;
                }
                stickXY = stoi(datas);

                output[0] = stick_output_left[stickXY] * -1;
                output[1] = stick_output_right[stickXY];
            }
            else if (data.compare("armmove") == 0)
            {
                is_pid = false;
                pid.reset();

                string datas = readlines(esp, true);
                if (datas == "")
                {
                    continue;
                }
                switch (stoi(datas))
                {
                case 0:
                    output[2] = 0;
                    break;
                case 1:
                    output[2] = -extract_speed;
                    break;
                case 2:
                    output[2] = extract_speed;
                    break;
                }
            }
            else if (data.compare("trash") == 0)
            {
                is_pid = false;
                pid.reset();

                string datas = readlines(esp, true);
                if (datas == "")
                {
                    continue;
                }
                switch (stoi(datas))
                {
                case 0:
                    output[3] = 0;
                    break;
                case 1:
                    output[3] = extract_speed;
                    break;
                case 2:
                    output[3] = -extract_speed;
                    break;
                }
            }
            else if (data.compare("arm") == 0)
            {
                pid.reset();
                is_pid = true;

                string datas = readlines(esp, true);
                if (datas == "")
                {
                    continue;
                }
                goal = arm_height[stoi(datas)];
            }
            else if (data.compare("conn") == 0)
            {
                led_conn = 1;
            }
            else if (data.compare("disconn") == 0)
            {
                led_conn = 0;
            }
            else if (data.compare("new") == 0)
            {
                string datas = readlines(esp, true);
                if (datas == "")
                {
                    continue;
                }
                if (stoi(datas) == 1)
                {
                    noid[0] = 1;
                } else {
                    noid[0] = 0;
                }
            }
        }

        can.read(msg_read);

        if (msg_read.id == 10)
        {
            int16_t enc = (msg_read.data[7] << 8 | msg_read.data[6]);
            const float k = 360.0 / (250.0 * 2.0);
            deg = enc * k;
        }
        if (msg_read.id == 9)
        {
            uint8_t sw = msg_read.data[5];
            is_limit_push = sw;
        }
        // printf("stick: %d, %d slider: %d\n", output[0], output[1], slider);

        if (eb_gnd)
        {
            led_eb = 1;
        }
        else
        {
            led_eb = 0;
        }

        if (now - pre >= 10ms)
        {
            pre = now;
            if (is_pid) output[2] = pid.calc(goal, deg, 0.01) * -1;
            if (is_limit_push && output[2] > 0)
            {
                output[2] = 0;
            }
            
            // printf("goal: %d, deg: %d, output: %d\n", goal, deg, output[2]);
            CANMessage msg(3, (const uint8_t *)output, 8);
            CANMessage msg_noid(29, noid, 8);
            can.write(msg);
            can.write(msg_noid);
        }
    }
}