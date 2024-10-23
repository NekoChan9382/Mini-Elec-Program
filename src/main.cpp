#include <mbed.h>
#include <cstdint>
#include <string>
#include <PID_new.hpp>

CAN can(PA_11, PA_12, 1e6);
CANMessage msg;
CANMessage msg_read;
BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial esp(PA_9, PA_10, 115200);
int16_t output[4] = {0}; // 0: left, 1: right, 2: extract
Pid pid({{150.0, 3.0, 0.01}, 20000, -20000});
int max_speed = 24000;

int stick_output_left[9] =  { 0, -max_speed,       -max_speed, -max_speed * 0.5,-max_speed, max_speed * 0.5,  max_speed, max_speed      , max_speed };
int stick_output_right[9] = { 0, -max_speed * 0.5, -max_speed, -max_speed      , max_speed, max_speed      ,  max_speed, max_speed * 0.5, -max_speed };

string readlines(BufferedSerial &serial, bool is_only_number = false)
{
    int i = 0;        // 繰り返し変数
    char buff = '0';  // シリアル受信
    string data = ""; // 受信データ保存

    while (buff != '\n' and i < 8)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信

        data += buff; // 受信データ保存
        i++;
        printf("%c", buff);
        if(is_only_number){
            if( (buff < '0' || buff > '9') && buff != '\n'){
                printf("error\n");
                return "";
            }
        }
    }
    // printf("%s\n", data); //受信データ表示
    printf("end\n");
    return data;
}

int main()
{
    int slider = 0;
    int stickXY = 0;
    int goal = 0;
    int deg = 0;

    printf("main\n");
    while (1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        if (esp.readable())
        {
            string data = readlines(esp);
            // int i = 0;          // 繰り返し変数
            // char buff = '0';    // シリアル受信
            // char data[10] = ""; // 受信データ保存

            // while (buff != '\n' and i < 10)
            // {
            //     esp.read(&buff, sizeof(buff)); // シリアル受信

            //     data[i] = buff; // 受信データ保存
            //     i++;
            //     // printf("%c", buff);
            // }

            if (data == "stick\n")
            {
                // printf("stick!!\n");

                string datas = readlines(esp, true);
                if(datas == ""){
                    break;
                }
                // printf("%s\n", datas);
                stickXY = stoi(datas);
                printf("%d\n", stickXY);
                
                output[0] = stick_output_left[stickXY] * -1;
                output[1] = stick_output_right[stickXY];
                
            }
            if (data == "slider\n")
            {
                printf("slider!!\n");

                // string datas = readlines(esp);
                int j = 0;          // 繰り返し変数
                char buff = '0';    // シリアル受信
                char datas[8] = ""; // 受信データ保存

                while (buff != '\n' and j < 8)
                {
                    esp.read(&buff, sizeof(buff)); // シリアル受信

                    datas[j] = buff; // 受信データ保存
                    j++;
                    printf("%c", buff);
                }
                slider = atoi(datas);
                goal = slider * 30;
            }
        }
        if (can.read(msg_read); msg_read.id == 10)
        {
            int16_t enc = (msg_read.data[7] << 8 | msg_read.data[6]);
            const float k = 360.0 / (250.0 * 2.0);
            deg = enc * k;
        }
        // printf("stick: %d, %d slider: %d\n", output[0], output[1], slider);

        if (now - pre >= 10ms)
        {
            pre = now;
            output[2] = pid.calc(goal, deg, 0.01);
            // printf("goal: %d, deg: %d, output: %d\n", goal, deg, output[2]);
            CANMessage msg(3, (const uint8_t *)output, 8);
            can.write(msg);
        }
    }
}