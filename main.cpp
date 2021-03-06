#include "mbed.h"
#include "mpu9250_i2c.h"

#include "Quaternion.hpp"
#include "MadgwickFilter.hpp"

I2C i2cBus(p9, p10);
mpu9250 nine(i2cBus, AD0_LOW);
RawSerial pc(USBTX, USBRX, 921600);

PwmOut m1(p21);
PwmOut m2(p22);
PwmOut m3(p23);
PwmOut m4(p24);

const double DEG_TO_RAD = 0.01745329251994329576923690768489f;//PI / 180.0;

volatile bool sendFlag = false;

const double ACC_LPF_COEF = 0.9;
const double GYRO_LPF_COEF = 0.8;
const double MAG_LPF_COEF = 0.9;

const double defaultSpeed = 0.5f;
const double adustNum = 2;

//varialbe to get offset
float O_Jyro[3], O_Acce[3], O_Magn[3];
double Jyro[3] = { 0, 0, 0 };
double Acce[3] = { 0, 0, 0 };
double Magn[3] = { 0, 0, 0 };
int count;
uint8_t stuck;

void setOffset() {
     //get offset
    while (count < 100) {
        ++count;
        nine.getGyro(Jyro);
        nine.getAcc(Acce);
        nine.getMag(Magn);

        for (int cnt = 0; cnt < 3; cnt++) {
            O_Jyro[cnt] += Jyro[cnt];
            O_Acce[cnt] += Acce[cnt];
            O_Magn[cnt] += Magn[cnt];
        }
        wait_us(1000);
    }
    for (int cnt = 0; cnt < 3; cnt++) {
        O_Jyro[cnt] /= 100;
        O_Acce[cnt] /= 100;
        O_Magn[cnt] /= 100;
    }

    //set offset
    nine.setOffset(O_Jyro[0], O_Jyro[1], O_Jyro[2],
        O_Acce[0], O_Acce[1], O_Acce[2],
        O_Magn[0], O_Magn[1], O_Magn[2]);
}

void keepDig(double x, double y, double z, double w, char direction){
    switch (direction)
    {
    case 'w':
        m1 = defaultSpeed + x / adustNum + 0.1f;
        m2 = defaultSpeed + x / adustNum - 0.1f;
        m3 = defaultSpeed + y / adustNum;
        m4 = defaultSpeed + y / adustNum;
        break;
    
    case 'a':
        m1 = defaultSpeed + x / adustNum;
        m2 = defaultSpeed + x / adustNum;
        m3 = defaultSpeed + y / adustNum + 0.1f;
        m4 = defaultSpeed + y / adustNum - 0.1f;
        break;
    
    case 's':
        m1 = defaultSpeed + x / adustNum;
        m2 = defaultSpeed + x / adustNum;
        m3 = defaultSpeed + y / adustNum - 0.1f;
        m4 = defaultSpeed + y / adustNum + 0.1f;
        break;
    
    case 'd':
        m1 = defaultSpeed + x / adustNum - 0.1f;
        m2 = defaultSpeed + x / adustNum + 0.1f;
        m3 = defaultSpeed + y / adustNum;
        m4 = defaultSpeed + y / adustNum;
        break;
    
    default:
        m1 = defaultSpeed + x / adustNum;
        m2 = defaultSpeed + x / adustNum;
        m3 = defaultSpeed + y / adustNum;
        m4 = defaultSpeed + y / adustNum;
        break;
    }
}

int main() {

    double imu[2][6] = { 0 };
    double mag[2][3] = { 0 };
    double accLPF[3] = { 0 };
    double gyroLPF[3] = { 0 };
    double magLPF[3] = { 0 };

    float dQX, dQY, dQZ, dQW;

    //initial setting
    nine.setAccLPF(_460HZ);
    nine.setGyro(_1000DPS);
    nine.setAcc(_16G);

    setOffset();

    //madgwick filter timer start
    MadgwickFilter attitude(0.05);

    Quaternion currentQ;
    Quaternion defaultQ;

    attitude.MadgwickAHRSupdate(O_Jyro[0], O_Jyro[1], O_Jyro[2], O_Acce[0], O_Acce[1], O_Acce[2], O_Magn[0], O_Magn[1], O_Magn[2]); 
    attitude.getAttitude(&defaultQ);

    while (1) {
        //get angle speed, acceralator, magnetic flux density
        nine.getGyroAcc(imu[1]);
        nine.getMag(mag[1]);
        for (int i = 0; i < 3; i++) {
            gyroLPF[i] = GYRO_LPF_COEF * imu[0][i] + (1 - GYRO_LPF_COEF) * imu[1][i];
            accLPF[i] = ACC_LPF_COEF * imu[0][i + 3] + (1 - ACC_LPF_COEF) * imu[1][i + 3];
            magLPF[i] = MAG_LPF_COEF * mag[0][i] + (1 - MAG_LPF_COEF) * mag[1][i];

            imu[0][i] = imu[1][i];
            imu[0][i + 3] = imu[1][i + 3];
            mag[0][i] = mag[1][i];

            gyroLPF[i] *= DEG_TO_RAD;
        }

        //reload
        attitude.MadgwickAHRSupdate(gyroLPF[0], gyroLPF[1], gyroLPF[2], accLPF[0], accLPF[1], accLPF[2], magLPF[0], magLPF[1], magLPF[2]);
        sendFlag = false;

        //get attitude with Quaternion
        
        attitude.getAttitude(&currentQ);


        dQX = currentQ.x - defaultQ.x;
        dQY = currentQ.y - defaultQ.y;
        dQZ = currentQ.z - defaultQ.z;
        dQW = currentQ.w - defaultQ.w;

        //draw for unity
        //pc.printf("x%d\r\ny%d\r\nz%d\r\nw%d\r\n", (int)(dQX*1000000), (int)(dQY*1000000), (int)(dQZ*1000000), (int)(dQW*1000000)); //for unity

        //Debug
        //pc.printf("imu:: %f %f %f\r\n", imu[1][0], imu[1][1], imu[1][2]);
        //pc.printf("gyro:: %f %f %f\r\n", gyroLPF[0], gyroLPF[1], gyroLPF[2]);
        //pc.printf("acce:: %f %f %f\r\n", accLPF[0], accLPF[1], accLPF[2]);
        //pc.printf("magn:: %f %f %f\r\n", mag[0], magLPF[1], magLPF[2]);
        //pc.printf("%f,%f,%f,%f\r\n\n", attitude.q1, attitude.q2, attitude.q3, attitude.q0);
        pc.printf("%f,%f,%f,%f\r\n", dQX, dQY, dQZ, dQW);
        pc.printf("\r\n");

        wait_us(1000);
    }
}