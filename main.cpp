#include "mbed.h"
#include "mpu9250_i2c.h"

#include "Quaternion.hpp"
#include "MadgwickFilter.hpp"

I2C i2cBus(p9, p10);
mpu9250 nine(i2cBus, AD0_LOW);
RawSerial pc(USBTX, USBRX, 921600);

const double DEG_TO_RAD = 0.01745329251994329576923690768489f;//PI / 180.0;

volatile bool sendFlag = false;

const double ACC_LPF_COEF = 0.9;
const double GYRO_LPF_COEF = 0.8;
const double MAG_LPF_COEF = 0.9;

//�I�t�Z�b�g�擾�ϐ�
float O_Jyro[3], O_Acce[3], O_Magn[3];
double Jyro[3] = { 0, 0, 0 };
double Acce[3] = { 0, 0, 0 };
double Magn[3] = { 0, 0, 0 };
int count;
uint8_t stuck;

void OffsetReload(bool enable) {
    if (enable && stuck < 50) {
        ++stuck;
    }
    if (enable && stuck >= 50) {
        stuck = 0;
        count = 0;

        while (count < 50)
        {
            ++count;
            nine.getGyro(Jyro);
            nine.getAcc(Acce);
            nine.getMag(Magn);

            for (int cnt = 0; cnt < 3; cnt++) {
                O_Jyro[cnt] += Jyro[cnt];
                O_Acce[cnt] += Acce[cnt];
                O_Magn[cnt] += Magn[cnt];
            }
            wait_us(100);
        }
        for (int cnt = 0; cnt < 3; cnt++) {
            O_Jyro[cnt] /= 50;
            O_Acce[cnt] /= 50;
            O_Magn[cnt] /= 50;
        }

        nine.setOffset(O_Jyro[0], O_Jyro[1], O_Jyro[2],
            O_Acce[0], O_Acce[1], O_Acce[2],
            O_Magn[0], O_Magn[1], O_Magn[2]);

    }
}

int main() {

    double imu[2][6] = { 0 };
    double mag[2][3] = { 0 };
    double accLPF[3] = { 0 };
    double gyroLPF[3] = { 0 };
    double magLPF[3] = { 0 };

    //�����ݒ�
    nine.setAccLPF(_460HZ);
    nine.setGyro(_1000DPS);
    nine.setAcc(_16G);

    //�I�t�Z�b�g�擾
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
        wait_us(100);
    }
    for (int cnt = 0; cnt < 3; cnt++) {
        O_Jyro[cnt] /= 100;
        O_Acce[cnt] /= 100;
        O_Magn[cnt] /= 100;
    }

    //�I�t�Z�b�g�ݒ�
    nine.setOffset(O_Jyro[0], O_Jyro[1], O_Jyro[2],
        O_Acce[0], O_Acce[1], O_Acce[2],
        O_Magn[0], O_Magn[1], O_Magn[2]);

    //madgwick filter timer �X�^�[�g
    MadgwickFilter attitude(0.05);

    while (1) {
        //�p���x�Ɖ����x�C�������x�f�[�^�̎擾
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

        //�X�V
        attitude.MadgwickAHRSupdate(gyroLPF[0], gyroLPF[1], gyroLPF[2], accLPF[0], accLPF[1], accLPF[2], magLPF[0], magLPF[1], magLPF[2]);
        sendFlag = false;

        //�p���擾 with Quaternion
        Quaternion myQ;
        attitude.getAttitude(&myQ);

        //�`�� for unity 
        pc.printf("x%d\r\ny%d\r\nz%d\r\nw%d\r\n", (int)(myQ.x*1000000), (int)(myQ.y*1000000), (int)(myQ.z*1000000), (int)(myQ.w*1000000)); //for unity


        //Debug
        //pc.printf("imu:: %f %f %f\r\n", imu[1][0], imu[1][1], imu[1][2]);
        //pc.printf("gyro:: %f %f %f\r\n", gyroLPF[0], gyroLPF[1], gyroLPF[2]);
        //pc.printf("acce:: %f %f %f\r\n", accLPF[0], accLPF[1], accLPF[2]);
        //pc.printf("magn:: %f %f %f\r\n", mag[0], magLPF[1], magLPF[2]);
        //pc.printf("%f,%f,%f,%f\r\n\n", attitude.q1, attitude.q2, attitude.q3, attitude.q0);

        wait_us(100);
    }
}
