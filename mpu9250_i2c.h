#ifndef _MPU9250_I2C_H_
#define _MPU9250_I2C_H_

#define MPU9250_SLAVE_ADDR_LOW (0b1101000 << 1)//AD0 == LOW
#define MPU9250_SLAVE_ADDR_HIGH (0b1101001 << 1)//AD0 == HIGH
#define MPU9250_MAG_ADDR (0b0001100 << 1)

#define MPU9250_WRITE_FLAG 0b00000000
#define MPU9250_READ_FLAG  0b00000001
#define MPU9250_CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR 0x1E
#define INT_PIN_CFG 0x37
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCLE_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define WHO_AM_I_MPU9250 0x75 //0x71ならおｋ
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x79
#define YA_OFFERT_L 0x80
#define ZA_OFFSET_H 0x81
#define ZA_OFFSET_L 0x82

#define MPU9250_WIA 0x00 //device ID
#define MPU9250_INFO 0x01
#define MPU9250_ST1 0x02
#define HXL 0x03//Low -> Highの順に注意
#define MPU9250_HXH 0x04
#define MPU9250_HYL 0x05
#define MPU9250_HYH 0x06
#define MPU9250_HZL 0x07
#define MPU9250_HZH 0x08
#define MPU9250_ST2 0x09
#define MPU9250_CNTL1 0x0A
#define MPU9250_CNTL2 0x0B

#define ACC_LSB (0.0000610350f)//[G / LSB]
#define GYRO_LSB (0.007630f) //[(degree / s) / LSB]
#define MPU9250_MAG_LSB (0.150f) //[uT / LSB]

typedef enum AD0 {
    AD0_HIGH = 1,
    AD0_LOW = 0
} ad0;

typedef enum ACC_RANGE {
    _2G = 1,
    _4G = 2,
    _8G = 4,
    _16G = 8
} acc_range;

typedef enum GYRO_RANGE {
    _250DPS = 1,
    _500DPS = 2,
    _1000DPS = 4,
    _2000DPS = 8
} gyro_range;

typedef enum MAG_RATE {
    _8HZ = 0,
    _100HZ = 1
} mag_rate;

typedef enum A_BAND_WIDTH {
    NO_USE = 0b00000000,
    _460HZ = 0b00001000,
    _184HZ = 0b00001001,
    _92HZ = 0b00001010,
    _41HZ = 0b00001011,
    _20HZ = 0b00001100,
    _10HZ = 0b00001101,
    _5HZ = 0b00001110,
} a_band_width;

/**
*  @bref  mpu9250を比較的簡単に利用できるようにしたライブラリ
*  @note  ローパスフィルタまわりの実装がまだです． 内部のAK8963はスレーブアドレス固定なので，複数台並列では使用できません．
*  @author  Gaku MATSUMOTO
*/
class mpu9250
{

public:

    /**
    *  @bref  mpu9250インスタンスを生成する
    *  @param  _i2c  メインプログラムで宣言したI2Cインスタンスのアドレス
    *  @param  celect  AD0ピンがHIGHならAD0_HIGH，LOWならAD0_LOW
    *  @note  第二引数なしだとAD0_HIGHになります．
    */
    mpu9250(I2C& _i2c, AD0 celect = AD0_HIGH);

    I2C* _nine;
public:
    void writeReg(char addr, char data);
    void writeReg(char addr, char reg, char data);
    char readReg(char addr, char reg);
    void readReg(char addr, char start_reg, char* buff, char num);

    /*!
     @bref  慣性センサと通信ができているか確認する
     @note  trueが返ってきたら成功，falseなら．．．
     */
    bool sensorTest();

    /**
    * @fn bool mpu9250::mag_senserTest()
    * @bref 地磁気センサと通信ができているか確認する
    * @note trueが返ってきたら成功，falseなら．．．
    */
    bool mag_sensorTest();

    /**
     *    @bref  加速度センサのレンジを設定
     *    @param  a_range _2G, _4G, _8G, _16Gの中から選択
     *    @note  引数無しで±4Gになる
     */
    void setAcc(ACC_RANGE a_range = _4G);

    /**
     *    @bref  角速度センサのレンジ設定
     *    @param  g_range _250DPS, _500DPS, _1000DPS, _2000DPSの中から選択
     *    @note  引数無しで±500DPS
     */
    void setGyro(GYRO_RANGE g_range = _500DPS);

    /**
     *    @bref  地磁気センサのデータレート設定
     *    @param  rate  _8HZ か _100HZを選択
     *    @note  あえて8Hzにする必要は無いと思います．
     */
     //void setMag(MAG_RATE rate = _100HZ);

private:
    void init();

public:
    /**
     *    @bref   I2Cの通信速度を変更できます．余程のことがない限り使用しなくていいです・
     */
    void frequency(int Hz);

    /**
     *    @bref  mpu9250のデジタルローパスフィルタの設定
     *    @param  band  NO_USE, _460HZ, _184HZ, _92HZ, _41HZ, _20HZ, _10HZ, _5HZから選択
     *    @note  カットオフ周波数なのかサンプルレートなのかよく分かりません．正直効果が見られません
     */
    void setAccLPF(A_BAND_WIDTH band);

    /**
     *    @bref  ゼロ点のずれを補正するオフセット値を設定する
     *    @param  ax,ay,az    加速度のオフセット
     *    @param  gx,gy,gz    角速度のオフセット
     *    @param  mx,my,mz    地磁気のオフセット
     *    @note  とても重要です．地磁気は定期的にキャリブレーションをしてください．ちなみに，これらの値は測定値より引かれています．
     */
    void setOffset(float gx = 0.0f, float gy = 0.0f, float gz = 0.0f,
        float ax = 0.0f, float ay = 0.0f, float az = 0.0f,
        float mx = 0.0f, float my = 0.0f, float mz = 0.0f);

    /**
     *    @bref  加速度を取得します．
     *    @param  ax  x軸方向の加速度[G]
     *    @param  ay  y軸方向の加速度[G]
     *    @param  az  z軸方向の加速度[G]
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getAcc(T* ax, T* ay, T* az);

    /**
     *    @bref  加速度を取得します．
     *    @param  acc  各軸方向の加速度[G]，x,y,zの順
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getAcc(T* acc);

    /**
     *    @bref  角速度を取得します．
     *    @param  gx  x軸方向の角速度[degree/s]
     *    @param  gy  y軸方向の角速度[degree/s]
     *    @param  gz  z軸方向の角速度[degree/s]
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getGyro(T* gx, T* gy, T* gz);

    /**
     *    @bref  角速度を取得します．
     *    @param  gyro  各軸方向の角速度[degree/s], x,y,zの順
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getGyro(T* gyro);

    /**
     *    @bref  磁束密度を取得します．
     *    @param  mx  x軸方向の磁束密度[uT]
     *    @param  my  y軸方向の磁束密度[uT]
     *    @param  mz  z軸方向の磁束密度[uT]
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getMag(T* mx, T* my, T* mz);


    /**
     *    @bref  磁束密度を取得します．
     *    @param  mag  各軸方向の磁束密度[uT]，x,y,zの順
     *    @note  型はfloat でも doubleでも構いません．
     */
    template<typename T>void getMag(T* mag);

    /**
     *    @bref  角速度と加速度を同時に取得します．
     *    @param  imu データを入れる配列，角速度[degree/s],加速度[G]の順
     *    @note  配列数は6以上で
     */
    template<typename T>void getGyroAcc(T* imu);//gx,gy,gz,ax,ay,az

    /**
     *    @bref  角速度と加速度と地磁気を同時に取得します．
     *    @param imu 角速度・加速度データを入れる配列，角速度[degree/s],加速度[G]の順
     *    @param mag 地磁気データを入れる配列
     *    @note  imuは6つ，magは3つ以上で
     */
    template<typename T>void getAll(T* imu, T* mag);

    /**
     *    @bref 角速度と加速度と地磁気を同時に取得します．
     *    @param all 角速度・加速度・地磁気データを入れる配列，角速度[degree/s],加速度[G],地磁気[uT]の順
     *    @note  要素数は9つ以上
     */
    template<typename T>void getAll(T* all);

private:
    char _addr;
    float acc_coef;//coefficient
    float gyro_coef;
    float mag_coef;
    float acc_offset[3];
    float gyro_offset[3];
    float mag_offset[3];
};



inline void mpu9250::writeReg(char addr, char data)
{
    _nine->write(addr | MPU9250_WRITE_FLAG, &data, 1, false);
}
inline void mpu9250::writeReg(char addr, char reg, char data)
{
    char temp[2] = { reg, data };
    _nine->write(addr | MPU9250_WRITE_FLAG, temp, 2, false);
}
inline char mpu9250::readReg(char addr, char reg)
{
    char buff[1];
    writeReg(addr, reg);
    _nine->read(addr | MPU9250_READ_FLAG, buff, 1, true);
    return buff[0];
}
inline void mpu9250::readReg(char addr, char start_reg, char* buff, char num)
{
    writeReg(addr, start_reg);
    _nine->read(addr | MPU9250_READ_FLAG, buff, num, true);
}

#endif