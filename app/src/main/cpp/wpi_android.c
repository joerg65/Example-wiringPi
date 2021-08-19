#include <jni.h>
#include <unistd.h>
#include <string.h>
#include <android/log.h>
#include <errno.h>
#include <wiringPi.h>
#include "bme280.h"
#include "bme280-i2c.h"
#include "si1132.h"
#include <softPwm.h>
#include <wiringSerial.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define LOG_TAG "wpi_android"

#define I2C_DEVICE  "/dev/i2c-1"

u32 pressure;
s32 temperature;
u32 humidity;

float SEALEVELPRESSURE_HPA = 1024.25;

jint Java_com_jw_wiringpi_wpiAndroid_analogRead(JNIEnv* env, jobject obj, jint port) {
    return analogRead(port);
}

void Java_com_jw_wiringpi_wpiAndroid_digitalWrite(JNIEnv* env, jobject obj, jint port, jint onoff) {
    digitalWrite(port, onoff);
}

jint Java_com_jw_wiringpi_wpiAndroid_digitalRead(JNIEnv* env, jobject obj, jint port) {
    return digitalRead(port);

}void Java_com_jw_wiringpi_wpiAndroid_pinMode(JNIEnv* env, jobject obj, jint port, jint value) {
    pinMode(port, value);
}

void Java_com_jw_wiringpi_wpiAndroid_pullUpDnControl(JNIEnv* env, jobject obj, jint port, jint pud) {
    return pullUpDnControl(port, pud);
}

jint Java_com_jw_wiringpi_wpiAndroid_wiringPiSetupSys(JNIEnv* env, jobject obj) {
    return wiringPiSetupSys();
}

jint Java_com_jw_wiringpi_wpiAndroid_wiringPiSetup(JNIEnv* env, jobject obj) {
    return wiringPiSetup();
}

jint Java_com_jw_wiringpi_wpiAndroid_openWeatherBoard(JNIEnv* env, jobject obj) {
    int result = -1;
    result = si1132_begin(I2C_DEVICE);
    if (result == -1)
        return result;
    result = bme280_begin(I2C_DEVICE);
    return result;
}

void Java_com_jw_wiringpi_wpiAndroid_closeWeatherBoard(JNIEnv* env, jobject obj) {
    bme280_end();
    si1132_end();
}

jint Java_com_jw_wiringpi_wpiAndroid_bme280_begin(JNIEnv* env, jobject obj) {
    return bme280_begin(I2C_DEVICE);
}

void Java_com_jw_wiringpi_wpiAndroid_bme280_end(JNIEnv* env, jobject obj) {
    bme280_end();
}

jint Java_com_jw_wiringpi_wpiAndroid_getUVindex(JNIEnv* env, jobject obj) {
    int result = 0;
    Si1132_readUV(&result);
    return result;
}

jfloat Java_com_jw_wiringpi_wpiAndroid_getVisible(JNIEnv* env, jobject obj) {
    float result = 0.0;
    Si1132_readVisible(&result);
    return result;
}

jfloat Java_com_jw_wiringpi_wpiAndroid_getIR(JNIEnv* env, jobject obj) {
    float result = 0.0;
    Si1132_readIR(&result);
    return result;
}

void Java_com_jw_wiringpi_wpiAndroid_readyData(JNIEnv* env, jobject obj) {
    bme280_read_pressure_temperature_humidity(&pressure, &temperature, &humidity);
}

jint Java_com_jw_wiringpi_wpiAndroid_getTemperature(JNIEnv* env, jobject obj) {
    return temperature;
}

jint Java_com_jw_wiringpi_wpiAndroid_getPressure(JNIEnv* env, jobject obj) {
    return pressure;
}

jint Java_com_jw_wiringpi_wpiAndroid_getHumidity(JNIEnv* env, jobject obj) {
    return humidity;
}

jint Java_com_jw_wiringpi_wpiAndroid_getAltitude(JNIEnv* env, jobject obj) {
    int result = 0;
    bme280_readAltitude(pressure, &SEALEVELPRESSURE_HPA, &result);
    return result;
}

jint Java_com_jw_wiringpi_wpiAndroid_softPwmCreate(JNIEnv* env, jobject obj, jint port, jint value, jint range) {
    return softPwmCreate(port, value, range);
}

void Java_com_jw_wiringpi_wpiAndroid_softPwmWrite(JNIEnv* env, jobject obj, jint port, jint value) {
    softPwmWrite(port, value);
}

void Java_com_jw_wiringpi_wpiAndroid_softPwmStop(JNIEnv* env, jobject obj, jint port) {
    softPwmStop(port);
}

jint Java_com_jw_wiringpi_wpiAndroid_serialOpen(JNIEnv* env, jobject obj, jstring device, jint baud) {
    const char *nativeString = (*env)->GetStringUTFChars(env, device, 0);
    int ret = serialOpen(nativeString, baud);
    if (ret < 0) LOGE("Unable to open serial device: %s", strerror (errno)) ;
    (*env)->ReleaseStringUTFChars(env, device, nativeString);
    return ret;
}

void Java_com_jw_wiringpi_wpiAndroid_serialClose(JNIEnv* env, jobject obj, jint fd) {
    serialClose(fd);
}

void Java_com_jw_wiringpi_wpiAndroid_serialFlush(JNIEnv* env, jobject obj, jint fd) {
    serialFlush(fd);
}

void Java_com_jw_wiringpi_wpiAndroid_serialPutchar(JNIEnv* env, jobject obj, jint fd, jbyte c) {
    serialPutchar(fd, c);
}

void Java_com_jw_wiringpi_wpiAndroid_serialPuts(JNIEnv* env, jobject obj, jint fd, jstring s) {
    const char *nativeString = (*env)->GetStringUTFChars(env, s, 0);
    serialPuts(fd, nativeString);
    (*env)->ReleaseStringUTFChars(env, s, nativeString);
}

jint Java_com_jw_wiringpi_wpiAndroid_serialDataAvail(JNIEnv* env, jobject obj, jint fd) {
    return serialDataAvail(fd);
}

jint Java_com_jw_wiringpi_wpiAndroid_serialGetchar(JNIEnv* env, jobject obj, jint fd) {
    return serialGetchar(fd);
}

#ifdef __cplusplus
}
#endif
