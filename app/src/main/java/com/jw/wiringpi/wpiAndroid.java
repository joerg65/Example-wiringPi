package com.jw.wiringpi;

/**
 * Created by joerg on 16.09.17.
 */

public final class wpiAndroid {
    static {
        System.loadLibrary("wpi_android");
    }
    static public native int wiringPiSetup();
    static public native int wiringPiSetupSys();
    static public native void digitalWrite(int port, int onoff);
    static public native int digitalRead(int port);
    static public native void pullUpDnControl(int port, int pud);
    static public native void pinMode(int port, int mode);
    static public native int softPwmCreate(int port, int value, int range);
    static public native void softPwmWrite  (int port, int value) ;
    static public native void softPwmStop   (int port) ;
    static public native int analogRead(int port);
    static public native int openWeatherBoard();
    static public native int closeWeatherBoard();
    static public native void readyData();
    static public native int getUVindex();
    static public native float getVisible();
    static public native float getIR();
    static public native int getTemperature();
    static public native int getPressure();
    static public native int getHumidity();
    static public native int getAltitude();

    static public native int serialOpen(String device, int baud);
    static public native void serialClose(int fd);
    static public native void serialFlush(int fd);
    static public native void serialPutchar(int fd, short c);
    static public native void serialPuts(int fd, String s);
    static public native int serialDataAvail(int fd);
    static public native int serialGetchar(int fd);
}
