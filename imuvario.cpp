#include "sti_gnss_lib.h"
#include "GNSS.h"
#include <math.h>
#include <Timer.h>
#include "common.h"
#include "spi.h"
#include "MS5611f.h"
#include "KalmanFilter.h"
#include "Beeper.h"
#include "max21100.h"
#include "hmc5883l.h"
#include "elapsedtime.h"
#include "MLR.h"
#include "SpiFlash.h"
#include "Log.h"
#include "IMU.h"
#include "GPS.h"
#include "Buttons.h"


MS5611      ms5611;
KalmanFilter kalman;
Beeper      audio;
MAX21100    max;
MLR         mlr;
Log         trklog;
SpiFlash    flash;

char gszBuf[120];
int ledState = 0;
float fgx,fgy,fgz;
float fax,fay,faz;
float fmx,fmy,fmz;
S16 gx,gy,gz;
S16 ax,ay,az;
S16 mx,my,mz;
S16 temp;

#define Z_VARIANCE		    200.0f
#define ZACCEL_VARIANCE	    100.0f
#define ZACCELBIAS_VARIANCE 1.0f

float zTrack, vTrack; // kalman filter output
float vIIR; // IIR filtered climbrate for xcsoar display
int   numAccelSamples; // count number of acceleration samples between altitude samples
float avgAccel;  // multiple accel readings per altitude sample, so use average
float lpfAvgAccel;  // low pass filtered avgAccel
float baroElapsedTimeSecs; // time interval between altitude samples
float kfElapsedTimeSecs; // time interval between Kalman filter iterations
float lk8ex1ElapsedTimeSecs; // time interval between lk8ex1 transmits
float ggaElapsedTimeSecs; // time interval between gga transmits
float vtgElapsedTimeSecs; // time interval between gprmc transmits
float ptgaElapsedTimeSecs; // time interval between $PTGA transmits

void PinConfig(void);
void TrklogConfig(void);
void IMUGetRawData(void);
void CheckMLRMode(void);
void GPSVarioHandler(float loopElapsedTimeSecs);
void SendPTGASentence(void);
void SendLK8EX1Sentence(void);

int gbAudio = 0;

#define MODE_GPSVARIO   333
#define MODE_MLRTRACK   444

int mode = MODE_GPSVARIO;


void setup() {
    GPS_GNSSConfig();
    PinConfig();
    CheckMLRMode();

    if (mode == MODE_MLRTRACK) {
        mlr.Config();
        }
    else {
        TrklogConfig();
        Serial.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
        Serial.begin(115200);

        delay(1000);

        Serial.print("\r\nPataga GPSVario v 0.84\r\n");
        Serial.print(gszBuf);

        U08 regval = max.Read8(MAX21100_WHO_AM_I);
        sprintf(gszBuf,"MAX21100 Whoami 0x%02x\r\n", regval);
        Serial.print(gszBuf);
        U16 flashID = flash.ReadID();
        sprintf(gszBuf,"Flash ID %04X Free Address %d\r\n",flashID, (int)trklog.addr_);
        Serial.print(gszBuf);

        ms5611.Configure();
        ms5611.AveragedSample(4);
        sprintf(gszBuf,"Altitude %fm Temperature %dC\r\n", ms5611.zCmAvg_/100.0f, (int)ms5611.celsiusSample_);
        Serial.print(gszBuf);
        audio.SetFrequency(0);
        max.ConfigureMaster();

        kalman.Configure(Z_VARIANCE, ZACCEL_VARIANCE, ZACCELBIAS_VARIANCE, ms5611.zCmAvg_,0.0f,0.0f);

        ms5611.InitializeSampleStateMachine();
        numAccelSamples = 0;
        baroElapsedTimeSecs = 0.0f;
        ptgaElapsedTimeSecs = 0.0f;
        kfElapsedTimeSecs = 0.0f;
        vtgElapsedTimeSecs = 0.0f;
        ggaElapsedTimeSecs = 0.0f;
        lk8ex1ElapsedTimeSecs = 0.0f;
        avgAccel = 0.0f;
        lpfAvgAccel = 0.0f;
        vIIR = 0.0f;
        dsu_SetTimeOrigin();
        }   
    }


void loop() {
    float loopElapsedTimeSecs = dsu_ElapsedTimeSeconds();

    switch (mode) {
        case MODE_MLRTRACK : 
        if (loopElapsedTimeSecs >= 0.010f) {
            dsu_SetTimeOrigin();
            mlr.EventHandler();
            }
        break;

        case MODE_GPSVARIO :
        default :
        if (loopElapsedTimeSecs >= 0.004f) {
            dsu_SetTimeOrigin();
            GPSVarioHandler(loopElapsedTimeSecs);
            }
        break;
        }
    }


void GPSVarioHandler(float loopElapsedTimeSecs) {
    int bUseAccel = 0;
    U08 status = max.Read8(MAX21100_SYSTEM_STATUS);
    if (status) {
        IMUGetRawData();
        if ((status & 0x01) && !(status & 0x02) ) {
            max.GetGyroValues(gx,gy,gz,&fgx, &fgy, &fgz);
            }
        if ((status & 0x04) && !(status & 0x08) ) { 
            max.GetAccelValues(ax,ay,az,&fax, &fay, &faz);
            float a = sqrt(fax*fax + fay*fay + faz*faz); 
            bUseAccel = ((a > 0.6f) && (a < 1.4f)) ? 1 : 0;
            }
        if ((status & 0x10) && !(status & 0x20) ) {
            max.GetMagValues(mx,my,mz, &fmx, &fmy, &fmz);
            }
        imu_MadgwickQuaternionUpdate(bUseAccel, loopElapsedTimeSecs,fax,fay,faz,fgx*PI_DIV_180,fgy*PI_DIV_180,fgz*PI_DIV_180,fmx,fmy,fmz);
        float compensatedAccel = 980.0f*imu_GravityCompensatedAccel(fax,fay,faz,quat);
        avgAccel += compensatedAccel;
        numAccelSamples++;
        }
    baroElapsedTimeSecs += loopElapsedTimeSecs;
    kfElapsedTimeSecs += loopElapsedTimeSecs;
    ggaElapsedTimeSecs += loopElapsedTimeSecs;
    ptgaElapsedTimeSecs += loopElapsedTimeSecs;
    lk8ex1ElapsedTimeSecs += loopElapsedTimeSecs;
    vtgElapsedTimeSecs += loopElapsedTimeSecs;

    if (baroElapsedTimeSecs >= 0.012f) {
        baroElapsedTimeSecs = 0.0f;
        ms5611.SampleStateMachine();
        if (ms5611.zGood) {
            avgAccel /= numAccelSamples;
            //lpfAvgAccel = 0.8f*lpfAvgAccel + 0.2f*avgAccel;
	    kalman.Update(ms5611.zCmSample_, avgAccel, kfElapsedTimeSecs, &zTrack, &vTrack);
            vIIR = 0.95f*vIIR + 0.05f*vTrack;
            if (gbAudio) audio.Beep((int)vTrack);
	    ms5611.zGood = 0;
            kfElapsedTimeSecs = 0.0f;
            avgAccel = 0.0f;
            numAccelSamples = 0;
            }
        else {
            Btn_Check();
            if (ggaElapsedTimeSecs >= 0.5f){
                ggaElapsedTimeSecs = 0.0f;
                GPS_ProcessData();
		GPS_TransmitGPGGA();
                }
            else
            if (vtgElapsedTimeSecs >= 0.5f){
                vtgElapsedTimeSecs = 0.0f;
                GPS_TransmitGPVTG();		
		}
            else
#ifdef PATAGA
            if (ptgaElapsedTimeSecs >= 0.5f){
                ptgaElapsedTimeSecs = 0.0f;
                SendPTGASentence();		
		}
#endif
#ifdef LK8EX1
            if (lk8ex1ElapsedTimeSecs >= 0.5f){
                lk8ex1ElapsedTimeSecs = 0.0f;
                SendLK8EX1Sentence();
                }
#endif
            }
        }            
    }



void task_called_after_GNSS_update(void) {
    if (gbBtnLPressed) {
        gbBtnLPressed = 0;
        }
    if (gbBtnRPressed) {
        gbBtnRPressed = 0;
        gbAudio = !gbAudio;
        }
    }


void IMUGetRawData(void) {
    U08 buf[20];
    max.ReadBuf(MAX21100_GYRO_X_H, AUTO_INCR,20,buf);
    gx = (S16)((((U16)buf[0]) << 8) | (U16)buf[1]);
    gy = (S16)((((U16)buf[2]) << 8) | (U16)buf[3]);
    gz = (S16)((((U16)buf[4]) << 8) | (U16)buf[5]);
    ax = (S16)((((U16)buf[6]) << 8) | (U16)buf[7]);
    ay = (S16)((((U16)buf[8]) << 8) | (U16)buf[9]);
    az = (S16)((((U16)buf[10]) << 8) | (U16)buf[11]);
    mx = (S16)((((U16)buf[12]) << 8) | (U16)buf[13]);
    my = (S16)((((U16)buf[14]) << 8) | (U16)buf[15]);
    mz = (S16)((((U16)buf[16]) << 8) | (U16)buf[17]);
    temp = (S16)((((U16)buf[18]) << 8) | (U16)buf[19]);
    }

void PinConfig(void) {
    pinMode(GPIO0_LED, OUTPUT);
    LED_OFF();
    analogADCClock(ADC_IN2,500000);
    pinMode(CSF_PIN, OUTPUT);
    CSF(1);
    pinMode(CSA_PIN, OUTPUT);
    CSA(1);
    pinMode(CSB_PIN, OUTPUT);
    CSB(1);
    pinMode(BTN_L_PIN, INPUT);
    pinMode(BTN_R_PIN, INPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);

    }


// Enter MLR track upload mode
// green button pressed while powering up
// confirmation : 4 brief led flashes
void CheckMLRMode(void) {
    if (digitalRead(BTN_R_PIN) == 0) {
        delay(100);
        if (digitalRead(BTN_R_PIN) == 0) {
            mode = MODE_MLRTRACK;
            for (int cnt = 0;cnt < 4; cnt++) {
                LED_ON();  
                delay(20); 
                LED_OFF();
                delay(500);
                }
            }
        }   
    }


// Erase all tracks
// red button pressed while powering up, then wait for led, immediately press green button and hold.
// confirmation : rapid flickering of led

void TrklogConfig(void) {
    flash.GlobalUnprotect();
    if (digitalRead(BTN_L_PIN) == 0) {  // red button
        delay(100);
        if (digitalRead(BTN_L_PIN) == 0) { // red button
            LED_ON();
            delay(500);
            delay(500);
            delay(500);
            delay(500);
            if (digitalRead(BTN_R_PIN) == 0) { // green button
                for (int cnt = 0;cnt < 20; cnt++) {
                   LED_ON();  delay(50); LED_OFF();delay(50);
                    }
                trklog.EraseTracks();
                for (int cnt = 0;cnt < 20; cnt++) {
                   LED_ON();  delay(50); LED_OFF();delay(50);
                    }
                }
            }
        }   
    trklog.GetFreeAddress();
    }


void SendPTGASentence(void) {
    //static int bvindex = 0;
    U16 adcVal = analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal /= 4;
    //sprintf(gszBuf,"ADC reading %d\r\n", (int)adcVal);
    //Serial.print(gszBuf);

    int batteryVoltage = (int)((float(adcVal)*3.0f*1.776f/1023.0f) *100.0f  + 0.5f);
    //bvindex = (bvindex+1)%10;
    //int batteryVoltage = 370+bvindex*10;
    float gr = glideRatio*10.0;
    if (gr >= 0.0) gr += 0.5;
    else gr -= 0.5;

	int altitudeM = (50+(int)zTrack)/100;
	sprintf(gszBuf,"$PTGA,%d,%d,%d,%d*00\r\n", altitudeM, (int)vIIR, (int)gr, batteryVoltage);
	//sprintf(gszBuf,"$PTGA,%d,%d,%d,%d,%d*00\r\n", altitudeM, (int)vIIR, glideRatio, batteryVoltage);
    GPS_NMEAChecksum(gszBuf);
	Serial.print(gszBuf);
    }

void SendLK8EX1Sentence(void) {
    U16 adcVal = analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal += analogRead(ADC_IN2);
    adcVal /= 4;
    float batteryVoltage = (float(adcVal)*3.0f*1.776f)/1023.0f;
 
    int altitudeM = (50+(int)zTrack)/100;
    sprintf(gszBuf,"$LK8EX1,%d,%d,%d,%d,%1.2f*00\r\n", (int)ms5611.paSample_, altitudeM, (int)vIIR, (int)ms5611.celsiusSample_, batteryVoltage);

    GPS_NMEAChecksum(gszBuf);
    Serial.print(gszBuf);
    }

