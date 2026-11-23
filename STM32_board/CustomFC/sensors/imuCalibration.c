#include "imuCalibration.h"
#include "i2c_IMU.h"
#include "accelFit.h"
#include "fcConfig.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

/* ---- Flash storage ---- */
#define calFlashSector   FLASH_SECTOR_7
#define calFlashAddress  0x08060000u
#define calMagic         0x43414C31u    // "CAL1"
#define calVersion       1u

typedef struct{
    uint32_t magic;
    uint32_t version;
    uint32_t accelCalibrated;
    uint32_t levelTrimmed;
    imuCorrection corr;
    uint32_t checksum;
} calRecord;

/* ---- Accel calibration ---- */
#define stillWindow       250       // reads per still check (~0.5 s at 2 ms)
#define stillGyroMax      0.15f     // [rad/s] faster than this = moving
#define stillAccelStdMax  0.02f     // [g] accel wobble allowed while "still"
#define minNewAngleCos    0.94f     // a new sample must point >20 deg away from all the others
#define faceThreshold     0.8f      // [g] counts as "this face down" (within ~37 deg)
#define minSamples        12
#define maxSamples        40

/* ---- Fit sanity limits ---- */
#define maxOffsetG        0.2f
#define minScale          0.9f
#define maxScale          1.1f
#define maxRmsG           0.02f

/* ---- Level trim ---- */
#define levelSamples      1000      // ~2 s
#define levelGyroMax      0.05f     // [rad/s]
#define maxTrimDeg        10.0f

#define radToDeg 57.29578f

// Which way the accel reads +1 g for each face. Level and upright reads (0, 0, -1 g)
static const char *faceNames[6] = {
    "nose up", "nose down", "left side down", "right side down", "upside down", "upright (level)"
};


static uint32_t checksum(const calRecord *rec){
    // FNV-1a over everything before the checksum
    const uint8_t *bytes = (const uint8_t *)rec;
    uint32_t h = 2166136261u;
    for(size_t i = 0; i < offsetof(calRecord, checksum); i++){
        h ^= bytes[i];
        h *= 16777619u;
    }
    return h;
}


static bool readRecord(calRecord *rec){
    memcpy(rec, (const void *)calFlashAddress, sizeof(calRecord));
    return rec->magic == calMagic && rec->version == calVersion && rec->checksum == checksum(rec);
}


static bool writeRecord(calRecord *rec){

    rec->magic = calMagic;
    rec->version = calVersion;
    rec->checksum = checksum(rec);

    printf("Saving to flash (takes a couple of seconds)...\r\n");

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = calFlashSector;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t badSector = 0;
    bool ok = HAL_FLASHEx_Erase(&erase, &badSector) == HAL_OK;

    const uint32_t *words = (const uint32_t *)rec;
    for(size_t i = 0; ok && i < (sizeof(calRecord) + 3) / 4; i++){
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, calFlashAddress + 4u * i, words[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();

    calRecord check;
    ok = ok && readRecord(&check) && memcmp(&check, rec, sizeof(calRecord)) == 0;
    printf(ok ? "Saved.\r\n" : "Flash write FAILED - calibration not saved\r\n");
    return ok;
}


static imuCorrection identity(void){
    imuCorrection c;
    c.accelOffset = (vector3){0.0f, 0.0f, 0.0f};
    c.accelScale = (vector3){1.0f, 1.0f, 1.0f};
    c.trimRoll = 0.0f;
    c.trimPitch = 0.0f;
    return c;
}


// nano printf has no floats: print x * 1000 as an integer
static long milli(float x){
    return (long)lroundf(x * 1000.0f);
}


static void finished(const char *msg){
    printf("%s\r\nPower cycle the board to fly.\r\n", msg);
    while(1){
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
        HAL_Delay(1000);
    }
}


imuCalStatus imuCalLoad(void){

    imuCalStatus status = {false, false};
    calRecord rec;

    if(readRecord(&rec)){
        status.accelCalibrated = rec.accelCalibrated != 0;
        status.levelTrimmed = rec.levelTrimmed != 0;
        i2cImuSetCorrection(&rec.corr);
    }else{
        imuCorrection c = identity();
        i2cImuSetCorrection(&c);
    }
    return status;
}


static int faceOf(vector3 a){
    if(a.x >  faceThreshold) return 0;
    if(a.x < -faceThreshold) return 1;
    if(a.y >  faceThreshold) return 2;
    if(a.y < -faceThreshold) return 3;
    if(a.z >  faceThreshold) return 4;
    if(a.z < -faceThreshold) return 5;
    return -1;
}


static void printMissing(const bool faceSeen[6]){
    printf("  still need:");
    for(int i = 0; i < 6; i++){
        if(!faceSeen[i]) printf(" [%s]", faceNames[i]);
    }
    printf("\r\n");
}


void imuCalRunAccel(void){

    static vector3 samples[maxSamples];
    int count = 0;
    bool faceSeen[6] = {false};
    i2cIMUData imu;

    i2cImuSetCorrection(NULL);   // fit the raw readings

    printf("\r\n=== ACCEL CALIBRATION ===\r\n"
           "Hold the drone still in many different orientations, ~1 s each.\r\n"
           "Every face must point down at least once; angles in between help too.\r\n");
    printMissing(faceSeen);

    uint32_t lastHint = HAL_GetTick();

    while(1){

        // ---- One window: is it still? ----
        float sum[3] = {0}, sumSq[3] = {0};
        float maxGyro = 0.0f;
        int good = 0;

        for(int i = 0; i < stillWindow; i++){
            if(i2cImuRead(&imu)){
                float a[3] = {imu.accelX / 9.81f, imu.accelY / 9.81f, imu.accelZ / 9.81f};
                for(int k = 0; k < 3; k++){
                    sum[k] += a[k];
                    sumSq[k] += a[k] * a[k];
                }
                float w = sqrtf(imu.gyroX*imu.gyroX + imu.gyroY*imu.gyroY + imu.gyroZ*imu.gyroZ);
                if(w > maxGyro) maxGyro = w;
                good++;
            }
            HAL_Delay(2);
        }
        if(good < stillWindow / 2) continue;

        vector3 mean = {sum[0] / good, sum[1] / good, sum[2] / good};
        bool still = maxGyro < stillGyroMax;
        for(int k = 0; k < 3; k++){
            float m = sum[k] / good;
            float var = sumSq[k] / good - m * m;
            if(var > stillAccelStdMax * stillAccelStdMax) still = false;
        }
        float norm = sqrtf(mean.x*mean.x + mean.y*mean.y + mean.z*mean.z);
        if(norm < 0.7f || norm > 1.3f) still = false;   // nothing like gravity: ignore

        if(!still){
            if(HAL_GetTick() - lastHint > 5000){
                printf("Waiting for a still, new orientation (%d samples)\r\n", count);
                printMissing(faceSeen);
                lastHint = HAL_GetTick();
            }
            continue;
        }

        // ---- New direction? ----
        bool isNew = true;
        for(int n = 0; n < count; n++){
            float c = (mean.x*samples[n].x + mean.y*samples[n].y + mean.z*samples[n].z)
                      / (norm * sqrtf(samples[n].x*samples[n].x + samples[n].y*samples[n].y + samples[n].z*samples[n].z));
            if(c > minNewAngleCos){
                isNew = false;
                break;
            }
        }
        if(!isNew) continue;

        samples[count++] = mean;
        int face = faceOf(mean);
        if(face >= 0) faceSeen[face] = true;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
        lastHint = HAL_GetTick();

        printf("Sample %d%s%s\r\n", count, face >= 0 ? ": " : "", face >= 0 ? faceNames[face] : "");

        bool allFaces = true;
        for(int i = 0; i < 6; i++) allFaces = allFaces && faceSeen[i];

        if(!allFaces){
            if(count >= maxSamples){
                printf("Too many samples without seeing every face. Starting again.\r\n");
                count = 0;
                for(int i = 0; i < 6; i++) faceSeen[i] = false;
            }
            printMissing(faceSeen);
            continue;
        }
        if(count < minSamples){
            printf("  all faces seen - %d more at any other angles\r\n", minSamples - count);
            continue;
        }
        break;
    }

    // ---- Fit ----
    vector3 offset, scale;
    float rms;
    printf("Fitting %d samples...\r\n", count);

    if(!accelFitEllipsoid(samples, count, &offset, &scale, &rms)){
        finished("Fit failed (bad samples). Try again.");
    }

    printf("offset [mg]  %ld %ld %ld\r\n", milli(offset.x), milli(offset.y), milli(offset.z));
    printf("scale x1000  %ld %ld %ld\r\n", milli(scale.x), milli(scale.y), milli(scale.z));
    printf("fit error    %ld mg rms\r\n", milli(rms));

    bool sane = rms < maxRmsG;
    float off[3] = {offset.x, offset.y, offset.z}, sc[3] = {scale.x, scale.y, scale.z};
    for(int k = 0; k < 3; k++){
        if(fabsf(off[k]) > maxOffsetG || sc[k] < minScale || sc[k] > maxScale) sane = false;
    }
    if(!sane){
        finished("Result out of range (moved during samples, or a bad sensor). NOT saved - try again.");
    }

    calRecord rec;
    rec.accelCalibrated = 1;
    rec.levelTrimmed = 0;           // old trim was measured with the old offsets: redo it
    rec.corr = identity();
    rec.corr.accelOffset = offset;
    rec.corr.accelScale = scale;
    if(!writeRecord(&rec)) finished("");

    finished("Accel calibration done. Next: level trim (drone on a level surface, roll stick right at power-up).");
}


void imuCalRunLevel(void){

    calRecord rec;
    if(!readRecord(&rec)){
        printf("No accel calibration saved - trimming the raw accel.\r\n");
        rec.accelCalibrated = 0;
        rec.corr = identity();
    }

    // Measure with offsets/scale applied but no old trim
    imuCorrection noTrim = rec.corr;
    noTrim.trimRoll = 0.0f;
    noTrim.trimPitch = 0.0f;
    i2cImuSetCorrection(&noTrim);

    printf("\r\n=== LEVEL TRIM ===\r\nKeep the drone still on a level surface...\r\n");
    HAL_Delay(1000);

    float sum[3] = {0};
    int good = 0;
    i2cIMUData imu;

    for(int i = 0; i < levelSamples; i++){
        if(i2cImuRead(&imu)){
            float w = sqrtf(imu.gyroX*imu.gyroX + imu.gyroY*imu.gyroY + imu.gyroZ*imu.gyroZ);
            if(w > levelGyroMax) finished("It moved. NOT saved - try again.");
            sum[0] += imu.accelX;
            sum[1] += imu.accelY;
            sum[2] += imu.accelZ;
            good++;
        }
        HAL_Delay(2);
    }
    if(good < levelSamples / 2) finished("IMU read errors. NOT saved.");

    float ax = sum[0] / good, ay = sum[1] / good, az = sum[2] / good;
    // Same as the attitude estimator's start-up: the tilt the board sees when the frame is level
    float roll = atan2f(-ay, -az);
    float pitch = atan2f(ax, sqrtf(ay*ay + az*az));

    printf("board tilt: roll %ld, pitch %ld (millideg)\r\n", milli(roll * radToDeg), milli(pitch * radToDeg));

    if(fabsf(roll) * radToDeg > maxTrimDeg || fabsf(pitch) * radToDeg > maxTrimDeg){
        finished("More than 10 deg - surface not level, or the IMU axis settings are wrong. NOT saved.");
    }

    rec.levelTrimmed = 1;
    rec.corr.trimRoll = roll;
    rec.corr.trimPitch = pitch;
    if(!writeRecord(&rec)) finished("");

    finished("Level trim done.");
}
