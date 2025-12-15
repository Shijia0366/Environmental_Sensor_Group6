#pragma once

/************ I2C  ************/
#define I2C_SDA_PIN            -1        
#define I2C_SCL_PIN            -1
#define I2C_FREQUENCY          400000    

// STEMMA/QT Power switch
#define PIN_I2C_POWER          7
#define I2C_POWER_DELAY_MS     50         
#define I2C_BUS_SETTLE_MS      100       

/************ SPI ************/

#define SPI_SCK_PIN            -1        
#define SPI_MISO_PIN           -1        
#define SPI_MOSI_PIN           -1        

/************ SD Card (SPI) ************/
#define SD_CS                  13        
#define SD_INIT_FREQ_HZ        400000    
#define SD_WORK_FREQ_HZ        20000000  
#define SD_POWER_DELAY_MS      10        

#define SD_CD_PIN              -1        
#define SD_CD_ACTIVE_LOW       1         
#define SD_CD_DEBOUNCE_MS      20

/************ Device I2C Addresses ************/
#define ToF_ADDR               0x29
#define GPS_ADDR               0x10      // according to your GPS module's actual address
#define BME_ADDR               0x77
#define OLED_ADDR              0x3D

/************ Device Control Pins ************/
#define OLED_RESET             5        // OLED RST
// #define OLED_RESET             -1        
#define TOF_XSHUT_PIN          5        
// #define TOF_XSHUT_PIN          -1       

#define TOF_INT_PIN     9      


/************ App Settings ************/
#define LOG_INTERVAL_MS        10      
#define CSV_FILE_PATH          "/logs/data.csv"
#define CSV_HEADER             "timestamp_ms,temp_c,hum_pct,press_hpa,gas_ohm,AQI,dist_mm,speed,lat,lon,alt_m,fix,acc_x,acc_y,acc_z,HR,SPO2"

#define DIAG_MODE 0   //1 = Enter diagnostic mode; 0 = Normal app mode
