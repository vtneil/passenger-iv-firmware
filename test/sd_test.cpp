#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_BNO08x.h"
#include "psg_4_definitions.h"
#include "psg_4_sd_tools.h"

SPIClass SPI_1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCLK1);
SdSpiConfig sd0_cfg(PIN_SPI_CS_SD_INT, DEDICATED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_1);
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;

void setup() {
    Serial.begin(115200);
    delay(2000);

    retry:
    delay(500);
    Serial.println("Begin SD!");
    bool status = sd0.begin(sd0_cfg);

    if (status)
        Serial.println("Sucess!");
    else {
        Serial.println("Failed!");
        goto retry;
    }

    make_new_filename(sd0, sd0_filename, "mcu0_data_test_", ".csv");
    list_files(sd0);
    open_for_append(sd0, sd0_file, sd0_filename);
    sd0_file.println("DATATADTATDASDHJAGSFHJGHJFGKJASHFAJSFH");
    sd0_file.close();
    read_file_to_stream(sd0, sd0_filename);
}

void loop() {
    Serial.println("Loop!");
    delay(5000);
}
