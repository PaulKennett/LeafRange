// ================================================
// LeafRange by Paul Kennett, 2022
/*
  Started from "CANa Display for Nissan LEAF"  https://ev-olution.yolasite.com/CANa.php © Copyright EV-OLUTION
  Modified by Paul Kennett to use a 128x160 TFT dsiplay and added Range functions.
  For Arduino Mega2650, Micro and Pro Mini boards
  For Arduino Micro and Mini - works by subtracting debug data out to serial port code
  The MCP2515 CAN module is connected to Vehicle-CAN ("Car-CAN") on the OBD2 socket
  See https://paulkennett.github.io/LeafRange/ for full project details

  v113 25 Apr 2022  moving Odo test display section off to a function (cleaner)
  v112 19 Apr 2022  fixed temperature display bug
  v111 18 Apr 2022  Aaargh! Why doesn't the odo reading in Jodene's 2014 Leaf work anymore?!?
  v111 17 Apr 2022  moving counter_pointer in EEPROM to location 4 (and 5) to see if that fixes new odo bug
  v110 16 Apr 2022  fixing bugs produced yesterday
  v109 15 Apr 2022  fixing bugs during drive to Tauranga in Gen 1.1 Leaf
  v108 10 Apr 2022  trying to fix charge/drive mode switching
  v107 10 Apr 2022  watthours_per_gid now derived from raw_battery_soh
  v106 09 Apr 2022  testing in Jodene's 2014 Leaf:
                    (1) raw_gids glitches 38 to 81
                    (2) Range is too pessimistic - need to factor in battery SOH
                    (3) Her SOH 2014 Leaf = 85, my SOH 2011 Leaf = 66
                    (4) Her WH_PER_GID = ???,   my WH_PER_GID 74.0F
  v105 06 Apr 2022  adding charge mode gridelines
  v104 06 Apr 2022  change charging mode x axis to 0 to 50 minutes
  v103 02 Apr 2022  Fixing Charging mode bug: after fast charge did not reset pionter when driving
  v102 31 Mar 2022  Fixing bug: Charging mode while driving
  v101 31 Mar 2022  Fixing bugs in the Charging mode
  v100 30 Mar 2022  starting to work on Charging mode feature
  v88  25 Mar 2022  replacing range_error with range + journey_odo on x axis
  v87  24 Mar 2022  tweaked the range error UI
  v86  23 Mar 2022  tweaked some colours
  v86  20 Mar 2022  optimizing code to fit into the Micro. It works!
  v85  19 Mar 2022  tidying up
*/

#define NAME    "LeafRange"
#define VERSION "Version 113"
#define DATE    "25 Apr 2022"
#define AUTHOR  "Paul Kennett"

#ifdef __AVR_ATmega2560__       // Debug mode only works on the Mega2650 board. The Micro and Mini don't haven enough program memory
bool debug_mode = false;        // Serial port debug verbosity. Turn ON by changing [false] to [true]. It will slow everything down a lot.
#endif

#include <Adafruit_GFX.h>       // by Limor Fried/Ladyada for Adafruit Industries
#include <Adafruit_ST7735.h>    // Hardware-specific library for ST7735 TFT display
#include <mcp_can.h>            // OBD2 CAN module
#include <EEPROM.h>             // read/write to the EEPROM memory
#include <Fonts/Logisoso_reduced11pt7b.h>
#include <Fonts/Logisoso_compact30pt7b.h>

#define BLACK         0x0000
#define RED           0x001F
#define ORANGE        0x041F
#define GREEN         0x07E0
#define YELLOW        0x07FF
#define LIGHT_YELLOW  0x479C
#define LIGHT_BROWN   0x061D
#define GREY          0x8410
#define LIGHT_GREY    0xC618
#define LIGHTER_GREY  0xE71C
#define LIGHTEST_GREY 0xF79E
#define CYAN          0xFFE0
#define CYAN2         0x8400
#define LIGHTER_BLUE  0xFED6
#define LIGHT_BLUE    0xFE10
#define WHITE         0xFFFF

// Car-CAN (0x174,3) raw_shifter_position vlues
#define PARK          170
#define DRIVE         187
#define REVERSE       153

// #define KM_PER_KWH 6.5F     // default km/kWh (from my car) was used to calculate Range estimate (7.3 was from best month in 2019, 6.4 was from worst month in 2019
//                             // this value is now derived from the battery_pack_temp
#define WH_PER_GID 74.0F       // The OVMS project uses 80.0, others use 74.73 (not sure why). LeafSpy default is 77.5
#define GIDS_TURTLE 8          // raw Gids-8 (the number of Gids to reach Turtle mode)
#define MILES_TO_KM 1.609344F  // conversion factor
#define CAN0_INT    2          // Set CAN0 INTerupt to pin 2
#define TFT_CS      7
#define TFT_RST     5
#define TFT_DC      6

#ifdef __AVR_ATmega2560__
#define EEPROM_SIZE 2047
#else
#define EEPROM_SIZE 1023       // currently experimenting with using just 1K of EEPROM, even on the Mega2560
#endif

#define EEPROM_ADDR_TEST 0     // EEPROM address for the test byte. If not what's expected then this must be a fresh board, so format the whole EEPROM
#define EEPROM_ADDR_DATA 4     // EEPROM address for the data pointer (2 bytes)
#define EEPROM_ADDR_DATA_START 16  // EEPROM address where the raw_gids/Odo_km data starts 
#define EEPROM_ADDR_DATA_END EEPROM_SIZE  // EEPROM address where the raw_gids/Odo_km data ends (Might as well make it EEPROM_SIZE until I think of something better.)
#define EEPROM_TEST_BYTE 151   // if EEPROM(0) is not this value then assume it's a new board so will format EEPROM
#define CHARGE_TEST_OFFSET 5   // Last_Gids plus this number is compared with raw_gids (first time thru main loop) to test if the car has just been charged

#define LINE1 20               // Y position for top line of text in start-up message
#define LINE2 42               // 2nd line of text
#define LINE3 64               // 3rd line
#define LINE4 86               // 4th line
#define LINE5 108              // 5th line
#define LINE6 130              // 6th line
#define DISPLAY_WIDTH 127      // from 0
#define DISPLAY_HEIGHT 160     // from 0
#define PLOT_TOP 37            // from top of display
#define PLOT_TOP_MARGIN 20     // from top of plot space to top line
#define PLOT_BOTTOM 33         // from bottom of display
#define PLOT_BOTTOM_MARGIN 12  // 
#define PLOT_LEFT 20           // from left side of display
#define PLOT_RIGHT 0           // from right side of display [was 27]
#define FONT_HALF_HEIGHT 4     // for gridline lables
#define PUSH_BUTTON1 8         // currently used for testing

byte plot_width  = DISPLAY_WIDTH - PLOT_LEFT - PLOT_RIGHT;
byte plot_top    = PLOT_TOP + PLOT_TOP_MARGIN;
byte plot_bottom = DISPLAY_HEIGHT - PLOT_BOTTOM - PLOT_BOTTOM_MARGIN;
byte plot_height = plot_bottom - plot_top;
byte plot_middle = plot_top + (plot_height / 2);
float scale_y = plot_height / 100.0;
float scale_x = plot_width / 100.0;
float odo_conversion = MILES_TO_KM;

long unsigned int rx_id;       // CAN message ID
byte len;                      // data length
byte rx_buf[8];                // 8 byte
uint16_t raw_gids;             // raw Gids from Vehicle CAN bus x5b3
uint16_t raw_gids2;            // raw Gids from Vehicle CAN bus x5b3
unsigned long raw_odo;         // Odometer from Vehicle CAN bus x5c5
byte raw_odo1;
byte raw_odo2;
byte raw_odo3;
byte raw_battery_pack_temp;    // raw battery pack temperature
byte raw_battery_soh;          // battery State of Health
byte raw_shifter_position;      // Shifter position
byte raw_cluster_units;

// Some nomenclature:
// A "trip" (or "leg") means: turn the car on, do some stuff (or not), then turn the car off
// A "journey" is made up of one or more trips/legs. A journey auto-starts when you start the car after the car has been charged
bool first_time = true;        // used to indicate first time through the main display loop (for driving behaviour)
bool second_time = false;      // used to indicate second time through the main display loop (used for charging behaviour)
bool charging = false;         // true when charing
int range;                     // Range estimate
int range_start;               // Range at start of this journey
int range_error;               // the difference between the initial range estimate and the current range + odo
float battery_pack_temp;       // battery pack temperature
float km_per_kWh;              // km per kWh (of course the real value would change with terrain & wind, etc) now derived via battery_pack_temp
float odo_km_start;            // Odo at start of drive session (leg) [presumably this is always zero!?]
float odo_km;                  // Odo in km (leg)
float odo_km2;                 // used to compare if new value has changed from old value
float journey_odo;             // the distance in km over the journey (multiple legs) since the car was charged
byte journey_odo_first;        // the first value of journey_odo from EEPROM(5) [presumably this is always zero!?]
byte journey_odo_last;         // the last value of journey_odo from previous drive session (leg)
byte eeprom_gids;              // the Gids value in the EEPROM
byte eeprom_gids_last;         // the last Gids value in EEPROM
unsigned int data_pointer;     // used to cound each time some fresh data is recieved from the CAN bus
float watthours_per_gid;       // value of this is now calculated via battery SOH

// for timestamps in the debug data going out to the serial port.
unsigned long all_seconds;
int run_hours;
int run_minutes;
int run_seconds;
int secs_remaining;


MCP_CAN CAN0(10);              // Set CS for MCP2515 module to pin 10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


// ============================================================================================
// ============================================================================================
// ============================================================================================


void setup() {

  // EEPROM.update(EEPROM_ADDR_TEST, 0); // Uncomment this line if you want to "manually" reformat the EEPROM. Upload & run once. Then recomment this line and upload & run.

  if (EEPROM.read(EEPROM_ADDR_TEST) != EEPROM_TEST_BYTE ) { // auto-format the EEPROM the first time this sketch is run on a board
    format_eeprom();
  }

#ifdef __AVR_ATmega2560__
  if (debug_mode) {
    Serial.begin(9600);
    copy_eeprom_to_serial_port(PUSH_BUTTON1);  // dump the whole EEPROM out the derial port
  }
#endif

  pinMode(PUSH_BUTTON1, INPUT_PULLUP);    // push button on LeafRange PCB

  // read the EEPROM
  data_pointer =  read_unsigned_int_from_eeprom(EEPROM_ADDR_DATA);
  journey_odo_last = EEPROM.read(data_pointer - 2); // reading data_pointer - 2 because data_pointer gets incremented at the end of the main loop [but what if it's a newly formatted EEPROM?!]
  eeprom_gids_last = EEPROM.read(data_pointer - 1);

#ifdef __AVR_ATmega2560__
  if (debug_mode) {
    sprint_setup_data();
  }
#endif

  tft.initR(INITR_GREENTAB);  // Init ST7735S chip, "green tab" variation
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(YELLOW);

#ifdef __AVR_ATmega2560__
  if (debug_mode) {
    Serial.println("------------------- Initialise MCP2515 CANbus Module");
  }
#endif

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    mcp2515_boot_message(); // Show version and date. Also functions as delay to get connection to MCP CANbus module running
  } else {
    mcp2515_error_message();
  }

#ifdef __AVR_ATmega2560__
  if (debug_mode) {
    sprint_time_since_boot();
    Serial.println("----------------------------------------------------");
  }
#endif

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);

  //  tft.fillScreen(BLACK);
  tft.setCursor(44, 155); tft.print("Loading...");

#ifdef __AVR_ATmega2560__
  if (debug_mode) {
    Serial.print("===================================== end setup loop\r\n\r\n");
  }
#endif

} // End setup


// ============================================================================================
// ============================================================================================
// ============================================================================================


void loop() {


  // grab a scoop of data from the Car-CAN bus using the MCP2515

  if (!digitalRead(CAN0_INT)) {                   // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rx_id, &len, rx_buf);        // Read data: len = data length, buf = data byte(s)
    if (rx_id == 0x174) {
      raw_shifter_position = (rx_buf[3]);   // not usin this yet
    } else if (rx_id == 0x355) {
      raw_cluster_units = (rx_buf[6]);
      if (raw_cluster_units == 64) { // 64 = km, 96 = miles
        odo_conversion = 1;
      }
    } else if (rx_id == 0x5b3) {
      raw_battery_pack_temp = (rx_buf[0]);
      raw_battery_soh = (rx_buf[1]) >> 1;
      raw_gids = (rx_buf[5]);
    } else if (rx_id == 0x5b3) {
      raw_battery_pack_temp = (rx_buf[0]);
      raw_battery_soh = (rx_buf[1]) >> 1;
      raw_gids = (rx_buf[5]);
    } else if (rx_id == 0x5c5) {
      raw_odo = (rx_buf[1] << 16 | rx_buf[2] << 8 | rx_buf[3]);
      raw_odo1 = rx_buf[1];
      raw_odo2 = rx_buf[2];
      raw_odo3 = rx_buf[3];
      if (odo_km_start == 0) {
        odo_km_start = (float)raw_odo * odo_conversion;
      } else {
        odo_km = ((float)raw_odo * odo_conversion) - odo_km_start;
      }
    }
  }
  // end grab CAN bus data


  // If got fresh raw_gids or odod_km - do all the stuff -------------------------------
  if ((raw_gids != 0) && (raw_gids2 != raw_gids || odo_km2 != odo_km)) {

    journey_odo = journey_odo_last + odo_km;

    if (!charging) {    // update data_pointer and save data to the EEPROM data array
      EEPROM.update(data_pointer, (byte)journey_odo);
      EEPROM.update(data_pointer + 1, (byte)raw_gids);
    }

    // calculate some stuff
    battery_pack_temp = (raw_battery_pack_temp / 4) - 10;
    watthours_per_gid = 52.17 + (raw_battery_soh * 0.33); // EXPERIMENTAL
    km_per_kWh = 5.05 + (0.1 * battery_pack_temp) - (0.0005 * (pow(battery_pack_temp, 2.1))); // formula is fudged from FlipTheFleet dataset (now tapers at higher temps)
    range = km_per_kWh * ((raw_gids - GIDS_TURTLE) * watthours_per_gid) / 1000;


    // ----------------------------------------------------------------------------------
    if (second_time) {                    // if this is the second time thru the data loop then test if charging
      if (raw_gids > raw_gids2 && odo_km2 == odo_km && raw_shifter_position == PARK) {         // If second raw_gids value has increased then it must be charging
        charging = true;
        //        range_start = km_per_kWh * ((EEPROM.read(EEPROM_ADDR_DATA_START + 1) - GIDS_TURTLE) * WH_PER_GID) / 1000;
        range_start = km_per_kWh * ((EEPROM.read(EEPROM_ADDR_DATA_START + 1) - GIDS_TURTLE) * watthours_per_gid) / 1000;
        print_static_drive_elements();
        data_pointer = EEPROM_ADDR_DATA_START;
        write_unsigned_int_to_eeprom(EEPROM_ADDR_DATA, data_pointer);
        EEPROM.update(data_pointer, 0); // reset first journey_odo value in EEPROM to zero
        journey_odo_last = 0;
        journey_odo = 0;
        EEPROM.update(data_pointer + 1, (byte)raw_gids);
        eeprom_gids_last = raw_gids;
      } else
        //if (raw_gids < raw_gids2 || raw_shifter_position == DRIVE) { // If raw_gids has decreased then it must be not charging
        // in Jolene's 2014 Leaf the raw_Gid glitches from 38 up to 81 and back intermitantly whilst slow charging
      {
        charging = false;
      }

      second_time = false;
    } // end of "second time only" tasks
    // ----------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------
    // "first time only" tasks: draw a dashed line and plot stored data
    if (first_time) {                    // if this is the first time then save raw_gids to EEPROM max_gids array
      tft.fillScreen(BLACK);

      // Has the car just been charged?
      if (raw_gids > eeprom_gids_last + CHARGE_TEST_OFFSET && !charging) { // If first raw_gids has increased then it must have charged
        tft.fillRect(0, 0, 15, 40, YELLOW);
#ifdef __AVR_ATmega2560__
        if (debug_mode) {
          Serial.println("| Was charging! Reset Drive and Display counters");
        }
#endif
        data_pointer = EEPROM_ADDR_DATA_START;
        write_unsigned_int_to_eeprom(EEPROM_ADDR_DATA, data_pointer);
        EEPROM.update(data_pointer, 0); // reset first journey_odo value in EEPROM to zero
        journey_odo_last = 0;
        journey_odo = 0;
        EEPROM.update(data_pointer + 1, (byte)raw_gids);
        eeprom_gids_last = raw_gids;
      }

      //      range_start = km_per_kWh * ((EEPROM.read(EEPROM_ADDR_DATA_START + 1) - GIDS_TURTLE) * WH_PER_GID) / 1000;
      range_start = km_per_kWh * ((EEPROM.read(EEPROM_ADDR_DATA_START + 1) - GIDS_TURTLE) * watthours_per_gid) / 1000;

      print_static_drive_elements();
      int y = plot_bottom - (range_start * scale_y) - 1;
      print_initial_range_on_y_axis(y);
      int x2 = (PLOT_LEFT + (range_start * scale_x));
      draw_dashed_line (x2, y);
      plot_saved_data(); // plot data from EEPROM data array

      first_time = false;
      second_time = true; //
    } // end of "first time only" tasks
    // ----------------------------------------------------------------------------------


#ifdef __AVR_ATmega2560__
    if (debug_mode) {
      sprint_display_update_data();
    }
#endif
    print_range(range);
    print_battery_pack_temp(battery_pack_temp);
    int y = plot_bottom - (range * scale_y) - 1;
    if (charging) {
      sprint_time_since_boot();
      tft.fillRect(PLOT_LEFT + (run_minutes * 2) - 1, y - 1, 3, 3, GREEN);   // plot new green range estimate "dot" whilst charging
      // temporary test data
      tft.setFont();
      tft.setTextSize(1);
      tft.setTextColor(LIGHTER_GREY);
      tft.setCursor(0, 151);
      tft.fillRect(0, 150, 128, 10, BLACK);
      tft.print(raw_gids); tft.print("G ");
      tft.print(run_minutes); tft.print("M ");
      // tft.print(data_pointer); tft.print("P ");
      tft.print(raw_battery_soh); tft.print("H ");
      // tft.print(raw_shifter_position); tft.print("S ");
      tft.print(watthours_per_gid); tft.print("W ");

    } else {
      tft.fillRect(PLOT_LEFT + journey_odo - 1, y - 1, 3, 3, YELLOW);     // plot new yellow range estimate "dot" whilst driving
      // range_error = (range + journey_odo) - range_start;
      // print_range_error(range_error);
      print_odo_plus_range_on_x_axis(range + journey_odo);
      // print_trip_odo(odo_km);
      // print_test_info();
      print_journey_odo(journey_odo);

    }
#ifdef __AVR_ATmega2560__
    if (debug_mode) {
      //   print_info_message(); // print some data for testing
    }
#endif

    // update "has it changed?" variables
    odo_km2 = odo_km;
    raw_gids2 = raw_gids;

    if (data_pointer < EEPROM_ADDR_DATA_END - 2 && !charging) { // don't increment pointer if it's reached the end of the EEPROM
      data_pointer = data_pointer + 2;     // increment data array pointer
      write_unsigned_int_to_eeprom(EEPROM_ADDR_DATA, data_pointer);
    }

#ifdef __AVR_ATmega2560__
    if (debug_mode) {
      sprint_time_since_boot();
      Serial.print("--------------------------- display update loop end\r\n\r\n");
    }
#endif
  } // end of the "got some data" stuff


  // Options for the push button (pick one of you want to) - I haven't tested these properly yet
  // push_button_reset_data_pointer(PUSH_BUTTON1);
  // push_button_format_eeprom(PUSH_BUTTON1);

} // END MAIN LOOP


// ============================================================================================
// ============================================================================================
// ============================================================================================


void mcp2515_boot_message() {
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setCursor(2, LINE1); tft.print(NAME);
  tft.setTextColor(LIGHTER_GREY);
  tft.setCursor(2, LINE2); tft.print(VERSION);
  tft.setCursor(2, LINE3); tft.print(DATE);
  tft.setCursor(2, LINE4); tft.print(AUTHOR);
  tft.setTextColor(GREY);
  tft.setCursor(2, 155); tft.print((data_pointer / 2) - (EEPROM_ADDR_DATA_START / 2) + 1);
  delay(1000);
}


void mcp2515_error_message() {
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setCursor(0, LINE1); tft.print("MCP2515");
  tft.setCursor(0, LINE2); tft.print("connect");
  tft.setCursor(0, LINE3); tft.print("error");
  delay(10000);
}


void format_eeprom() {
  for (int i = 0; i <= EEPROM_SIZE; i++) {
    EEPROM.update(i, 0);
  }
  EEPROM.update(0, EEPROM_TEST_BYTE);
  write_unsigned_int_to_eeprom(EEPROM_ADDR_DATA, EEPROM_ADDR_DATA_START);
}


void print_static_drive_elements() {
  tft.fillScreen(BLACK);
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setTextColor(LIGHTEST_GREY);
  tft.setCursor(77, 16);  tft.print("km");
  tft.setCursor(77, 40);  tft.print("Range ");
  if (!charging) {
    //    tft.setTextColor(LIGHT_BLUE);
    //    tft.setCursor(0, 135);  tft.print("Trip");
    //    tft.setCursor(103, 135);  tft.print("km");
    tft.setTextColor(LIGHTER_BLUE);
    tft.setCursor(0, 155);  tft.print("Journey");
    tft.setCursor(103, 155);  tft.print("km");
  }
  // y axes
  tft.drawFastHLine (PLOT_LEFT - 1, plot_top, 3, LIGHTER_GREY);         // 100km Range tick mark
  tft.drawFastHLine (PLOT_LEFT - 1, plot_middle, 3, LIGHTER_GREY);      // 50km Range tick mark
  tft.drawFastHLine (PLOT_LEFT - 1, plot_bottom, plot_width + 1, LIGHTER_GREY);
  // x axes
  tft.drawFastVLine (PLOT_LEFT , plot_top - 6, plot_height + 8, LIGHTER_GREY); //
  tft.drawFastVLine (PLOT_LEFT + 100, plot_bottom - 1, 3, LIGHTER_GREY); //100km teavelled tick mark

  tft.setFont();
  tft.setTextColor(LIGHTER_GREY);
  tft.setTextSize(1);
  // print y axis lables
  tft.setCursor(0, plot_top - FONT_HALF_HEIGHT + 1);  tft.print("100");
  tft.setCursor(6, plot_middle - FONT_HALF_HEIGHT);  tft.print("50");
  tft.setCursor(12, plot_bottom - FONT_HALF_HEIGHT);  tft.print("0");
  // print x axis lables
  if (!charging) {
    tft.drawFastVLine (PLOT_LEFT + 50, plot_bottom - 1, 3, LIGHTER_GREY); // 50km travelled tick mark
    tft.setCursor(PLOT_LEFT - 2, plot_bottom + 4);  tft.print("0km");
    tft.setCursor(PLOT_LEFT + 50 - 5, plot_bottom + 4);  tft.print("50km");
    tft.setCursor(PLOT_LEFT + 100 - 9, plot_bottom + 4);  tft.print("100");
  } else {
    tft.drawFastHLine (PLOT_LEFT - 1, plot_top + (plot_height * 0.25), 3, LIGHTER_GREY);      // 75km Range tick mark
    tft.drawFastHLine (PLOT_LEFT - 1, plot_top + (plot_height * 0.75), 3, LIGHTER_GREY);      // 50km Range tick mark
    tft.drawFastHLine (PLOT_LEFT + 2, plot_top + (plot_height * 0.25), PLOT_LEFT + 89, GREY);  // 75km line
    tft.drawFastHLine (PLOT_LEFT + 2, plot_middle, PLOT_LEFT + 89, GREY);                      // 50km line
    tft.drawFastHLine (PLOT_LEFT + 2, plot_top + (plot_height * 0.75), PLOT_LEFT + 89, GREY);  // 25km line
    tft.drawFastHLine (PLOT_LEFT + 2, plot_top, PLOT_LEFT + 89, GREY);                         // 100km line
    //  tft.fillRect (PLOT_LEFT + 2, plot_top, PLOT_LEFT + 89, plot_height * 0.2, GREY);  // 75km line
    tft.drawFastVLine (PLOT_LEFT + 20, plot_top, plot_height + 2, GREY); // 10 minute line
    tft.drawFastVLine (PLOT_LEFT + 40, plot_top, plot_height + 2, GREY); // 20 minute line
    tft.drawFastVLine (PLOT_LEFT + 60, plot_top, plot_height + 2, GREY); // 30 minute line
    tft.drawFastVLine (PLOT_LEFT + 80, plot_top, plot_height + 2, GREY); // 40 minute line
    tft.drawFastVLine (PLOT_LEFT + 100, plot_top, plot_height + 2, GREY); //50 minute line
    tft.drawFastVLine (PLOT_LEFT + 20, plot_bottom - 1, 3, LIGHTER_GREY); //20km teavelled tick mark
    tft.drawFastVLine (PLOT_LEFT + 40, plot_bottom - 1, 3, LIGHTER_GREY); //40km teavelled tick mark
    tft.drawFastVLine (PLOT_LEFT + 60, plot_bottom - 1, 3, LIGHTER_GREY); //60km teavelled tick mark
    tft.drawFastVLine (PLOT_LEFT + 80, plot_bottom - 1, 3, LIGHTER_GREY); //80km teavelled tick mark
    tft.drawFastVLine (PLOT_LEFT + 100, plot_bottom - 1, 3, LIGHTER_GREY); //100km teavelled tick mark
    tft.setTextColor(GREEN);
    tft.setCursor(PLOT_LEFT - 2, plot_bottom + 4);  tft.print("0");
    tft.setCursor(PLOT_LEFT + 31, plot_bottom + 4);  tft.print("Minutes");
    tft.setCursor(PLOT_LEFT + 100 - 4, plot_bottom + 4);  tft.print("50");
  }
}


void print_initial_range_on_y_axis(int y) {
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(LIGHT_YELLOW);
  tft.fillRect(0, y - 4, 18, 10, BLACK);
  if (range_start < 100) {
    tft.setCursor(6, y - 4);
  } else {
    tft.setCursor(0, y - 4);
  }
  tft.print(range_start);
}


void draw_dashed_line(int x2, int y) {
  int dx = x2 - PLOT_LEFT;
  int dy = plot_bottom - y;
  int D = 2 * dy - dx;
  for (int x = PLOT_LEFT; x <= x2; x ++) {
    if ( (x % 8) > 3) {      // dash the line
      tft.drawPixel(x, y, LIGHTEST_GREY);
    }
    if (D > 0) {
      //      y = y + 1;
      y++;
      D = D - 2 * dx;
    }
    D = D + 2 * dy;
  }
}


void plot_saved_data () {
  for (int i = EEPROM_ADDR_DATA_START; i <= data_pointer; i += 2) {
    journey_odo = EEPROM.read(i);
    range = km_per_kWh * ((EEPROM.read(i + 1) - GIDS_TURTLE) * watthours_per_gid) / 1000;
    int y = plot_bottom - (range * scale_y) - 1;
    tft.fillRect(PLOT_LEFT + journey_odo - 1, y - 1, 3, 3, LIGHT_BROWN);
  }
}


void print_range(int range) {
  tft.setFont(&Logisoso_compact30pt7b);
  tft.setTextColor(GREEN); // make range number green
  tft.setCursor(20, 40);
  if (range > 99) {
    tft.setCursor(-6, 40);
  } else if (range < 80) {
    tft.setTextColor(CYAN); // if range is under 80 km
  } else if (range < 10) {
    tft.setCursor(45, 40);
    tft.setTextColor(RED); // if range is under 10 km
  }
  tft.fillRect(0, 0, 70, 42, BLACK);
  tft.print(range);
}


void print_odo_plus_range_on_x_axis(int x) {
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(LIGHT_YELLOW);
  tft.fillRect(PLOT_LEFT + x - 6, plot_bottom + 1, 18, 11, BLACK);
  //tft.drawRect(PLOT_LEFT + x - 3, plot_bottom + 1, 18, 11, RED);
  tft.drawFastVLine (PLOT_LEFT + x + 2, plot_bottom + 1, 2, YELLOW); // 50km travelled tick mark
  if (x < 100) {
    tft.setCursor(PLOT_LEFT + x - 2, plot_bottom + 4);
  } else {
    tft.setCursor(PLOT_LEFT + x - 5, plot_bottom + 4);
  }
  tft.print(x);
}

/*
  void print_range_error(int error) {
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.fillRect(89, 50, 30, 18, BLACK);
  #ifdef __AVR_ATmega2560__
  if (debug_mode) {
    Serial.print("| range_error         "); Serial.print(error); Serial.println();
  }
  #endif
  if (error > 9) {
    tft.setTextColor(GREEN);
    tft.setCursor(97, 66);
    tft.drawRect(89, 59, 8, 2, GREEN);
    tft.drawRect(92, 55, 2, 10, GREEN);
    tft.drawTriangle(119, 64, 127, 64, 123, 53, GREEN);
  } else if (error < 9 && error > 2) {
    tft.setTextColor(GREEN);
    tft.setCursor(108, 66);
    tft.drawRect(99, 59, 8, 2, GREEN);
    tft.drawRect(102, 55, 2, 10, GREEN);
    tft.drawTriangle(119, 64, 127, 64, 123, 53, GREEN);
  }
  if (error < -1) {
    tft.setTextColor(LIGHTER_GREY);
    tft.setCursor(97, 66);
    tft.drawTriangle(119, 64, 127, 64, 123, 53, LIGHTER_GREY);
  }
  if (error < -5) {
    tft.setTextColor(ORANGE);
    tft.setCursor(97, 66);
    tft.drawTriangle(119, 64, 127, 64, 123, 53, ORANGE);
  }
  if (error < -9) {
    tft.setTextColor(RED);
    tft.setCursor(87, 66);
    tft.drawTriangle(119, 64, 127, 64, 123, 53, RED);
  }
  tft.print(error);
  }
*/

void print_battery_pack_temp(int temperature) {
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setTextColor(RED);
  tft.setCursor(97, 70);
  if (temperature < 35) {
    tft.setTextColor(ORANGE);
  }
  //  if (temperature < 30) {
  //    tft.setTextColor(BLACK);
  //  }
  if (temperature < 13) {
    tft.setTextColor(LIGHT_BLUE);
  }
  if (temperature < 8) {
    tft.setTextColor(CYAN);  // was CYAN
    tft.setCursor(107, 70);
  }
  if (temperature < 0) {
    tft.setTextColor(WHITE);
    tft.setCursor(97, 70);
  }
  if (temperature < -9) {
    tft.setTextColor(WHITE);
    tft.setCursor(87, 70);
  }
  if (temperature > 30 || temperature < 13) {
    tft.fillRect(98, 54, 20, 18, BLACK);
    tft.print(temperature);
    tft.setCursor(119, 70);
    tft.print("C");
  } else if (temperature == 30 || temperature == 13) {
    tft.fillRect(98, 54, 20, 18, BLACK);
  }
}


void print_trip_odo(float trip) { // this leg of the journey
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setTextColor(LIGHT_BLUE);
  tft.setCursor(54, 135);
  if (trip < 100) {
    tft.setCursor(62, 135);
  }
  if (trip < 10) {
    tft.setCursor(72, 135);
  }
  tft.fillRect(48, 120, 50, 16, BLACK);
  // tft.drawRect(48, 120, 50, 16, RED); // used during design
  tft.print(trip, 1);
}


void print_journey_odo(float journey) {
  tft.setFont(&Logisoso_reduced11pt7b);
  tft.setTextColor(LIGHTER_BLUE);
  if (journey > 99) {
    tft.setCursor(70, 155);
    tft.fillRect(69, 139, 30, 18, BLACK);
    //    tft.drawRect(69, 139, 30, 18, RED); // used during design
    tft.print(journey, 0);
  } else if (journey < 10) {
    tft.setCursor(72, 155);
    tft.fillRect(69, 139, 30, 18, BLACK);
    //    tft.drawRect(69, 139, 30, 18, RED); // used during design
    tft.print(journey, 1);
  } else {
    tft.setCursor(78, 155);
    tft.fillRect(69, 139, 30, 18, BLACK);
    //    tft.drawRect(69, 139, 30, 18, RED); // used during design
    tft.print(journey, 0);
  }
}


void print_info_message() {    // this is currently used for bug fixing. Planning to remove this when code is stable (haha!)
  tft.fillRect(0, 133, 128, 9, LIGHT_GREY);
  //tft.drawRect(70, 130, 58, 9, RED);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.setCursor(2, 134);
  tft.print("Cnt:"); tft.print((data_pointer / 2) - (EEPROM_ADDR_DATA_START / 2) + 1);  tft.print(" of ");   tft.print((EEPROM_ADDR_DATA_END - EEPROM_ADDR_DATA_START) / 2);
}


void print_test_info() {
  // temporary test data
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(LIGHTER_GREY);
  tft.setCursor(0, 131);
  tft.fillRect(0, 130, 128, 10, BLACK);
  //      tft.print(raw_odo); tft.print("RO ");
  //    tft.print(odo_km_start); tft.print("KMs ");
  //  tft.print(odo_km); tft.print("KM ");
  tft.print(raw_odo1); tft.print("O1 ");
  tft.print(raw_odo2); tft.print("O3 ");
  tft.print(raw_odo3); tft.print("O3 ");
  //      tft.print(raw_odo); tft.print("RO ");

}


void write_unsigned_int_to_eeprom(int address, unsigned int number) {
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int read_unsigned_int_from_eeprom(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}


void push_button_reset_data_pointer(int pin) {
  // push button during boot action - reset data pointer
  if (digitalRead(pin) == LOW) {
    tft.fillRect(0, 0, 10, 10, YELLOW);
    digitalWrite(13, HIGH);
    data_pointer = EEPROM_ADDR_DATA_START;
    write_unsigned_int_to_eeprom(EEPROM_ADDR_DATA, data_pointer);
    EEPROM.update(data_pointer - 2, 0);
    //  EEPROM.update(data_pointer - 1,0);
    delay (1000);
    digitalWrite(13, LOW);
    tft.fillRect(0, 0, 10, 10, BLACK);
  }
}


void push_button_format_eeprom(int pin) {
  // push button option: reformats the EEPROM (for testing)
  if (digitalRead(pin) == LOW) {
    tft.fillRect(0, 0, 10, 10, YELLOW);
    digitalWrite(13, HIGH);
    EEPROM.update(0, 0); // trigger EEPROM reformat - for testing!
    delay (100);
    digitalWrite(13, LOW);
    tft.fillRect(0, 0, 10, 10, BLACK);
  }
}


void sprint_time_since_boot() {
  all_seconds = millis() / 1000;
  run_hours = all_seconds / 3600;
  secs_remaining = all_seconds % 3600;
  run_minutes = secs_remaining / 60;
  run_seconds = secs_remaining % 60;
#ifdef __AVR_ATmega2560__
  char buf[33];  //Warning to self: if buf[?] is too small then Arduino board hangs!
  sprintf(buf, "Elapsed time         [%02d:%02d:%02d]", run_hours, run_minutes, run_seconds);
  Serial.println(buf);
#endif
}

#ifdef __AVR_ATmega2560__
void sprint_setup_data() {
  Serial.println("\r\n\r\n====================================================");
  Serial.println(NAME" by "AUTHOR", "DATE", "VERSION);
  Serial.println("====================================================");
  Serial.print("Fresh EEPROM test     "); Serial.print(EEPROM.read(EEPROM_ADDR_TEST)); Serial.println();
  Serial.print("data_pointer (2 bytes)"); Serial.print(data_pointer); Serial.println();
  Serial.print("Odo first             "); Serial.print(EEPROM.read(EEPROM_ADDR_DATA_START));      Serial.print(" at EEPROM("); Serial.print(EEPROM_ADDR_DATA_START);   Serial.println(")");
  Serial.print("Gids first            "); Serial.print(EEPROM.read(EEPROM_ADDR_DATA_START + 1));  Serial.print(" at EEPROM("); Serial.print(EEPROM_ADDR_DATA_START + 1); Serial.println(")");
  Serial.print("Odo second            "); Serial.print(EEPROM.read(EEPROM_ADDR_DATA_START + 2));  Serial.print(" at EEPROM("); Serial.print(EEPROM_ADDR_DATA_START + 2); Serial.println(")");
  Serial.print("Gids second           "); Serial.print(EEPROM.read(EEPROM_ADDR_DATA_START + 3));  Serial.print(" at EEPROM("); Serial.print(EEPROM_ADDR_DATA_START + 3); Serial.println(")");
  Serial.print("Odo  @ J Pointer -2   "); Serial.print(EEPROM.read(data_pointer - 2)); Serial.print(" at EEPROM("); Serial.print(data_pointer - 2); Serial.print(")\r\n");
  Serial.print("Gids @ Pointer -1     "); Serial.print(EEPROM.read(data_pointer - 1)); Serial.print(" at EEPROM("); Serial.print(data_pointer - 1); Serial.print(")\r\n");
  Serial.print("Odo  @ J Pointer      "); Serial.print(journey_odo_last); Serial.print(" at EEPROM("); Serial.print(data_pointer  ); Serial.print(")\r\n");
  Serial.print("Gids @ Pointer +1     "); Serial.print(eeprom_gids_last); Serial.print(" at EEPROM("); Serial.print(data_pointer + 1); Serial.print(")\r\n");
  sprint_time_since_boot();
  Serial.println("----------------------------------------------------");
}

void sprint_display_update_data() {
  Serial.print("|------------------------- display update loop start\r\n");
  Serial.print("| data_pointer        "); Serial.print(data_pointer); Serial.println();
  Serial.print("| EEPROM("); Serial.print(data_pointer);     Serial.print(") Odo_km  "); Serial.print(EEPROM.read(data_pointer)); Serial.println();
  Serial.print("| EEPROM("); Serial.print(data_pointer + 1); Serial.print(") raw_gids "); Serial.print(EEPROM.read(data_pointer + 1)); Serial.println();
  Serial.print("| raw_gids             "); Serial.print(raw_gids); Serial.println();
  Serial.print("| eeprom_gids_last    "); Serial.print(eeprom_gids_last); Serial.println();
  Serial.print("| range               "); Serial.print(range); Serial.println();
  Serial.print("| journey_odo_last    "); Serial.print(journey_odo_last); Serial.println(" km");
  Serial.print("| odo_km              "); Serial.print(odo_km); Serial.println(" km");
  Serial.print("| journey_odo         "); Serial.print(journey_odo); Serial.println(" km");
}


void copy_eeprom_to_serial_port(int pin) {   // Output the whole EEPROM to the serial port
  Serial.println("EEPROM DUMP");
  int i2 = read_unsigned_int_from_eeprom(EEPROM_ADDR_DATA);
  for (int i = 0; i < i2 + 10; i++) {
    Serial.print(i); Serial.print(", "); Serial.print(EEPROM.read(i));
    if (i == EEPROM_ADDR_DATA_START) {
      Serial.println(", <-- Data starts");
    } else if (i == i2) {
      Serial.println(", <-- pointer");
    } else {
      Serial.println();
    }
  }
  Serial.println("===========");
}
#endif
