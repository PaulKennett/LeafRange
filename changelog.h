/*
LeafRange
https://paulkennett.github.io/LeafRange/

A project to create a small OLED display that fits INSIDE the main dash of a 
2011-2012 Nissan Leaf, over the top of and covering/replacing the 
Guess-o-meter Range estimater (and charge bars).

The LeafRange project is a sister project of my LeafSOC diisplay which was based 
on "CANa Display for Nissan LEAF"  https://ev-olution.yolasite.com/CANa.php 
© Copyright EV-OLUTION

  Modified by Paul Kennett October 2019 to talk to a standard/cheap 128x64 OLED dsiplay.
  See also http://www.myonlinediary.com/index.php/Energy/SOCDisplay for my project page

  v113 25 Apr 2022  moving Odo test display section off to a function
  v112 19 Apr 2022  Fixing temperature display bug
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
  v101 31 Mar 2022  Fixing bugs in the Charging mode
  v100 30 Mar 2022  starting to work on Charging mode feature
  v88  25 Mar 2022  replacing range_error with range + journey_odo on x axis
  v87  24 Mar 2022  tweaked the range error UI
  v86  23 Mar 2022  tweaked some colours
  v86  20 Mar 2022  optimizing code to fit into the Micro. It works!
  v85  19 Mar 2022  tidying upv85  19 Mar 2022  tidying up
  v84  18 Mar 2022  passing variables to functions more
  v83  17 Mar 2022  fiddling with colours
  v82  13 Mar 2022  fixed the "initial range after charge" bug!
  v81C 13 Mar 2022  still trying to fix the "initial range after charge" bug
  v81  12 Mar 2022  bug fixing
  v80  11 Mar 2022  Add Trip feature back in
  v79  10 Mar 2022  Removed Max Gids & boot_counter code as I'm not using it for this project anymore.
  v78  10 Mar 2022  Standardize variable naming to: lowercase_underscore_names
  v77  10 Mar 2022  Using #define to set EEPROM addresses. Adding DEBUG_MODE code to allow serial stuff to be turned off
  v76   9 Mar 2022  bug fixing. It kinda works!
  v75   8 Mar 2022  fiddling - it seems to be working!
  v74B  7 Mar 2022  minor tweaks
  v74   6 Mar 2022  move things around a bit
  v73   6 Mar 2022  debugging and added routine to dump whole EEPROM to serial port
  v72   5 Mar 2022  more debugging
  v71   2 Mar 2022  more debugging
  v70   2 Mar 2022  more debugging behaviour after a charge session
  v69   1 Mar 2022  debugging test for previous charge session
  v68  28 Feb 2022  working on test for previous charge session
  v67  27 Feb 2022  fixed Journey_Odo bugs
  v66  26 Feb 2022  fixing bugs
  v65B 26 Feb 2022  fixing bugs
  v64B 21 Feb 2022  fiddling
  v63D 20 Feb 2022  fiddling
  v62  17 Feb 2022  cleaning
  v61  14 Feb 2022  cleaning code
  v60  12 Feb 2022  bug fixing
  v59  09 Feb 2022  more fiddling
  v58  07 Feb 2022  shifting code blocks into functions for clarity - ended badly, reverted to v57
  v57  07 Feb 2022  Revamp EEPROM layout
  v56  03 Feb 2022  dashed line formoula now starts from first Gids value in EEPROM
  v60  09 Feb 2022  more fiddling
  v59  09 Feb 2022  more fiddling
  v58  07 Feb 2022  shifting code blocks into functions for clarity - ended badly, reverted to v57
  v57  07 Feb 2022  Revamp EEPROM layout
  v56  03 Feb 2022  dashed line formoula now starts from first Gids value in EEPROM
  v55  02 Feb 2022
  v54  30 Jan 2022  adding drive counter
  v53  29 Jan 2022  adding more sprint lines
  v52  26 Jan 2022  send test data out the serial port
  v51  12 Jan 2022  tweaking screen update counter
  v50  10 Jan 2022  switching Micro to Mega 2650
  v42  04 Jan 2022  tweaking range_error and batteryPackTemp and WH_PER_GID
  v41  30 Dec 2021  colourizing range_error and batteryPackTemp
  v40  29 Dec 2021  add initial SOC in yellow on Y axes
  v39  25 Dec 2021  updated km_per_kW to taper effiency at high temperatures
  v38  08 Dec 2021  changed WH_PER_GID to 75
  v37  01 Dec 2021  Reinstate the auto setting of MaxGids
  v36D 20 Nov 2021  set km_per_kWh value from BatteryPackTemperature
  v36  20 Nov 2021  downward sloping graph working
  v35  17 Nov 2021  Failed to get downward sloping graph
  v34  17 Oct 2021  Testing
  v33  07 May 2021  Minor tweaks
  V31  06 May 2021  Minor tweaks
  V30  05 May 2021  Cleaning out cruft
  V29  05 May 2021  Set kw_per_kWh via the AC controls [couldn't get Temp reading working]
  V28  04 May 2021  Auto set kw_per_kWh from the Outside Ambient Temp
  V28  01 May 2021  Starting on adjusting the km/kWh factor via the AC settings, (or have it drift automatically?)
                    Could do a looku ptable for different battery temps instead?


  V27 - 30 Apr 2021
  Trivial tweaking

  V26 - 24 Apr 2021
  Reintegrating the CAN code and trimming excess code & variables
  
  V25 - 23 Apr 2021
  Finished UI design (for the moment)

  V24 - 19 Apr 2021
  Removing the red and green line code

  V24 - 19 Apr 2021
  RangeP branch is for display in portrait mode
  Begin simplifying range estimate lines to single line
  
  V22 - 8 Feb 2021
  Improving "shaded" lines

  V21 - 28 Dec 2020
  Adding "shaded" lines

  V20 - 27 Nov 2020
  Adding Logisoso font (matches the font used in my LeafSOC project)
  - https://fonts2u.com/logisoso.font
  - using https://rop.nl/truetype2gfx/
  - Logisoso 16pt too big
  - Logisoso 12pt about right

  V17 - 10 Oct 2020
  changed display to 1.8" colour TFT, 128 x 160 px
  changing to use Adafruit GFX lib (rather than U8clib, just to see how I like it. Already missing good font selection)

  V15 - 22 Sept 2020
  started to implement chart scaling - evetually I want the graph to adjust the scale to suit each sessions range
  (ie, if the range is 72 km then I want the graph to zoom in so that the x axes has 72 km at the right side of the display)

  V09 - 26/08/2020
  Improving the Range chart(?)

  V08 - 26/08/2020
  Changing to button reasding code that uses millis and allows for long button press

  V07 - 25 Aug 2020
  Discovered that the rawOdo value from Car-Can (Instrument cluster to BCM, VCM & AV) is in Miles!
  Making do with rawOdo as a 2 byte value. I haven't figuired out how to union all 3 bytes (embarrising),
  but I don't actualy need all 3 bytes.

  V06 - 12/8/2020
  Still trying to crack the Odometer data fields - AAAARGH!

  V05
  Added summer and winter range to Range vs Distance chart

  V04
  Added Range chart (looks good, but still missing the Odometer data)
  Added Range
  Added Battery SOH
  Added Gids
  Added Odometer (does not work)
  Added Time (does not work)
  Added Battery Temp (not completely sure it's right though)
  Moved 12V Battery to the last page (does not work)
  Removed Throttle (even though it's cool)

  V03
  cleaning out the old EV-CAN stuff

  V01 (based off Leaf SOC project code)
  Starting to play with the Car-CAN instead of the EV-CAN

##################################################################################
 
*/
