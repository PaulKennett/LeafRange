# LeafRange
A better range "guess-o-meter" for early Nissan Leaf cars

![LeafRange display installed](/../main/LeafRange-600x264-20220425.jpg)

This project follows on from (and compliments) my earlier [LeafSOCdisplay](https://github.com/PaulKennett/LeafSOCdisplay) project, which grew from [CANa Display for Nissan LEAF](https://ev-olution.yolasite.com/CANa.php) © Copyright EV-OLUTION

The project uses an Ardunio Micro (or Mini Pro, or Mega2650) with a 160x128 pixel TFT display and a MCP2515 CAN-bus module to passively listen to the Car-CAN data bus and use the info to provide a more rational range estimate. The display is mounted in a 3D printed frame that fits *inside* the main dash unit and covers the old range estimate (ie., the "guess-o-meter") with a better range estimate based on battery capacity and temperature. There's also an optional frame that allows the display to be mounted in the corner of the dash, so you can see both the standard Guess-O-Meter and my range estimate.

YouTube drive demo: https://www.youtube.com/watch?v=xt2o3SWRRq8

YouTube hardware show & tell: https://www.youtube.com/watch?v=k4QYvayp_lU

## Features

* better range estimate in large friendly numerals (this estimate is not stupidly optomistic at the start of the drive)
* range estimate does not disappear when the battery dips below 20% (to be replaced by the flashing ---). You get an accurate range estimate down to 0 km. However; there's no buffer below that - so you can't drive another 10 km on "empty" like in a petrol car.
* range chart that shows how well the estiimate matches reality (stores jouurney data in EEPROM over multiple drives)
* shows battery pack temperature in Centigrate (more understandable than the temp "bars") when the battery is cold or hot
* auto-starts a journey odo (multiple trips on a single charge)
* range calculation factors in: battery State of Charge, battery State of Health, and battery temperature (see https://github.com/PaulKennett/LeafRange/wiki/Notes}

The significance of the range chart feature is that it makes it much easier to adjust your driving to achieve a desired range.

### For example
Say you need to drive 75 km in your 2011 Leafster. You charge to "100%" which gives you a rage estimate of 82km range on my LeafRange display (not that bad for an old gen 1.0 Leaf). The guess-o-meter would be saying something like "103km" which would be a bold-faced lie. 

So you head off, stopping at a few places along the way (my display saves range and odo data along the way). About half way there you notice that the chart is showing you're actually trending _below_ the initial estimate line - meaning you should start to ease off the speed a little so that you won't run short. 

On the other hand - if you see that you're driving _above_ the initial estimate line then you know you have plenty of capacity to spare so can afford to drive faster.

### Limitations

This range estimate is not very sophisticated. It doesn't know about terrain, weather, driving conditions, car weight, etc. The key benefits are; (a) it's not stupidly optomistic at the start of a drive, and (b) the range chart allows you to easily adjust your driving behaviour to achieve a desired range.

## Hardware

1. Arduino Micro, Mini Pro, or Mega2560 (use a Mega2560 for development, use a Micro or Mini Pro for use in the car as it boots up much faster and uses less power)
2. MCP2515 (CAN bus module)
3. 1.8 inch TFT display 160x128 pixels (like this https://www.aliexpress.com/item/4000016800528.html)
4. TXS0108E 8 Channel Logic Level Shifter (https://www.aliexpress.com/item/4001131536726.html)
5. 10 Position Flat Flex Cable Assembly Receptacle to Solder Tab 4.00" (101.60mm) https://www.digikey.co.nz/product-detail/en/te-connectivity-amp-connectors/A9AAT-1004F/A9AAT-1004F-ND/137494
6. 5V to 3.3V DC-DC Step-Down Power Supply module (https://www.aliexpress.com/item/4001039310587.html)
7. 70CM 10P IDC Flat Ribbon DATA Cable 2.54mm (https://www.aliexpress.com/item/32891063802.html)
8. Male DC3 10 Pin 2x5Pin Right Angle Double Row Pitch 2.54mm (https://www.aliexpress.com/item/32945804425.html)
9. 3D printed frame for the display (https://www.myminifactory.com/object/3d-print-nissan-leaf-range-graph-display-154555)
10. 3D printed project box DRAFT (https://www.tinkercad.com/things/lRlt66oEZCM-leafrange-box-v16 - I'll provide a better link soon}
11. PBC motherboard for a Pro Micro or Pro Mini Arduino (Gerber file provided above or https://oshwlab.com/paulkennett/LeafRange-for-Micro)
12. PCB for level shifter module (Gerber file provided above - you could use https://jlcpcb.com/ to make them)
13. OBD2 plug (like this https://www.aliexpress.com/item/32865761651.html)
14. 97% Mini DC-DC 15V 12V 9V 7.4V to 5V 4A Step-down Power Module https://www.aliexpress.com/item/4001313603033.html

[Parts list](https://docs.google.com/spreadsheets/d/e/2PACX-1vS2Nyihl8SLagP5YnfhvbngJ51losEeNEex3Urd8iOHfg4pMDlcg-ZONYQ7dYSHcnsXQ2ahFD6JTfKS/pubhtml?gid=487315142&single=true) with indicative NZ$ and US$ prices (as at 11 May 2022)

## Relation to the LeafSOC project

The LeafRange project started out as a fork of the LeafSOC project (with a larger 3.3V TFT display to replace the 5V OLED display). So the LeafRange motherboard has been designed to also be used for the LeafSOC project. The on-board jumpers allow you to switch between Car-CAN (for LeafRange) or EV-CAN (for LeafSOC). 

Also: 
* the latest LeafRange motherboard also has the correct dimensions for the OBD2 plug holes. T
* the LeafSOC project uses a 5V OLED display so you can leave the 5V-to-3.3V DC-DC converter off  


## Todo
- see the Wiki Todo page https://github.com/PaulKennett/LeafRange/wiki/Todo
