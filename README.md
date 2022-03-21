# LeafRange
A better range "guess-o-meter" for early Nissan Leaf cars

This project follows from my earlier [LeafSOCdisplay project](https://github.com/PaulKennett/LeafSOCdisplay), which grew from [CANa Display for Nissan LEAF](https://ev-olution.yolasite.com/CANa.php) © Copyright EV-OLUTION

The project uses an Ardunio Micro (or Mini Pro, or Mega2650) with a 160x128 pixel TFT display and a MCP2515 CAN-bus module. The display is mounted in a 3D printed frame that fits *inside* the main dash unit and covers the old range estimate (ie., the "guess-o-meter") with a less-wrong range based on battery capacity and temperature.

## Features

* a better range estimate in large freindly numerals (this estimate is not stupidly optomistic at the start of the drive)
* range chart that shows how well the estiimate matches reality (stores jouurney data in EEPROM over multiple drives)
* provides the difference between initial range estimate and current estimate plus distance driven
* shows battery pack temperature in Centigrate (more understandable than the temp "bars") 
* auto-starts a trip odo, and
* auto-starts a journey odo (multiple trips on a single charge)

The significance of the range chart feature is that it makes it much easier to adjust your driving to achieve a desired range.

### For example
Say you need to drive 75 km in your 2011 Leafster. You charge to "100%" which gives you 82km range (not that bad for an old gen 1.0 Leaf). The guess-o-meter would be saying "103km" which would be a bold-faced lie. 

So you head off, stopping at a few places along the way to collect vital supplies (ice creams, etc). About half way there you notice that the chart is showing you're actually trending below the initial estimate line - meaning you should start to ease off the speed a little so that you won't run short. 

On the other hand - if you see that your driving above the initial estimate line then you know you have plenty of capacity to spare so can afford to drive faster.

### Limitaions

This range estimate is not very sophisticated. It doesn't know about terrain, weather, driving conditions, car weight, etc. The key benefits are; (a) it's not stupidly optomistic at the start of a drive, and (b) the range chart allows you to easily adjust your driving behaviour to achieve a desired range.
