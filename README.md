# /MarlinFeatureAnnotations
This branch contains dataset of feature annotation for Marlin RC8 Release.
The annotation can be found in files with format:
```sh
\\&begin[Annotation Name]
    code;
\\&end[Annotation Name]
```
The annotation can be found by searching the following Annotation Names:

| Feature Name | Annotation Name |
|  ------ | ------ |
| Allow distinct factors for multiple extruders | DISTINCT_E_FACTORS|
| Arc Movement | ARC_SUPPORT|
|Auto Bed Leveling Bilinear | Auto_Bed_Leveling_Bilinear|
|Auto filament change | FILAMENT_CHANGE_FEATURE|
|BLTouch Sensor for Homing | BLTOUCH|
|Board | Board|
|Buzzer | Buzzer|
|Case Light Menu | CASE_LIGHT|
|Command Input Process | Command_Input_Process|
|Emergency Command Parser | EMERGENCY_PARSER|
|Endstop | Endstop|
|Extended Capabilities Report | Extended_Capabilities_Report|
|Extruder | Extruder|
|G20 Set units to inches | Inch_Mode_Support|
|Heated Bed | Bed_Heated|
|Hotend | HotEnd|
|Input and Output Process | IO_Handling|
|Linear Advance Extrusion Algorithm | LIN_ADVANCE|
|M108 Cancel Heat Up | Emergency_Cancel_Heatup|
|M149 set temperature units | Temperature_Units_Support|
|M155 Auto temp report | AUTO_REPORT_TEMPERATURES|
|M211 Enable/Disable Software Endstops | Control_Software_EndStop|
|M43 Pin report and debug | PINS_DEBUGGING|
|Minimum Stepper Pulse Option | MINIMUM_STEPPER_PULSE|
|Mixing Extruders | MIXING_EXTRUDER|
|Move to Destination | Move_To_Destination|
|Move to Home Position | Homing|
|Nozzle Clean | NOZZLE_CLEAN_FEATURE|
|Park Nozzle | NOZZLE_PARK_FEATURE|
|Power Supply | Power_Supply|
|Print Counter | PRINTCOUNTER|
|Print Job Timer | Print_Job_Timer|
|Servo Motor | Moter_Type_Servo|
|Single Nozzle Multiple Extruders | SINGLENOZZLE|
|Stepper Motor | Moter_Type_Stepper|
|Support for an RGB LED light using 3 pins | RGB_LED|
|Support for COREXY, COREXZ, and COREYZ | Support_COREXY_COREXZ_COREYZ|
|Support for multiple PWM fans | PWM|
|Support G2/G3 with R parameter | G2_G3_R_Parameter|
|Switching Extruders | SWITCHING_EXTRUDER|
|Temperature Control | Temperature|
|Temperature watch protection for heated bed | THERMAL_PROTECTION_BED|
|TMC2130 Silent StepStick support | TMC2130|
|WatchDog | WatchDog|

# Usage
The tool for analysis Feature Characteristics can be found in:
[FeatureCharacteristicsMeasurementTool] 


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>
   [FeatureCharacteristicsMeasurementTool]:<https://github.com/hui8958/FeatureCharacteristicsMeasurementTool>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
