# Indoor_Positioning_DW3000
Illinois Institute of Technology ECE 441 Seinor Capstone Project

This project uses four MakerFabs DW3000 UWB ESP32 modules. Three modules are used as anchors, and one is used as the tag.

# User Manual
In this user manual, we will walk you through how to initialize the system, and then how to operate the system.
## Initial System Setup
### **Prerequisites**
This project utilizes Makerfabs DW3000 library for the ESP32. You can download it from here: 
  
https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
  
Install this library before proceeding with the rest of the instructions.
### **Antenna Delay Calibration**
If you are setting up the system for the first time with new devices, you will need to determine the antenna delays for the following combinations of devices.  
     
* Tag to Anchor 1  
* Tag to Anchor 2  
* Tag to Anchor 3  
* Anchor 1 to Anchor 2  
* Anchor 1 to Anchor 3  
* Anchor 2 to Anchor 3  
  
To calculate the antenna delays, you will place your anchor device and your tag at a measured distance. This distance will then be entered into the software global variable defined at the top of the sketch called “this_anchor_target_distance”. Once you have this set up, upload the anchor and tag codes to the respective devices. Open the serial monitor, and you will see the calculated anchor delays for the pair.    
  
Once you have determined all of the anchor antenna delays, you will need to enter them into the sketch for the anchors in the indoor positioning file. You will see a section near the top of the file beginning with the #if statement “#if anchornum == 1”. For each statement, that statement will contain the unique data that each anchor will contain. Inside of these statements, you will see several antenna delay definitions. For each pair of TX/RX delays, keep the value the same. The first delay you will see is for the tag. Enter the antenna delays you got between the tags and the anchors for each respective anchor. Next you will see the antenna delays between the anchors. Enter the antenna delays you determined earlier for each anchor. You will also see that for each anchor, the antenna delay for that anchor is also a value, for instance, the definitions for anchor 1 will contain the antenna delays for anchor 1 as well. For these values, the default value of 16384 will already be entered, so you do not need to change these values.    
### **Entering MAC Addresses**
In order for all of the devices to communicate with one another, you will need to find and define all of the devices’ MAC Addresses in both the files for the anchors and the tag. To find each device’s MAC Address, run the get_mac_address file on each device and record each one. Enter these MAC addresses into the anchor (lines 12 to 15) and tag (lines 17 to 24) codes.  
### **Tag Initialization**
In the code for the tag, you will need to enter your wifi credentials, including SSID and password. You can find these variables near the top of the file. You will also need to enter the IP address of the host computer that the positioning server will run on. Once these are entered, you can upload the code onto the board.  
### **Uploading anchor code to the ESP32**
To upload the anchor code onto the ESP32, all you will need to do is specify which anchor you are uploading to, defined in the top of the file in the macro called anchornum. This ranges from 1 to 3, for the three anchors you will be using for the system. Change this number for each anchor you upload the code to.  
### **Setting up Positioning Server**
In the positioning server file, there is only one value that needs to be adjusted. This variable is located near the top of the file, and is called UDP_IP. Enter in the local ip address of the system you are running the server on. This address should be the same address you entered into the tag.  
## How to use the system
### **System Setup**
To set up the system, power on all three anchors and place them a good distance apart from each other. Try to keep the anchors at the same height. If you need to place the anchors at different heights, you can measure out the heights and input them into the tag code inside the variable called anchor matrix. The Z height is the third variable on each line. A Z height of 0 is defined as the height you expect the tag will be most of the time. For instance, if the tag is to go in your pocket, that height will be defined as 0. If the anchor is placed on a table 1 foot higher than your pocket, then the z height will be defined as 1.    
  
Once you have determined the heights, you can now power on the tag. When the tag powers on, it will initialize the anchor positions using the automatic anchor calibration process we created.     
### **Defining Rooms**
With all three anchors and the tag powered up and ready, you can now start the positioning server by running the python file. On startup, you will be directed to the main menu. To define rooms, you will select option 2, add room. To select the option, enter the number 2 into the console. You will then be directed into the room setup. Read the instructions printed out, and enter 1 to add a room when you are done. Next, you will be entering the coordinates for the room. To do this, bring the tag to the first corner, and try to keep the height of the tag at the height you defined to be Z = 0. Next, enter 1 into the console to start the distance measurements. You will now see a progress bar detailing the progress of the measurements. Once that is complete, move to the next corner. It is important to move to each corner sequentially by following the walls. Do not cut corners. Enter 1 again to record the coordinates. Repeat these steps until you have recorded every corner of the room. Make sure not to repeat any corners. When you are done, enter 2 to finalize the room. You will now be instructed to enter the name of the room. When you finish the room, you can either choose to enter another room, or exit to the main menu. Exit to the main menu when you have completed entering all of the rooms.  
### **Start Positioning**
Finally, you can start the live positioning demonstration. To do this, enter 1 into the main menu. You will now see the rooms you defined in the previous step, as well as the position of your tag, and all three anchors. As you walk through the rooms, the room you are currently in will be outlined in red. You can now use the system as you wish.  
