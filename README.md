# Carleton_AAV_UWB_Localisation

The objective of this sub project is to design an emergency stop system that will be triggered when a pedestrian, carrying a tag as small as an Apple Air Tag, is on a potential collision course with our Accessible Autonomous Vehicle(AAV). This trigger will override the functions of the AAV and stop the vehicle. This type of emergency stop is designed to be  carried by elder people and pets with reduced spatial awareness will be compensated by the proposed method. Given the novelty of this system, significant research was conducted, and Ultra Wideband (UWB) technology was deemed as one of the solutions or this application.

# Hardware used
For the purpose of this sub-project the following UWB transceiver from Makerfab was utilized, namely the ESP 32 UWB with the DW1000 chip:

 ![UWB_transceiver](https://user-images.githubusercontent.com/46099671/233884353-9472c9ec-ac07-46db-9e21-4eec008c861a.jpg)

# Setup Diagram (scenario used in)
Two anchors will be fixed onto the AAV, and the tag will be carried by a person. The distance between the anchors will always be known and distance d1 and d2 are measured from the respectively UWB sensors (anchors in this case) . Since the distances make up on a triangle no matter what side the tag is on, using the acquired Light of Sight (LOS) distances and the cosine rule, the bearing of the tag can be computed and updated in real-time.

![Setup_Diagram ](https://user-images.githubusercontent.com/46099671/233884616-0857e1c4-6325-461f-b684-9dc26836449e.jpg)

# Testing Results
## One stationary transceiver and one moving transceiver

![BackNForth_QUICK__3_10avg](https://user-images.githubusercontent.com/46099671/233884687-afe48113-a50d-4ea9-98d7-4ea3f781bd34.png)

![BackNForth_QUICK_2](https://user-images.githubusercontent.com/46099671/233884696-459b8089-951e-4785-94ea-d583b2bfad47.png)

## One stationary transceiver and one moving transceiver with Kalman Filter

![Results_with_simple_Kalman_Filter_2](https://user-images.githubusercontent.com/46099671/233884766-865d9053-f1bd-4c24-a35a-334a47cdfb66.png)

### Kalman Filter Implementation

A simple Kalman filter employs the concept of a state observer and works in two 
steps: 
1. Prediction 
2. Correction
   
The Kalman filter requires an initial estimate of the variables to be approximated. The input is then interpreted as a previous estimate which is also known as “a priori” state 
estimate. A prediction of the next state is then made using the previous state estimate and the system’s model equation which describes the dynamics of the system and how all the variablesare related and change with time. Consequently, this mathematical prediction is used in conjunction with data recorded from the sensor to compute the Kalman gain which dictates the contribution of the sensor data when processing the next estimate, which is also called the “posteriori” estimat. The latter is the correction step.

![KF_framework ](https://user-images.githubusercontent.com/46099671/233885272-ad36a32d-5a17-4ed4-a58a-2194cfca2cae.jpg)

insert kalman filter parameters used

# UWB Position Tracking System

## Left/Right Side determination 
In an attempt to determine the position (Left or Right) of the tag relative to the AAV, the variation of angle ∝ with the motion of the tag around the anchors was investigated. It was observed that, in a clockwise motion around the vehicle, the size of the angle increases as long as the tag is on the right side and decreases if the tag is on the left side. For an anti-clockwise motion, it is vice versa, meaning that the angle 𝛼 decreases when it is on the right side of the vehicle and increases as long as it is on the left side of the vehicle.

![LR_determination](https://user-images.githubusercontent.com/46099671/233885438-bb143e3e-e06b-407b-a596-09eba38b571f.jpg)

## Caution Zones 
Caution zones were established by including categorizing certain ranges around the vehicle. The AAV’s behaviour will change in response to the zone that the tag (or person carrying the tag)is detected in at any point in time.

![caution zone](https://user-images.githubusercontent.com/46099671/233885715-a85afb0d-de1f-4503-9436-7aaa932412f0.jpg)

# Localization 2D Python Simulation 

After implementing the caution zones and the algorithm discussed, a python simulation was built and ran to ensure that the script would output the correct bearing of the tag relative to the anchors fixed onto the AAV and hence can accurately detect the position of the tag in real time and take action accordingly while both the AAV and tag is moving. The code used is also available above and  the screen recording of the simulation is also pasted below.The implemented algorithm described is not always accurate as the recoding of the 2D simulation will depict. However, by increase the amount of the readings processed every second and taking an average, the inconsistencies should be reduced at the expense of potentially using more computing power and hence hindering the capability of the tag-based emergency stop system to give real time feedbacks. Additionally, the script will only be operational and determine if the tag is on the right or left side of the AAV if at least the AAV O the tag is moving.

https://user-images.githubusercontent.com/46099671/233885795-8072aa9f-791b-4ca2-8c24-ff44e3749d3f.mp4

# Implementation of UWB Localization Algorithm on arduino car

The UWB localisation python script was modified and segregated into a python class before adapting it to the ROS2 environment constructed by another team member. Since the UWB transceiver received by the end of March was not readily compatible with the other two existing transceivers due to the distinct chipset mentioned previously, only two transceivers(the anchor fixed onto the car and the tag) was used to showcase thecapabilities of the tag-based emergency stop. Consequently, the intended functionalities such as the real time bearing computations and the left/right determination algorithm, was not successfully tested. Nevertheless, the Kalman filter was used and the UWB zones and the feedback system were tested. The UWB zones were showcased by the use of a single LED, given the power limitations of the small robot car. The LED lights up red when the tag is the collision zone, yellow in the caution zone and green in the warning zone and beyond.

![image](https://github.com/KeshwarmenMulliah/Carleton_AAV_UWB_Localisation/assets/46099671/814b9ea2-bb6d-4479-8d89-cae373bf78b8)

