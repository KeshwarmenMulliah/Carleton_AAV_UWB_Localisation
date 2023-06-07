# Carleton_AAV_UWB_Localisation
Scripts that use sensor readings from UWB transceivers to locate a tag relative to a moving autonomous vehicle



# Hardware used

 ![UWB_transceiver](https://user-images.githubusercontent.com/46099671/233884353-9472c9ec-ac07-46db-9e21-4eec008c861a.jpg)

# Setup Diagram (scenario used in)

![Setup_Diagram ](https://user-images.githubusercontent.com/46099671/233884616-0857e1c4-6325-461f-b684-9dc26836449e.jpg)

# Testing Results
## One stationary transceiver and one moving transceiver

![BackNForth_QUICK__3_10avg](https://user-images.githubusercontent.com/46099671/233884687-afe48113-a50d-4ea9-98d7-4ea3f781bd34.png)

![BackNForth_QUICK_2](https://user-images.githubusercontent.com/46099671/233884696-459b8089-951e-4785-94ea-d583b2bfad47.png)

## One stationary transceiver and one moving transceiver with Kalman Filter

![Results_with_simple_Kalman_Filter_2](https://user-images.githubusercontent.com/46099671/233884766-865d9053-f1bd-4c24-a35a-334a47cdfb66.png)

### Kalman Filter Implementation

![KF_framework ](https://user-images.githubusercontent.com/46099671/233885272-ad36a32d-5a17-4ed4-a58a-2194cfca2cae.jpg)

insert kalman filter parameters used

# UWB Position Tracking System

## Left/Right Side determination 

![LR_determination](https://user-images.githubusercontent.com/46099671/233885438-bb143e3e-e06b-407b-a596-09eba38b571f.jpg)

## Caution Zones 

![caution zone](https://user-images.githubusercontent.com/46099671/233885715-a85afb0d-de1f-4503-9436-7aaa932412f0.jpg)

# Localization 2D Python Simulation 

https://user-images.githubusercontent.com/46099671/233885795-8072aa9f-791b-4ca2-8c24-ff44e3749d3f.mp4

# Implementation of UWB Localization Algorithm on arduino car

