# SSBot
Arduino code for SSBot with Summer Springboard Fundamentals of Engineering at UCSD 

## Instructions

#### Short version
1. Download the code as a zipfile and extract it.
2. Open the Arduino IDE and install the libraries `NewPing`, `IRremote`, and `SafeString`.
3. Copy the `SSBotMotor` and `SSBotSensor` folders into `/path/to/sketch/directory/libraries`.
4. Restart the Arduino IDE for it to notice that there are new libraries.

#### Detailed version

1. Click the green button labeled "<> Code", then click the "Download ZIP" button.
![image](https://github.com/user-attachments/assets/7f47bb76-82c5-4c38-9abe-c20b2828fea4)

2. Open the Arduino IDE. In the sidebar, click on the icon that looks like a stack of books (<img width="41" alt="image" src="https://github.com/user-attachments/assets/93aaef93-c4bf-4a43-bece-f82806bfa6e0">) to open the library manager. Search and install these three libraries: `NewPing`, `IRremote`, and `SafeString`.
<img width="257" alt="image" src="https://github.com/user-attachments/assets/caed97f5-a49c-4579-885b-4b49ed56f98e">

3. In the top toolbar, go to **Arduino IDE > Settings** and copy the path written under "Sketchbook location".
<img width="559" alt="image" src="https://github.com/user-attachments/assets/d10c92ec-4b2a-45c8-af5c-2ce7f2bed4e3">
<img width="791" alt="image" src="https://github.com/user-attachments/assets/0ce6fe80-1baa-4a3c-ab02-4f36cf1fa945">

4. Open your file manager (File Explorer, Finder, Dolphin, etc.) and navigate to the location from the Arduino IDE "Sketchbook location" setting. You should see a `libraries` folder alongside any sketches you've saved previously; double click to go into the directory.

5. Open a **new** window or tab of your file manager and navigate to your Downloads folder. Find the the downloaded ZIP file called `SSBot-main.zip` and extract it (you can often just double-click on the ZIP file).

6. Open the newly extracted `SSBot-main` folder, then copy the `SSBotMotor` and `SSBotSensor` folders with `Ctrl+C` or by right clicking as shown below.
<img width="527" alt="image" src="https://github.com/user-attachments/assets/b9566b93-5fc0-4e9a-86d4-cfe178c144bd">

7. Go back to the other tab/window of your file manager (which should be in the `libraries` folder) and paste the `SSBotMotor` and `SSBotSensor` folders. You can now close your file managers tabs/windows.
   
8. Close the Arduino IDE and then re-open it. This will make it check the  `libraries` folder again so it notices the new directories we just added.
   
9. In the toolbar of Arduino IDE, go to **File > Examples**. You should see `SSBotMotor` and `SSBotSensor` all the way at the bottom. Open `SSBotSensor > MotorControlWithRemoteExample` and press the "Verify" (checkmark) button. If it compiles with no errors, you've installed the libraries correctly! If there is an error, call me over for help.

<img width="823" alt="image" src="https://github.com/user-attachments/assets/fe072827-d5c9-4f54-bb4a-cab70842a863">


