#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
    Serial.print("CALLBACK: No empty slot for new controller.");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            return;
        }
    }
    Serial.print("CALLBACK: Controller disconnected, but not found.");
}

void setup() {
    Serial.begin(115200);

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void loop() {
int L1;
int R1;
int L2;
int R2;
int UP;
int DOWN;
int LEFT;
int RIGHT;
int Y;
int X;
int A;
int B;
int START;
int BACK;
int RX;
int RY;
int LX;
int LY;
    bool dataUpdated = BP32.update();
    if (!dataUpdated) {
        return; // No new data; exit the loop
    }

    for (auto myController : myControllers) {
        if (myController && myController->isConnected()) {
            // Access D-pad state
            int dpad = myController->dpad();
            UP = (dpad & 0x01) ? 1 : 0;
            DOWN=(dpad & 0x02) ? 1 : 0;
            RIGHT=(dpad & 0x04) ? 1 : 0;
            LEFT=(dpad & 0x08) ? 1 : 0;
           

            // Access button state
            int buttons = myController->buttons();
            A=(buttons & 0x0001) ? 1 : 0;
            B=(buttons & 0x0002) ? 1 : 0;
            X=(buttons & 0x0004) ? 1 : 0;
            Y=(buttons & 0x0008) ? 1 : 0;
            L1=(buttons & 0x0010) ? 1 : 0;
            R1=(buttons & 0x0020) ? 1 : 0;
            
            

            // Access misc button state
            int miscButtons = myController->miscButtons();
            START= (miscButtons & 0x02) ? 1 : 0;
            BACK= (miscButtons & 0x04) ? 1 : 0;
            

            // Access axis values
            int axisLX = myController->axisX();  // Left stick X-axis
            int axisLY = myController->axisY();  // Left stick Y-axis
            int axisRX = myController->axisRX(); // Right stick X-axis
            int axisRY = myController->axisRY(); // Right stick Y-axis

            axisLX=map(axisLX,-512,512,-127,127);
            LX=constrain(axisLX,-127,127);
            axisLY=map(axisLY,-512,512,-127,127);
            LY=constrain(axisLY,-127,127);
            axisRX=map(axisRX,-512,512,-127,127);
            RX=constrain(axisRX,-127,127);
            axisRY=map(axisRY,-512,512,-127,127);
            RY=constrain(axisRY,-127,127);
            RY=-RY;
            LY=-LY;

            if(RX>-25 && RX<25) RX=0;
            if(RY>-25 && RY<25) RY=0;
            if(LX>-25 && LX<25) LX=0;
            if(LY>-25 && LY<25) LY=0;
                  
            // // Access brake and throttle
            L2 = myController->brake();
            R2 = myController->throttle();

            
            
           
        }
    }
    Serial.println();
}