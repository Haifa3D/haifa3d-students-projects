# Inference On uController

## Files

- [pitches.hpp](pitches.hpp) - defines tones to be used by buzzer.
- [forrest.hpp](forrest.hpp) - the trained classifier converted to cpp  
- [on_esp32_ble.ino](on_esp32_ble.ino) - real-time program to be flashed to uController
  - To change the number of modes - modify `NUM_ACTIONS` (also need to modify `actions` list accordingly).
  - To change mode actions - modify `actions` list - you can choose from the defined actions, or creaqte custom actions according to [this protocol](https://github.com/Haifa3D/haifa3d-hand-app/blob/master/Protocol.md)  
  - the code includes serial messages printing used for debug and can only be seen if connected to serail monitor  
