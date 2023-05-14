# flapping_wing_tailsitter_simulation
---
This repo is about flight simulation of flapping wing tailsitter in matlab.
- Hit and RUN  
  Yes!  Just open Main.m hit the play buttom and you will get simulation by defalt setting.
  
- How to modify your own aircraft config?
  1. aircraft properties  
    You can modify your own aircraft properties such as aircraft weight and Cd,Cl etc in the Main.m.
  2. aircraft config (mixer)  
    Since every aircraft may be different(numbers of motors or servos...), you can edit allocation_matrix.m with your setting.  
    It's not necessary to use allocation_matrix.m, if you just want simulate the aircraft modle. Just make sure max_throttle, max_torque of your aircraft were set in Main.m.
    


