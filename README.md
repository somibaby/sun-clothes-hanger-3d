# Solar Telescoping Clothes Hanger — MATLAB Simulation

A wall-mounted hanger with two telescoping arms. A sun sensor decides when to push the clothes out into the sun and when to pull them back in (dark or rain).

- **State machine:** RETRACTED → EXTENDING → EXTENDED → RETRACTING
- **Hysteresis:** extend above 20,000 lux, retract below 8,000 lux
- **Debounce:** condition must hold for 60 s before the arms move
- **Safety:** rain overrides everything and retracts immediately

Run `sun_clothes_hanger_3d.m` in MATLAB to see the plots and 3D animation.
