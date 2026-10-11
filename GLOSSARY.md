# Glossary

**Joint 1 / shoulder**: the joint on the base, turned by motor 1; it turns link 1.

**Joint 2 / elbow**: the joint between link 1 and link 2, turned by motor 2, which rides on link 1.

**Joint angle**: in degrees, counter-clockwise positive seen from above. θ1 (shoulder) is measured from the +X axis; θ2 (elbow) from the line of link 1, so 0 means the arm is straight. Not to be confused with motor revolutions or microsteps: a joint degree is 32 microsteps on the shoulder and 20 on the elbow.

**Origin**: the shoulder axis, (0, 0). X and Y are in mm.

**Home**: the pose where both endstops are pressed. After homing, the joint angles are known exactly: each joint is at its `home_angle`.

**Homed**: the flag in `/run/scara.pos` saying a joint's position is known. Disabling a motor, a driver reset, `run` or a reboot clears it; arm-level moves refuse to run without it.

**Joint limits**: `min_angle`..`max_angle` per joint, the range `scara joint` may move to, measured on the arm with a 5° margin.

**Elbow side**: most X/Y points can be reached two ways, mirror images about the line from the shoulder to the tip: the elbow bent counter-clockwise (+, θ2 > 0) or clockwise (−, θ2 < 0). Without a choice, the one inside the joint limits is used, the closer to the current pose if both are.

**Reach**: from |link1_mm − link2_mm| to link1_mm + link2_mm from the shoulder axis (29.0 to 361.8 mm), less what the joint limits cut off.
