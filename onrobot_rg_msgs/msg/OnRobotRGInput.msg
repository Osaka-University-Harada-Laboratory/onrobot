# gFOF : Current fingertip offset in 1/10 millimeters. The value is a signed two's complement number.

uint16 gFOF

# gGWD : Current width between the gripper fingers in 1/10 millimeters.
#        The width is provided without any fingertip offset, as it is measured between the insides of the aluminum fingers.

uint16 gGWD

# gSTA : Current device status, which indicates the status of the gripper and its motion.
# Bit       - Name              - Description
# 0 (LSB)   - Busy              - High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
# 1         - Grip detected     - High (1) when an internal- or external grip is detected.
# 2         - S1 pushed         - High (1) when safety switch 1 is pushed.
# 3         - S1 trigged        - High (1) when safety circuit 1 is activated. The gripper will not move while this flag is high.
# 4         - S2 pushed         - High (1) when safety switch 2 is pushed.
# 5         - S2 trigged        - High (1) when safety circuit 2 is activated. The gripper will not move while this flag is high.
# 6         - Safety error      - High (1) when on power on any of the safety switch is pushed.
# 7 - 15    - Reserved          - Not used.

uint8 gSTA

# gWDF : Current width between the gripper fingers in 1/10 millimeters.
#        The set fingertip offset is considered.

uint16 gWDF
