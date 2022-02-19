# rMCA : The Control mode
# rVCA : The Target vacuume field
#
#        The register allows for control of channel A.
#        The register is split into two 8-bit fields:
#        Bits 15-8        Bits 7-0
#        Control mode     Target vacuum
#
#        The Control mode field must contain one of these three values:
#
#        Value    Name    Description
#        0 (0x00) Release Commands the channel to release
#                         any work item and stop the pump,
#                         if not required by the other channel.
#        1 (0x01) Grip    Commands the channel to build up
#                         and maintain vacuum on this channel.
#        2 (0x02) Idle    Commands the channel to neither release nor grip.
#                         Workpieces may "stick" to the channel
#                         if physically pressed towards its vacuum cups,
#                         but the VG will use slightly less power.
#
#        The Target vacuum field sets the level of vacuum
#        to be build up and maintained by the chann el.
#        It is used only when the control mode is 1 (0x01) / Grip.
#        The target vacuum should be provided in % vacuum.
#        It should never exceed 80.
#
#        Examples:
#        Setting the register value 0 (0x0000)
#            will command the VG to release the work item.
#        Setting the register value 276 (0x0114)
#            will command the VG to grip at 20 % vacuum.
#        Setting the register value 296 (0x0128)
#            will command the VG to grip at 40 % vacuum.
#        Setting the register value 331 (0x014B)
#            will command the VG to grip at 75 % vacuum.
#        Setting the register value 512 (0x0200)
#            will command the VG to idle the channel.

uint16 rMCA
uint16 rVCA

# rVCB : Same as the one of channel A.

uint16 rMCB
uint16 rVCB
