#
# Property of ABB Vasteras/Sweden. All rights reserved.
# Copyright 2003.
#
# Startup.cmd script for 5.0
#

startup_log -file $INTERNAL/startup.log

# Base support initialization start.

ifvc -label VC_NO_TRCREC
invoke2 -entry trcrec_init -format int -int1 100000
#VC_NO_TRCREC

baseinit

# Initialize and enable system dump service
sysdmp_init -max_dumps 3 -delay 5000 -dir $INTERNAL/SYSDMP -compr no
ifvc -label VC_SKIP_SYSDMPTASK
task -slotname sysdmpts -slotid 82 -pri 253 -vwopt 0x1c -stcks 25000 -entp sysdmpts_main -auto
#VC_SKIP_SYSDMPTASK
sysdmp_add -class trcrec
sysdmp_add -show print_spooler_show_buffer
uprobe_init -points 50000 -pretrig_points 45000 -trace_buf_sz 10000

config -version 5 -revision 0

# Start event log 
task -slotname elogts -slotid 1 -pri 90 -vwopt 0x1c -stcks 7000 -entp elog_main -auto
synchronize -level task
go -level task

# Enable Service Box in devices
invoke -entry RemoteDiagnosticInit -noarg -nomode

task -slotname delay_high -entp delay_ts -pri 60 -vwopt 0x1c -stcks 8000 -nosync -auto -wait_ready 10
task -slotname delay_medium -entp delay_ts -pri 78 -vwopt 0x1c -stcks 8000 -nosync -auto -wait_ready 10
task -slotname delay_low -entp delay_ts -pri 140 -vwopt 0x1c -stcks 8000 -nosync -auto -wait_ready 10

basenew
# Base support initialization finished.

ifvc -label VC_NO_MC_PSU_UPGRADE
### Check MC board firmware version
invoke -entry mcfirmw_upgrade -noarg -nomode

### Check PSU board firmware version
invoke -entry psu_upgrade -noarg -nomode
#VC_NO_MC_PSU_UPGRADE

iomgrinstall -entry simFBC -name /simfbc
creat -name /simfbc/SIM_SIM1: -pmode 0
creat -name /simfbc/SIM_SIM2: -pmode 0

invoke -entry read_init

# VC SKIP #2
ifvc -label VC_RCC_SKIP1
invoke -entry fpgar11_init
iomgrinstall -entry rccfbc -name /rccfbc
creat -name /rccfbc/LOC_LOC1: -pmode 0

task -slotname LOCALBUS -entp read_ts -pri 72 -vwopt 0x1c -stcks 10000 -nosync -auto
readparam -devicename /LOC_LOC1:/bus_read -rmode 1 -buffersize 100
goto -label VC_RCC_SKIP2

#VC_RCC_SKIP1
creat -name /simfbc/LOC_LOC1: -pmode 0 -errlabel ERROR_RCCFBC
#VC_RCC_SKIP2

#ERROR_RCCFBC

iomgrinstall -entry UserSio -name /usersio
creat -name /usersio/SIO1: -pmode 0

# Fieldbus command interface
ifvc -label VC_SKIP_FCI

iomgrinstall -entry Fci -name /fci
creat -name /fci/FCI1: -pmode 0

task -slotname fcits -slotid 80 -pri 80 -vwopt 0x1c -stcks 10000 -entp fcits -auto

#VC_SKIP_FCI

fileexist -path $INTERNAL/opt_l0.cmd -label LOAD_CMD
goto -label NEXT_STEP
#LOAD_CMD
include -path $INTERNAL/opt_l0.cmd
#NEXT_STEP

init -resource eio
init -resource sio
init -resource motion
init -resource cab

task -path $(SEARCH)$(HPBIN)/alarmts  \
-slotname alarmts -slotid 16 -pri 80 -vwopt 0x1c -stcks 7000 \
-entp alarmts -auto

ifvc -label VC_CABSUPTS_SKIP
task -path $(SEARCH)$(HPBIN)/cabsupts  \
-slotname cabsupts -slotid 62 -pri 135 -vwopt 0x1c -stcks 7000 \
-entp cabsup_main -auto
#VC_CABSUPTS_SKIP

task -path $(SEARCH)$(HPBIN)/pstopts \
-slotname pstopts -pri 30 -vwopt 0x1c -stcks 5000 \
-entp pstopts_main -auto -nosync

task -path $(SEARCH)$($(HPBIN))/rhaltts \
-slotname rhaltts -pri 30 -vwopt 0x1c -stcks 5000 \
-entp rhaltts_main -auto -nosync

task -slotname eiocrsts -slotid 56 -pri 74 -vwopt 0x1c -stcks 10000 \
-entp eiocrsts -auto

task -slotname eiots -slotid 54 -pri 70 -vwopt 0x1c -stcks 10000 \
-entp eiots -auto -slots 2

task -slotname eioadmts -slotid 55 -pri 76 -vwopt 0x1c -stcks 12000 \
-entp eioadmts -auto

reconfigure -resource eio
io_synchronize -timeout 30000

reconfigure -resource sio

task -slotname safevtts  -slotid 20 -pri 20 -vwopt 0x1c -stcks 10000 \
-entp safevtts -auto

task -slotname safcycts  -slotid 29 -pri 80 -vwopt 0x1c -stcks 10000 \
-entp safcycts -auto

task -slotname sysevtts  -slotid 30 -pri 95 -vwopt 0x1c -stcks 30000 \
-entp sysevtts_main -auto

task -slotname ipolts0 -slotid 65 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname ipolts1 -slotid 66 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname ipolts2 -slotid 67 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname ipolts3 -slotid 68 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname ipolts4 -slotid 91 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname ipolts5 -slotid 92 -pri 90 -vwopt 0x1c -stcks 50000 \
-entp ipolts_main -auto

task -slotname statmats -slotid 64 -pri 6 -vwopt 0x1c -stcks 24000 \
-entp statmats -auto

task -slotname compliance_masterts -slotid 124 -pri 8 -vwopt 0x1c -stcks 30000 \
-entp compliance_masterts -auto

task -slotname sens_memts -slotid 89 -pri 150 -vwopt 0x1c -stcks 20000 \
-entp sens_memts -auto

task -slotname servots0 -slotid 69 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname servots1 -slotid 71 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname servots2 -slotid 73 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname servots3 -slotid 75 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname servots4 -slotid 93 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname servots5 -slotid 95 -pri 4 -vwopt 0x1c -stcks 65000 \
-entp servots -auto -slots 2

task -slotname evhats -slotid 117 -pri 3 -vwopt 0x1c -stcks 20000 \
-entp evhats -auto

task -slotname refmats -slotid 52 -pri 2 -vwopt 0x1c -stcks 30000 \
-entp refmats -auto

ifvc -label VC_SKIP_SIS
task -slotname sismats -slotid 63 -pri 106 -vwopt 0x1c -stcks 12500 \
-entp sismats -auto 
#VC_SKIP_SIS

task -slotname bcalcts -slotid 44 -pri 200 -vwopt 0x1c -stcks 45000 \
-entp bcalcts -auto

task -slotname logsrvts -slotid 51 -pri 100 -vwopt 0x1c -stcks 30000 \
-entp logsrvts -auto

ifvc -label RW_MOTION_END
task -slotname com_rec_fdbts -slotid 77 -pri 2 -vwopt 0x1c -stcks 5000 -entp com_rec_fdbts -auto

task -slotname axc_failts -slotid 28 -pri 80 -vwopt 0x1c -stcks 7000 -entp axc_failts -auto

#RW_MOTION_END

task -slotname peventts -slotid 21 -pri 65 -vwopt 0x1c -stcks 45000 \
-entp peventts -auto

task -slotname rloadts -slotid 45 -pri 105 -vwopt 0x1c -stcks 40000 \
-entp rloadts -auto

task -slotname pusts -slotid 84 -pri 130 -vwopt 0x1c -stcks 40000 \
-entp pusts_main -auto

task -slotname barts -slotid 79 -pri 140 -vwopt 0x1c -stcks 40000 \
-entp barts_main -auto

task -slotname rlcomts -slotid 27 -pri 100 -vwopt 0x1c -stcks 12000 \
-entp rlcomts_main -auto

task -slotname rlsocketts -slotid 108 -pri 99 -vwopt 0x1c -stcks 8000 \
-entp rlsocketts_main -auto

task -slotname rlsocketts_rec -slotid 120 -pri 99 -vwopt 0x1c -stcks 8000 \
-entp rlsocketts_rec_main -auto -noreg

task -slotname cabts -slotid 17 -pri 100 -vwopt 0x1c -stcks 78000 \
-entp cabts_main -auto

task -slotname hpjts -slotid 22 -pri 99 -vwopt 0x1c -stcks 45000 \
-entp hpj_main -auto

ifvc -label VC_NETCMD_SKIP
task -slotname elog_axctsr -pri 100 -vwopt 0x1c -stcks 9000 \
-entp elog_axctsr -auto -nosync
#VC_NETCMD_SKIP

synchronize -level task
delay -time 500
go -level task

sysdmp_add -show alarm_show_failed

ifvc -label VC_AXC_SKIP
netcmd_run
#VC_AXC_SKIP

#PNT new -entry purge_sync
#PNT delay -time 5000

fileexist -path $INTERNAL/option.cmd -label LOAD_CMD
goto -label NEXT_STEP
#LOAD_CMD
include -path $INTERNAL/option.cmd
#NEXT_STEP

ifvc -label VC_AXC_SKIP

print -text "Check AXC board firmware version ..."
invoke -entry netcmd_upgrade -arg 0 -strarg "axcfirmw_upgrade" -nomode
invoke -entry axcfirmw_devices_install_hook -nomode
print -text "Check drive unit board firmware version ..."
invoke -entry netcmd_upgrade -arg 0 -strarg "drive_unit_firmw_upgrade" -nomode
invoke -entry drive_unit_firmw_devices_install_hook -nomode

#VC_AXC_SKIP
new -resource motion
delay -time 1500
connect_dependings -object SYS_STORED_OBJ_CAB -instance 0 -man 0 -ipm 0 -servo 0 -ipol 0 -statma 0

# syncronization (setsync) to be used with network systems only
# setsync
if -var 52 -value 1 -label SYNCROB
goto -label NOSYNC
#SYNCROB
# syncronization (setsync) to be used with network systems only
setsync
#NOSYNC

new -resource cab

#task -slotname spoolts -slotid 34 -pri 140 -vwopt 0x1c -stcks 3000 \
#-entp spool_main -auto

# RobAPI communication and event tasks

# Uncomment invoke below to disable tcp_nodelay for robapi on the lan interface. 
# tcp_nodelay enabled, causes packets to be flushed on to the network more frequently.
# Default is tcp_nodelay enabled on all robapi interfaces.
# For other robapi interface use strarg={"lan" | "service" | "tpu"}.
# invoke -entry robcmd_set_tcp_nodelay -arg 0 -strarg "lan" -nomode

task -slotname robdispts -slotid 58 -pri 125 -vwopt 0x1c -stcks 32000 \
-entp robdispts -auto -noreg

task -slotname robesuts -slotid 59 -pri 125 -vwopt 0x1c -stcks 18000 \
-entp robesuts -auto -noreg

task -slotname robesuts_p -slotid 85 -pri 125 -vwopt 0x1c -stcks 15000 \
-entp robesuts_p -auto -noreg

task -slotname robesuts_hp -slotid 123 -pri 125 -vwopt 0x1c -stcks 15000 \
-entp robesuts_hp -auto -noreg

task -slotname robcmdts -slotid 60 -pri 125 -vwopt 0x1c -stcks 23000 \
-entp robcmdts -auto -noreg

task -slotname robayats -slotid 61 -pri 125 -vwopt 0x1c -stcks 7000 \
-entp robayats -auto -noreg

task -slotname robrspts -slotid 57 -pri 124 -vwopt 0x1c -stcks 15000 \
-entp robrspts -auto -noreg

task -slotname robmasts -slotid 13 -pri 126 -vwopt 0x1c -stcks 48000 \
-entp robmasts -auto -noreg

task -slotname dipcts -slotid 122 -pri 126 -vwopt 0x1c -stcks 18000 \
-entp dipcts -auto -noreg

# Start stream support 
task -slotname streamts -slotid 81 -pri 120 -vwopt 0x1c -stcks 10000 \
-entp streamts -auto

# Start the stream read threads
task -slotname stream_readts1 -slotid 104 -entp stream_readts -pri 120 -vwopt 0x1c -stcks 8000 -auto
task -slotname stream_readts2 -slotid 105 -entp stream_readts -pri 120 -vwopt 0x1c -stcks 8000 -auto

fileexist -path $INTERNAL/opt_l2.cmd -label LOAD_CMD
goto -label NEXT_STEP
#LOAD_CMD
include -path $INTERNAL/opt_l2.cmd
#NEXT_STEP

task -slotname ns_send -slotid 78 -entp robnetscansendts -pri 98 -vwopt 0x1c \
-stcks 15000 -auto -noreg

ifvc -label NETSCANSKIP
task -slotname ns_receive -slotid 87 -entp robnetscanreceivets -pri 99 -vwopt 0x1c \
-stcks 10000 -auto -noreg
#NETSCANSKIP

synchronize -level task
cab_startup_ready
go -level task

### Notify CHANREF data is valid for PSC operation
invoke -entry pscio_chanref_valid -noarg -nomode

ifvc -label VTSPEED
goto -label VTSPEED_END
#VTSPEED
# Set Virtual Time Speed to 100% for VC at end of startup-script
invoke -entry VTSetSpeed -arg 100 -nomode

#VTSPEED_END

ifvc -label VCSHELL
goto -label VCSHELL_END
#VCSHELL
task -slotname vcshell -entp vcshell -pri 120 -vwopt 0x1c -stcks 15000 -auto -noreg
#VCSHELL_END

fileexist -path $INTERNAL/opt_l3.cmd -label LOAD_CMD
goto -label NEXT_STEP
#LOAD_CMD
include -path $INTERNAL/opt_l3.cmd
#NEXT_STEP

### Reset firmware upgrade error monitoring. No device upgrade methods may be invoked after this point.
upgrade_warm_start_completed

# Set priority for safety hooks
set_hook_exec_prio -sysstop 15 -sysfail 15 -syshalt 15

# Notify user if this is an unofficial RobotWare release in a customer system
unofficial_rw_notification

# Add IPC to the sysdump
sysdmp_add -show ipc_show
# Add elog to the sysdump (dumps elog messages to a text file)
sysdmp_add -class elog
# Add semaphore info to the sysdump
sysdmp_add -show sysdmp_sem_show
# Add devices info to system dump service
sysdmp_add -show devices_show
# Add ipm to the sysdump
sysdmp_add -show ipm_show_data
# Add pgm info to system dump service
sysdmp_add -show cab_pgm_show
# Add pgmrun info to system dump service
sysdmp_add -show cab_pgmrun_show
# Add pgmexe info to system dump service
sysdmp_add -show cab_pgmexe_show_all
# Add robdisp info to system dump service
sysdmp_add -show robdisp_show
# Add uprobe log to system dump service
sysdmp_add -class uprobe
# Add ethernet packet log to system dump service
sysdmp_add -source ethernet -show packet_show -stop packet_stop_log -start packet_start_log
# Add FlexPendant dump info to system dump service
sysdmp_add -source fpcmd -save fpcmd_diagnostics

# Generate sysdump on "refma underrun"
sysdmp_trigger_add -elog_domain 5 -elog_number 226
# Generate sysdump on "Communication lost with Drive Module"
sysdmp_trigger_add -elog_domain 3 -elog_number 9520

# Additional log files to be copied to the sysdump
sysdmp_add_logfile -move 0 -file "$INTERNAL/install.log"
sysdmp_add_logfile -move 0 -file "$INTERNAL/startup.log"
sysdmp_add_logfile -move 0 -file "$INTERNAL/pf_info.log"
sysdmp_add_logfile -move 1 -file "$INTERNAL/tt.log"

# Add EIO to system dump service
sysdmp_add -show eio_sysdmp

# Include command file for additional logging mechanisms 
fileexist -path $SYSTEM/service_debug.cmd -label LOAD_CMD
goto -label NEXT_STEP
#LOAD_CMD
include -path $SYSTEM/service_debug.cmd
sysdmp_add_logfile -move 0 -file "$SYSTEM/service_debug.cmd"
#NEXT_STEP

systemrun
