set_global_assignment -entity "Video_System" -library "lib_Video_System" -name IP_TOOL_NAME "Qsys"
set_global_assignment -entity "Video_System" -library "lib_Video_System" -name IP_TOOL_VERSION "18.0"
set_global_assignment -entity "Video_System" -library "lib_Video_System" -name IP_TOOL_ENV "Qsys"
set_global_assignment -library "lib_Video_System" -name SPD_FILE [file join $::quartus(sip_path) "../Video_System.spd"]
set_global_assignment -library "lib_Video_System" -name MISC_FILE [file join $::quartus(sip_path) "../../Video_System.qsys"]
