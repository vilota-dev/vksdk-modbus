cmake_minimum_required(VERSION 3.10)
project(main)

# flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -MD -g3 -Wall -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -MD -g3 -std=c++17 -Wno-reorder -Werror -pthread")

include_directories(
	${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/LINUX/port
	${PROJECT_SOURCE_DIR}/modbus
	${PROJECT_SOURCE_DIR}/modbus/include
	${PROJECT_SOURCE_DIR}/modbus/rtu
	${PROJECT_SOURCE_DIR}/modbus/ascii
	# ${PROJECT_SOURCE_DIR}/freemodbus/modbus/tcp
	${PROJECT_SOURCE_DIR}/freemodbus/modbus/functions
	)

set(MB_AICAM_SRC
    main.cpp
)

set(MB_LIB_SRC 
    LINUX/port/portevent.c
    LINUX/port/portother.c
	LINUX/port/portserial.c
	LINUX/port/porttimer.c
	modbus/mb.c
	modbus/rtu/mbcrc.c
	modbus/rtu/mbrtu.c
	modbus/tcp/mbtcp.c
	modbus/ascii/mbascii.c
	modbus/functions/mbfunccoils.c
	modbus/functions/mbfuncdiag.c
	modbus/functions/mbfuncholding.c
	modbus/functions/mbfuncinput.c
    modbus/functions/mbfuncother.c
	modbus/functions/mbfuncdisc.c
	modbus/functions/mbutils.c
	)

set(MB_COMM_AICAM_SRC
    ${MB_LIB_SRC}
    ${MB_AICAM_SRC}
)

set(CapnProto_DIR "/usr/local/lib/cmake/CapnProto")
set(vk_sdk_DIR "/opt/vilota/lib/cmake/vk_sdk")

find_package(vk_sdk REQUIRED)



add_executable(main ${MB_COMM_AICAM_SRC})
target_link_libraries(main PUBLIC pthread gpiod CapnProto::capnp PRIVATE vilota::vk_sdk)