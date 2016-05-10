INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

 
# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(FreeRTOSDir $ENV{FREERTOS_DIR})

# DEBUG LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${FreeRTOSDir}/platform/devices/MCIMX7D/linker/gcc/MCIMX7D_M4_ocram.ld  -static")

# RELEASE LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${FreeRTOSDir}/platform/devices/MCIMX7D/linker/gcc/MCIMX7D_M4_ocram.ld  -static")

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x400  -Xlinker --defsym=__heap_size__=0x200")

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x400  -Xlinker --defsym=__heap_size__=0x200")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

# C MACRO
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DCPU_IMX7D_M4")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DCPU_IMX7D_M4")

# CXX MACRO

# INCLUDE_DIRECTORIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
	INCLUDE_DIRECTORIES(${ProjDirPath}/../board)
	INCLUDE_DIRECTORIES(${ProjDirPath}/../drivers)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/examples/imx7_colibri_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices/MCIMX7D/include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices/MCIMX7D/startup)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/drivers/inc)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/utilities/inc)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/examples/imx7_colibri_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/middleware/multicore/open-amp)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../board)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../drivers)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/examples/imx7_colibri_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices/MCIMX7D/include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/devices/MCIMX7D/startup)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/drivers/inc)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/platform/utilities/inc)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/examples/imx7_colibri_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4)
    INCLUDE_DIRECTORIES(${FreeRTOSDir}/middleware/multicore/open-amp)
ENDIF()

# ADD_EXECUTABLE
ADD_EXECUTABLE(taq
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h"
    "${FreeRTOSDir}/platform/devices/MCIMX7D/startup/gcc/startup_MCIMX7D_M4.S"
    "${FreeRTOSDir}/platform/drivers/inc/gpio_imx.h"
    "${FreeRTOSDir}/platform/drivers/src/gpio_imx.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c"
    "${ProjDirPath}/../FreeRTOSConfig.h"
    "${ProjDirPath}/../main.c"
    "${ProjDirPath}/../pid.h"
    "${ProjDirPath}/../pid.c"
    "${ProjDirPath}/../kalman.h"
    "${ProjDirPath}/../kalman.c"
    "${ProjDirPath}/../board/board.c"
    "${ProjDirPath}/../board/board.h"
    "${ProjDirPath}/../board/pin_mux.c"
    "${ProjDirPath}/../board/pin_mux.h"
    "${ProjDirPath}/../board/gpio_pins.h"
    "${ProjDirPath}/../board/gpio_pins.c"
    "${ProjDirPath}/../drivers/seg_timers.c"
    "${ProjDirPath}/../drivers/seg_timers.h"
	"${ProjDirPath}/../board/gpio_pins.h"
    "${ProjDirPath}/../board/gpio_pins.c"
    "${ProjDirPath}/../drivers/i2c_xfer.c"
    "${ProjDirPath}/../drivers/i2c_xfer.h"
    "${ProjDirPath}/../drivers/motor_driver.c"
    "${ProjDirPath}/../drivers/motor_driver.h"
    "${ProjDirPath}/../drivers/mpu6000.c"
    "${ProjDirPath}/../drivers/mpu6000.h"
    "${ProjDirPath}/../drivers/pca9685.c"
    "${ProjDirPath}/../drivers/pca9685.h"
    "${ProjDirPath}/../drivers/ads1015.c"
    "${ProjDirPath}/../drivers/ads1015.h"
    "${FreeRTOSDir}/platform/drivers/src/uart_imx.c"
    "${FreeRTOSDir}/platform/drivers/inc/uart_imx.h"
    "${FreeRTOSDir}/platform/drivers/src/i2c_imx.c"
    "${FreeRTOSDir}/platform/drivers/inc/i2c_imx.h"
    "${FreeRTOSDir}/platform/drivers/src/mu_imx.c"
    "${FreeRTOSDir}/platform/drivers/inc/mu_imx.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/croutine.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/event_groups.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/FreeRTOS.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/list.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/mpu_wrappers.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/portable.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/projdefs.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/queue.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/semphr.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/StackMacros.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/task.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/include/timers.h"
    "${FreeRTOSDir}/platform/drivers/inc/ccm_analog_imx7d.h"
    "${FreeRTOSDir}/platform/drivers/inc/ccm_imx7d.h"
    "${FreeRTOSDir}/platform/drivers/inc/rdc.h"
    "${FreeRTOSDir}/platform/drivers/inc/rdc_defs_imx7d.h"
    "${FreeRTOSDir}/platform/drivers/inc/wdog_imx.h"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/croutine.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/event_groups.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/list.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/queue.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/tasks.c"
    "${FreeRTOSDir}/rtos/FreeRTOS/Source/timers.c"
    "${FreeRTOSDir}/platform/drivers/src/ccm_analog_imx7d.c"
    "${FreeRTOSDir}/platform/drivers/src/ccm_imx7d.c"
    "${FreeRTOSDir}/platform/drivers/src/rdc.c"
    "${FreeRTOSDir}/platform/drivers/src/wdog_imx.c"
    "${FreeRTOSDir}/platform/utilities/src/debug_console_imx.c"
    "${FreeRTOSDir}/platform/utilities/inc/debug_console_imx.h"
    "${FreeRTOSDir}/platform/utilities/src/print_scan.c"
    "${FreeRTOSDir}/platform/utilities/src/print_scan.h"
    "${FreeRTOSDir}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.c"
    "${FreeRTOSDir}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.h"
    "${FreeRTOSDir}/examples/imx7_colibri_m4/clock_freq.c"
    "${FreeRTOSDir}/examples/imx7_colibri_m4/clock_freq.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4/platform.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4/plat_porting.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/rpmsg/rpmsg.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/hil/hil.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/llist/llist.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/shm/sh_mem.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/config/config.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/env/env.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/rpmsg/rpmsg_core.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/virtio/virtio.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/virtio/virtio_ring.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/virtio/virtqueue.h"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4/platform.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/imx7d_m4/platform_info.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/hil/hil.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/llist/llist.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/common/shm/sh_mem.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/config/config.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/porting/env/freertos_env.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/rpmsg/remote_device.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/rpmsg/rpmsg.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/rpmsg/rpmsg_core.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/virtio/virtio.c"
    "${FreeRTOSDir}/middleware/multicore/open-amp/virtio/virtqueue.c"
    "${ProjDirPath}/../hardware_init.c"
)
SET_TARGET_PROPERTIES(taq PROPERTIES OUTPUT_NAME "taq.elf")

TARGET_LINK_LIBRARIES(taq -Wl,--start-group)
# LIBRARIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(taq m)
TARGET_LINK_LIBRARIES(taq c)
TARGET_LINK_LIBRARIES(taq gcc)
TARGET_LINK_LIBRARIES(taq nosys)
TARGET_LINK_LIBRARIES(taq -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  -Xlinker -Map=debug/taq.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  -Xlinker -Map=release/taq.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET taq POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/taq.elf ${EXECUTABLE_OUTPUT_PATH}/taq.hex)
ADD_CUSTOM_COMMAND(TARGET taq POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/taq.elf ${EXECUTABLE_OUTPUT_PATH}/taq.bin)
