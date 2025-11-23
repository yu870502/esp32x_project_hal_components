idf_build_get_property(target IDF_TARGET)   #该命令将芯片型号存储在变量target中，后续可通过if语句进行分支处理

if(${target} STREQUAL "linux")
    return() # This component is not supported by the POSIX/Linux simulator
endif()

set(srcs
    "humidity_sensor.c")

#set(include_dirs "include")
set(includes 
    .)

idf_component_register(SRCS "${srcs}"
                        INCLUDE_DIRS "${includes}"
                        PRIV_REQUIRES esp_adc esp_driver_gpio unity)

