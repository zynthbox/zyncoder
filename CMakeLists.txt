cmake_minimum_required(VERSION 2.6)
project(zyncoder)

include(CheckIncludeFiles)
include(CheckLibraryExists)

find_package(Python 3.5 REQUIRED COMPONENTS Interpreter)

link_directories(/usr/local/lib)

check_library_exists(wiringPi wiringPiSetup "" HAVE_WIRINGPI_LIB)

if (HAVE_WIRINGPI_LIB)
	message("++ Defined HAVE_WIRINGPI_LIB")
	add_definitions(-DHAVE_WIRINGPI_LIB)
endif()

if(("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "Z2_V2"))
	message("++ Defined MCP23017_ENCODERS Z2_V2")
	add_definitions(-DMCP23017_ENCODERS)
	add_definitions(-DZ2_VERSION=2)
elseif(("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "Z2_V3"))
	message("++ Defined MCP23017_ENCODERS")
	add_definitions(-DMCP23017_ENCODERS)
	add_definitions(-DZ2_VERSION=3)
elseif (("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "MCP23017_ENCODERS")
 OR ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "MCP23017_EXTRA")
 OR ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "MCP23017_ZynScreen")
 OR ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "MCP23017_EPDF")
 OR ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "MCP23017_EPDF_REVERSE"))
	message("++ Defined MCP23017_ENCODERS")
	add_definitions(-DMCP23017_ENCODERS)
elseif(NOT ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "DUMMIES"))
	message("++ Defined MCP23008_ENCODERS")
	add_definitions(-DMCP23008_ENCODERS)
else()
	message("++ Defined DUMMY_ENCODERS")
	add_definitions(-DDUMMY_ENCODERS)
endif()

if (DEFINED ENV{ZYNTHIAN_WIRING_MCP23017_INTA_PIN} AND NOT ("$ENV{ZYNTHIAN_WIRING_MCP23017_INTA_PIN}" STREQUAL ""))
	message("++ Defined MCP23017 INTA PIN")
	add_definitions(-DMCP23017_INTA_PIN=$ENV{ZYNTHIAN_WIRING_MCP23017_INTA_PIN})
endif()

if (DEFINED ENV{ZYNTHIAN_WIRING_MCP23017_INTB_PIN} AND NOT ("$ENV{ZYNTHIAN_WIRING_MCP23017_INTB_PIN}" STREQUAL ""))
	message("++ Defined MCP23017 INTB PIN")
	add_definitions(-DMCP23017_INTB_PIN=$ENV{ZYNTHIAN_WIRING_MCP23017_INTB_PIN})
endif()

if (DEFINED ENV{ZYNTHIAN_FORCE_WIRINGPI_EMU})
	message("++ Forced wiringPiEmu")
	set(ZYNTHIAN_FORCE_WIRINGPI_EMU "$ENV{ZYNTHIAN_FORCE_WIRINGPI_EMU}")
endif()

if (DEFINED ENV{ZYNTHIAN_WIRING_ZYNAPTIK_CONFIG} AND NOT ("$ENV{ZYNTHIAN_WIRING_ZYNAPTIK_CONFIG}" STREQUAL ""))
	message("++ Defined ZYNAPTIK_CONFIG")
	add_definitions(-DZYNAPTIK_CONFIG="$ENV{ZYNTHIAN_WIRING_ZYNAPTIK_CONFIG}")
	set(BUILD_ZYNAPTIK "1")
endif()

if (DEFINED ENV{ZYNTHIAN_WIRING_ZYNTOF_CONFIG} AND NOT ("$ENV{ZYNTHIAN_WIRING_ZYNTOF_CONFIG}" STREQUAL ""))
	message("++ Defined ZYNTOF_CONFIG")
	add_definitions(-DZYNTOF_CONFIG="$ENV{ZYNTHIAN_WIRING_ZYNTOF_CONFIG}")
	set(BUILD_ZYNTOF "1")
endif()

set_source_files_properties( zynrv112.c PROPERTIES LANGUAGE CXX LINKER_LANGUAGE CXX)

if ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "I2C_HWC")
	message("++ Using I2C HWC")
	if (BUILD_ZYNTOF AND BUILD_ZYNAPTIK)
		message("++ Building Zynaptik & Zyntof support")
		add_library(zyncoder SHARED zyncoder_i2c.h zyncoder_i2c.c zynmcp23017.c zyncontrol.h zyncontrol_z2.c zynmidirouter.h zynmidirouter.c zynaptik.h zynaptik.c zyntof.h zyntof.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo tof)
	elseif (BUILD_ZYNAPTIK)
		message("++ Building Zynaptik support")
		add_library(zyncoder SHARED zyncoder_i2c.h zyncoder_i2c.c zynmcp23017.c zyncontrol.h zyncontrol_z2.c zynmidirouter.h zynmidirouter.c zynaptik.h zynaptik.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo)
	elseif (BUILD_ZYNTOF)
		message("++ Building Zyntof support")
		add_library(zyncoder SHARED zyncoder_i2c.h zyncoder_i2c.c zynmcp23017.c zyncontrol.h zyncontrol_z2.c zynmidirouter.h zynmidirouter.c zyntof.h zyntof.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo tof)
	else ()
		add_library(zyncoder SHARED zyncoder_i2c.h zyncoder_i2c.c zynmcp23017.c zyncontrol.h zyncontrol_z2.c zynmidirouter.h zynmidirouter.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo)
	endif()

elseif (NOT ZYNTHIAN_FORCE_WIRINGPI_EMU AND HAVE_WIRINGPI_LIB)
	message("++ Using wiringPI")
	if (BUILD_ZYNTOF AND BUILD_ZYNAPTIK)
		message("++ Building Zynaptik & Zyntof support")
		add_library(zyncoder SHARED zyncoder.h zyncoder.c zynmidirouter.h zynmcp23017.c zynmcp23008.h zynmcp23008.c zynmidirouter.c zynaptik.h zynaptik.c zyntof.h zyntof.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo tof)
	elseif (BUILD_ZYNAPTIK)
		message("++ Building Zynaptik support")
		add_library(zyncoder SHARED zyncoder.h zyncoder.c zynmidirouter.h zynmcp23017.c zynmcp23008.h zynmcp23008.c zynmidirouter.c zynaptik.h zynaptik.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo)
	elseif (BUILD_ZYNTOF)
		message("++ Building Zyntof support")
		add_library(zyncoder SHARED zyncoder.h zyncoder.c zynmidirouter.h zynmcp23017.c zynmcp23008.h zynmcp23008.c zynmidirouter.c zyntof.h zyntof.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo tof)
	elseif(("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "Z2_V2") OR ("$ENV{ZYNTHIAN_WIRING_LAYOUT}" STREQUAL "Z2_V3"))
		message("++ Building With Z2")
		add_library(zyncoder SHARED zyncoder.h zyncoder.c zyncontrol.h zynmcp23017.c zynmcp23008.h zynmcp23008.c zyncontrol_z2.c lm4811.h lm4811.c zynpot.h zynpot.c zynads1115.h zynads1115.c zynrv112.h zynrv112.c zynmidirouter.h zynmidirouter.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo)
	else()
		message("++ Building Base Version")
		add_library(zyncoder SHARED zyncoder.h zyncoder.c zyncontrol.h zynmidirouter.h zynmidirouter.c)
		target_compile_options(zyncoder PRIVATE -fcommon)
		target_link_libraries(zyncoder wiringPi jack lo)
	endif()
else()
	message("++ Using wiringPiEmu")
	add_library(zyncoder SHARED zyncoder.h zyncoder.c wiringPiEmu.c zynmidirouter.h zynmidirouter.c)
	#add_library(wiringPiEmu SHARED wiringPiEmu.h wiringPiEmu.c)
	#add_library(zynmidirouter SHARED zynmidirouter.h zynmidirouter.c)
	target_compile_options(zyncoder PRIVATE -fcommon)
	target_link_libraries(zyncoder jack lo)
	#install(TARGETS wiringPiEmu LIBRARY DESTINATION lib)
endif()

add_executable(zyncoder_test zyncoder_test.c)
target_link_libraries(zyncoder_test zyncoder)
install(TARGETS zyncoder_test DESTINATION bin)

install(TARGETS zyncoder LIBRARY DESTINATION lib)
#install(TARGETS zynmidirouter LIBRARY DESTINATION lib)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')"
  OUTPUT_VARIABLE Python_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)

# Install zyncoder module to Python dist-packages
install(FILES __init__.py zyncoder.py DESTINATION lib/python${Python_VERSION}/dist-packages/zyncoder)
