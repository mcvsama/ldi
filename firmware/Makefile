# vim:ts=4

first: all

# Override these variables in Makefile.local.

# Show verbose compilation commands?
VERBOSE			:= 0
# How many threads use for compilation?
THREADS			:= $(shell grep processor /proc/cpuinfo|wc -l)
# Architecture:
ARCH			:= avr
# Enable profiling?
PROFILING		:= 0
# Debugging enabled?
DEBUG			:= 0
# Where Qt is installed?
QT_PREFIX		:= /usr
# Profile name:
PROFILE			:=
# Version:
VERSION			:= alpha0.1dev
# Release version? (no asserts, etc).
RELEASE			:= 0
# Set to 1 to disable some UB-generating optimizations:
UB_OPTS_DISABLE	:= 0
# AVR
MCU				:= atxmega128a1u
MCU_FREQUENCY	:= 32000000L
TOOLCHAIN		:= /usr

-include Makefile.local

#### Core (special) vars ####
# TODO list all special vars used by Makefile.core

DEFINES			+= -DMCU_TYPE=$(MCU) -DF_CPU=$(MCU_FREQUENCY)
#C_CXX_OPT_FLAGS	+= -finline -funroll-loops -fomit-frame-pointer -DQT_NO_DEBUG
LIBS			+=
PKGCONFIGS		+=

AS				:= $(TOOLCHAIN)/bin/avr-gcc -c
ASFLAGS			+= $(DEFINES) -O3 -mmcu=$(MCU) -Ilib -Wall
C				:= $(TOOLCHAIN)/bin/avr-gcc -c
CFLAGS			+= $(DEFINES) -O3 -mmcu=$(MCU) -Ilib -Ilib/mulabs-avr/src -Wall -I../software/src/ $(C_CXX_OPT_FLAGS)
CXX				:= $(TOOLCHAIN)/bin/avr-g++ -c
CXXFLAGS		+= $(DEFINES) -O3 -mmcu=$(MCU) -Ilib -Ilib/mulabs-avr/src -Wall -I../software/src/ -std=c++14 -fno-rtti $(C_CXX_OPT_FLAGS)
SO				:= $(CXX) -shared
SOFLAGS			+=
LD				:= $(TOOLCHAIN)/bin/avr-g++
LDFLAGS			+= -L$(TOOLCHAIN)/avr/lib -mmcu=$(MCU)
AR				:= $(TOOLCHAIN)/bin/avr-ar
ARFLAGS			+=
DEPGEN			:= $(CXX)
DEPFLAGS		+= $(DEFINES)
MOC				:= $(QT_PREFIX)/bin/moc-qt5

ELF_TO_HEX		:= $(TOOLCHAIN)/bin/avr-objcopy -O ihex

#### Generic additional flags ####

ifeq ($(CXX),g++)
CXXFLAGS		+= -fPIE -O4 -g -c -std=c++14 -Wall -Wall -Wextra -Wunused  -Wunused-function -Wunused-label -Wnoexcept -fstrict-aliasing -Wstrict-aliasing=3 -fnothrow-opt
CXXFLAGS		+= -I. -Ilib
endif

ifeq ($(CXX),clang++)
CLANG_DISABLED_WARNINGS = mismatched-tags disabled-macro-expansion c++98-compat-pedantic documentation-unknown-command
CXXFLAGS		+= -fPIE -O3 -g -c -std=c++14 -Wall -Wall -Wextra -Wunused  -Wunused-function -Wunused-label -fstrict-aliasing $(foreach warning, $(CLANG_DISABLED_WARNINGS), -Wno-$(warning))
CXXFLAGS		+= -I. -Ilib
endif

# Undefined-Behaviour handling:
ifeq ($(UB_OPTS_DISABLE),1)
CFLAGS			+= -fno-strict-overflow -fno-delete-null-pointer-checks -fno-strict-aliasing
CXXFLAGS		+= -fno-strict-overflow -fno-delete-null-pointer-checks -fno-strict-aliasing
endif

ifeq ($(DEBUG),1)
CFLAGS			+= -O2 -ggdb3 -fno-inline -fno-eliminate-unused-debug-types -D_GLIBCXX_DEBUG
CXXFLAGS		+= -O2 -ggdb3 -fno-inline -fno-eliminate-unused-debug-types -D_GLIBCXX_DEBUG
endif

ifneq ($(LIBS),)
LDFLAGS			+= $(foreach lib, $(LIBS), -l$(lib))
endif

ifneq ($(PKGCONFIGS),)
LDFLAGS			+= $(shell pkg-config --libs $(PKGCONFIGS))
CFLAGS			+= $(shell pkg-config --cflags $(PKGCONFIGS))
CXXFLAGS		+= $(shell pkg-config --cflags $(PKGCONFIGS))
endif

ifneq ($(VERBOSE),1)
MAKEFLAGS += --silent
endif

ifneq ($(THREADS),)
MAKEFLAGS += -j$(THREADS)
endif

ifeq ($(PROFILING),1)
CXXFLAGS += -pg -fno-omit-frame-pointer
LDFLAGS += -pg
endif

#### Sources and rules ####

HEADERS =
SOURCES =
OBJECTS =
NODEP_OBJECTS =
MOCSRCS =
TARGETS =

include Makefile.sources
include Makefile.core
include Makefile.rules

