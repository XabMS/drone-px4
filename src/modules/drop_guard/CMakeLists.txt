############################################################################
#
#   drop_guard — filtro de suelta de carga por zona (proyecto dron de reparto).
#   Distribuido bajo la licencia BSD de 3 cláusulas de PX4.
#
############################################################################

# Lógica de decisión (C++ puro, IDAL C), separada para poder probarla sola.
px4_add_library(drop_guard_logic
	DropGuardLogic.cpp
	DropGuardLogic.hpp
)

px4_add_module(
	MODULE modules__drop_guard
	MAIN drop_guard
	SRCS
		DropGuard.cpp
		DropGuard.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		drop_guard_logic
	)

px4_add_unit_gtest(SRC DropGuardLogicTest.cpp LINKLIBS drop_guard_logic)
