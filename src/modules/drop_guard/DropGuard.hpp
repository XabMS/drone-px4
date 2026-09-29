/****************************************************************************
 *
 *   drop_guard — filtro de suelta de carga por zona (proyecto dron de reparto).
 *   Distribuido bajo la licencia BSD de 3 cláusulas de PX4.
 *
 ****************************************************************************/

/**
 * @file DropGuard.hpp
 *
 * Módulo drop_guard (IDAL C, ADR-007). Sustituye a payload_deliverer:
 *  - atiende VEHICLE_CMD_DO_GRIPPER;
 *  - publica gripper COMMAND_GRAB siempre que se pide;
 *  - publica gripper COMMAND_RELEASE solo si DropGuardLogic lo autoriza;
 *  - confirma cada orden con vehicle_command_ack (DENIED + motivo si rechaza);
 *  - publica drop_guard_status a 5 Hz.
 *
 * Toda la decisión vive en DropGuardLogic (C++ puro, con tests unitarios).
 * Esta clase solo conecta uORB y parámetros con esa lógica.
 *
 * Trazabilidad: IR-DG-001, 002, 008, 009, 010, 011 (diseño de drop_guard §6).
 */

#pragma once

#include "DropGuardLogic.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/drop_guard_status.h>
#include <uORB/topics/gripper.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

extern "C" __EXPORT int drop_guard_main(int argc, char *argv[]);

class DropGuard : public ModuleBase<DropGuard>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DropGuard();
	~DropGuard() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	static constexpr hrt_abstime kStatusPeriod = 200_ms; ///< IR-DG-009: 5 Hz

	void Run() override;

	/** Lee DG_* y los pasa a la lógica como parámetros vivos. */
	void update_params();

	/** Construye las entradas de la lógica con los últimos datos de uORB. */
	drop_guard::Inputs build_inputs(hrt_abstime now) const;

	void handle_vehicle_command(const vehicle_command_s &cmd, hrt_abstime now);

	void publish_gripper(int8_t command, hrt_abstime now);
	void publish_ack(const vehicle_command_s &cmd, uint8_t result, int32_t result_param2, hrt_abstime now);
	void publish_status(hrt_abstime now);

	drop_guard::DropGuardLogic _logic{};

	vehicle_global_position_s _global_pos{};
	home_position_s _home{};
	bool _armed{false};

	// Contadores e histórico para el estado y la consola
	uint32_t _release_count{0};
	uint32_t _deny_count{0};
	uint8_t _last_result{drop_guard_status_s::RESULT_NONE};
	uint8_t _last_deny_reason{0};

	// Suscripciones
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Publicaciones
	uORB::Publication<gripper_s> _gripper_pub{ORB_ID(gripper)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<drop_guard_status_s> _status_pub{ORB_ID(drop_guard_status)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::DG_ENABLE>) _param_enable,
		(ParamInt<px4::params::DG_LAT_E7>) _param_lat_e7,
		(ParamInt<px4::params::DG_LON_E7>) _param_lon_e7,
		(ParamFloat<px4::params::DG_RADIUS>) _param_radius,
		(ParamFloat<px4::params::DG_ALT_MIN>) _param_alt_min,
		(ParamFloat<px4::params::DG_ALT_MAX>) _param_alt_max,
		(ParamFloat<px4::params::DG_EPH_MAX>) _param_eph_max,
		(ParamFloat<px4::params::DG_EPV_MAX>) _param_epv_max,
		(ParamInt<px4::params::DG_POS_TOUT>) _param_pos_timeout_ms,
		(ParamInt<px4::params::DG_ZONE_HASH>) _param_zone_hash
	)
};
