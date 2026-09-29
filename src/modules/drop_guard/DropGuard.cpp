/****************************************************************************
 *
 *   drop_guard — filtro de suelta de carga por zona (proyecto dron de reparto).
 *   Distribuido bajo la licencia BSD de 3 cláusulas de PX4.
 *
 ****************************************************************************/

/**
 * @file DropGuard.cpp
 * Ver DropGuard.hpp y el diseño de drop_guard.
 */

#include "DropGuard.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h> // define CONFIG_MODULES_* de la placa

#include <cmath>

// drop_guard sustituye a payload_deliverer: los dos no pueden atender a la vez DO_GRIPPER,
// porque payload_deliverer abriría el gripper sin comprobar la zona (IR-DG-007).
#if defined(CONFIG_MODULES_PAYLOAD_DELIVERER) && (CONFIG_MODULES_PAYLOAD_DELIVERER == 1)
#error "drop_guard y payload_deliverer no pueden compilarse juntos: desactiva CONFIG_MODULES_PAYLOAD_DELIVERER"
#endif

namespace
{

constexpr double kE7 = 1e-7;

} // namespace

DropGuard::DropGuard() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool DropGuard::init()
{
	update_params();

	// IR-DG-011: FunctionGripper arranca con el valor de "abierto" hasta recibir un mensaje,
	// así que lo primero es ordenar el cierre.
	publish_gripper(gripper_s::COMMAND_GRAB, hrt_absolute_time());

	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("registro del callback de vehicle_command fallido");
		return false;
	}

	ScheduleOnInterval(kStatusPeriod);
	return true;
}

void DropGuard::update_params()
{
	updateParams();

	drop_guard::Params p{};
	p.enable = _param_enable.get();
	p.lat_deg = static_cast<double>(_param_lat_e7.get()) * kE7;
	p.lon_deg = static_cast<double>(_param_lon_e7.get()) * kE7;
	p.radius_m = _param_radius.get();
	p.alt_min_m = _param_alt_min.get();
	p.alt_max_m = _param_alt_max.get();
	p.eph_max_m = _param_eph_max.get();
	p.epv_max_m = _param_epv_max.get();
	p.pos_timeout_ms = _param_pos_timeout_ms.get();
	p.zone_hash = _param_zone_hash.get();
	p.payload_deliverer_enabled = false; // garantizado en compilación (#error arriba)

	const bool was_ignored = _logic.param_change_ignored();
	_logic.update_params(p);

	if (_logic.param_change_ignored() && !was_ignored) {
		PX4_WARN("DG_* cambiados con el dron armado: se ignoran hasta desarmar");
	}
}

drop_guard::Inputs DropGuard::build_inputs(hrt_abstime now) const
{
	drop_guard::Inputs in{};
	in.now_us = now;
	in.pos_valid = _global_pos.lat_lon_valid && _global_pos.alt_valid;
	in.pos_timestamp_us = _global_pos.timestamp;
	in.lat_deg = _global_pos.lat;
	in.lon_deg = _global_pos.lon;
	in.alt_amsl_m = _global_pos.alt;
	in.eph_m = _global_pos.eph;
	in.epv_m = _global_pos.epv;
	in.home_alt_valid = _home.valid_alt;
	in.home_alt_amsl_m = _home.alt;
	return in;
}

void DropGuard::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		update_params();
	}

	vehicle_status_s status{};

	if (_vehicle_status_sub.update(&status)) {
		const bool armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

		if (armed && !_armed) {
			_logic.update_arming(true);

			if (_logic.state() == drop_guard::State::ARMED_FAULT) {
				PX4_ERR("parámetros DG_* no válidos al armar: la suelta quedará denegada en este vuelo");

			} else {
				PX4_INFO("zona de suelta capturada (hash %" PRId32 ")", _logic.active_params().zone_hash);
			}

			// IR-DG-011: cada vuelo empieza con el gripper cerrado.
			publish_gripper(gripper_s::COMMAND_GRAB, now);

		} else if (!armed && _armed) {
			_logic.update_arming(false);
		}

		_armed = armed;
	}

	// Mantener siempre la última posición y altura de home.
	_global_pos_sub.update(&_global_pos);
	_home_sub.update(&_home);

	// Procesar todas las órdenes en cola, no solo la última.
	vehicle_command_s cmd{};

	while (_vehicle_command_sub.update(&cmd)) {
		handle_vehicle_command(cmd, now);
	}

	publish_status(now);
}

void DropGuard::handle_vehicle_command(const vehicle_command_s &cmd, hrt_abstime now)
{
	if (cmd.command != vehicle_command_s::VEHICLE_CMD_DO_GRIPPER) {
		return;
	}

	const int32_t action = static_cast<int32_t>(lroundf(cmd.param2));

	if (action == vehicle_command_s::GRIPPER_ACTION_GRAB) {
		// IR-DG-002: cerrar siempre está permitido.
		publish_gripper(gripper_s::COMMAND_GRAB, now);
		publish_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 0, now);

	} else if (action == vehicle_command_s::GRIPPER_ACTION_RELEASE) {
		const drop_guard::Evaluation e = _logic.evaluate(build_inputs(now));

		if (e.authorized) {
			// IR-DG-001: única ruta que publica COMMAND_RELEASE.
			publish_gripper(gripper_s::COMMAND_RELEASE, now);
			publish_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 0, now);
			_release_count++;
			_last_result = drop_guard_status_s::RESULT_RELEASED;
			PX4_INFO("suelta autorizada: d=%.1f m, h=%.1f m", e.distance_m, static_cast<double>(e.rel_alt_m));

		} else {
			// IR-DG-008: rechazo con motivo en result_param2.
			publish_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, static_cast<int32_t>(e.reason), now);
			_deny_count++;
			_last_result = drop_guard_status_s::RESULT_DENIED;
			_last_deny_reason = static_cast<uint8_t>(e.reason);
			PX4_WARN("suelta denegada, motivo %u", static_cast<unsigned>(e.reason));
		}

	} else {
		publish_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 0, now);
	}
}

void DropGuard::publish_gripper(int8_t command, hrt_abstime now)
{
	gripper_s gripper{};
	gripper.timestamp = now;
	gripper.command = command;
	_gripper_pub.publish(gripper);
}

void DropGuard::publish_ack(const vehicle_command_s &cmd, uint8_t result, int32_t result_param2, hrt_abstime now)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = now;
	ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
	ack.result = result;
	ack.result_param2 = result_param2;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.from_external = false;
	_ack_pub.publish(ack);
}

void DropGuard::publish_status(hrt_abstime now)
{
	const drop_guard::Evaluation e = _logic.evaluate(build_inputs(now));

	drop_guard_status_s s{};
	s.timestamp = now;
	s.state = static_cast<uint8_t>(_logic.state());
	s.authorized = e.authorized;
	s.reason = static_cast<uint8_t>(e.reason);
	s.distance_m = static_cast<float>(e.distance_m);
	s.rel_alt_m = e.rel_alt_m;
	s.param_change_ignored = _logic.param_change_ignored();
	s.zone_hash = _logic.active_params().zone_hash;
	s.last_result = _last_result;
	s.last_deny_reason = _last_deny_reason;
	s.release_count = _release_count;
	s.deny_count = _deny_count;
	_status_pub.publish(s);
}

int DropGuard::print_status()
{
	static const char *const kStateNames[] = {"DESARMADO", "ARMADO_BLOQUEADO", "ARMADO_FALLO"};
	const drop_guard::Params &p = _logic.active_params();
	const auto state_index = static_cast<unsigned>(_logic.state());

	PX4_INFO("estado: %s", (state_index < 3U) ? kStateNames[state_index] : "?");
	PX4_INFO("zona: lat %.7f lon %.7f r %.1f m, h [%.1f, %.1f] m, hash %" PRId32,
		 p.lat_deg, p.lon_deg, static_cast<double>(p.radius_m),
		 static_cast<double>(p.alt_min_m), static_cast<double>(p.alt_max_m), p.zone_hash);
	PX4_INFO("cambio de DG_* ignorado: %s", _logic.param_change_ignored() ? "sí" : "no");
	PX4_INFO("sueltas autorizadas: %" PRIu32 ", denegadas: %" PRIu32 " (último motivo %u)",
		 _release_count, _deny_count, static_cast<unsigned>(_last_deny_reason));
	return 0;
}

int DropGuard::custom_command(int argc, char *argv[])
{
	// Solo se permite cerrar desde la consola: abrir por consola saltaría el filtro.
	if ((argc >= 1) && (strcmp(argv[0], "close") == 0) && is_running()) {
		get_instance()->publish_gripper(gripper_s::COMMAND_GRAB, hrt_absolute_time());
		return 0;
	}

	return print_usage("comando no reconocido");
}

int DropGuard::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Descripción
Filtro de suelta de carga por zona. Sustituye a payload_deliverer: atiende VEHICLE_CMD_DO_GRIPPER
y solo publica la apertura del gripper si la aeronave está armada dentro de la zona de suelta
definida por los parámetros DG_*, con posición válida, reciente y precisa.
Los parámetros se capturan al armar y sus cambios se ignoran hasta desarmar.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("drop_guard", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("close", "Cierra el gripper (abrir no está permitido desde la consola)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int DropGuard::task_spawn(int argc, char *argv[])
{
	DropGuard *instance = new DropGuard();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("sin memoria");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int drop_guard_main(int argc, char *argv[])
{
	return DropGuard::main(argc, argv);
}
