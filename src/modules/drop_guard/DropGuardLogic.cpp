/**
 * @file DropGuardLogic.cpp
 *
 * Implementación de la lógica de autorización de drop_guard.
 * Ver DropGuardLogic.hpp y el diseño de drop_guard (§3 y §4).
 */

#include "DropGuardLogic.hpp"

#include <cmath>
#include <cstring>

namespace drop_guard
{

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

bool in_range(float value, float min_value, float max_value)
{
	return std::isfinite(value) && (value >= min_value) && (value <= max_value);
}

bool same_bits(float a, float b)
{
	uint32_t ua = 0U;
	uint32_t ub = 0U;
	static_assert(sizeof(ua) == sizeof(a), "float de 32 bits");
	(void)std::memcpy(&ua, &a, sizeof(ua));
	(void)std::memcpy(&ub, &b, sizeof(ub));
	return ua == ub;
}

bool same_bits(double a, double b)
{
	uint64_t ua = 0U;
	uint64_t ub = 0U;
	static_assert(sizeof(ua) == sizeof(a), "double de 64 bits");
	(void)std::memcpy(&ua, &a, sizeof(ua));
	(void)std::memcpy(&ub, &b, sizeof(ub));
	return ua == ub;
}

} // namespace

double horizontal_distance_m(double lat0_deg, double lon0_deg, double lat_deg, double lon_deg)
{
	if (!std::isfinite(lat0_deg) || !std::isfinite(lon0_deg) ||
	    !std::isfinite(lat_deg) || !std::isfinite(lon_deg)) {
		return NAN;
	}

	double dlon_deg = lon_deg - lon0_deg;

	// Normaliza a [-180, 180] para que dos puntos a ambos lados del antimeridiano estén cerca.
	if (dlon_deg > 180.0) {
		dlon_deg -= 360.0;

	} else if (dlon_deg < -180.0) {
		dlon_deg += 360.0;
	}

	const double x = dlon_deg * kDegToRad * std::cos(lat0_deg * kDegToRad) * kEarthRadiusM;
	const double y = (lat_deg - lat0_deg) * kDegToRad * kEarthRadiusM;
	return std::sqrt((x * x) + (y * y));
}

bool validate_params(const Params &p)
{
	const bool lat_ok = std::isfinite(p.lat_deg) && (p.lat_deg >= -90.0) && (p.lat_deg <= 90.0);
	const bool lon_ok = std::isfinite(p.lon_deg) && (p.lon_deg >= -180.0) && (p.lon_deg <= 180.0);
	const bool radius_ok = in_range(p.radius_m, limits::kRadiusMin, limits::kRadiusMax);
	const bool alt_ok = in_range(p.alt_min_m, limits::kAltMin, limits::kAltMax) &&
			    in_range(p.alt_max_m, limits::kAltMin, limits::kAltMax);
	const bool ep_ok = in_range(p.eph_max_m, limits::kEpMin, limits::kEpMax) &&
			   in_range(p.epv_max_m, limits::kEpMin, limits::kEpMax);
	const bool timeout_ok = (p.pos_timeout_ms >= limits::kTimeoutMinMs) &&
				(p.pos_timeout_ms <= limits::kTimeoutMaxMs);

	if (!(lat_ok && lon_ok && radius_ok && alt_ok && ep_ok && timeout_ok)) {
		return false;
	}

	// La banda de altura, recortada por la peor incertidumbre vertical admitida, no puede quedar vacía:
	// si lo estuviera, la suelta nunca se autorizaría y el fallo solo se descubriría en vuelo.
	const float band_width = p.alt_max_m - p.alt_min_m;

	if (!(band_width > (2.f * p.epv_max_m))) {
		return false;
	}

	// Dos módulos atendiendo la misma orden de gripper anularían la protección.
	return !p.payload_deliverer_enabled;
}

bool params_equal(const Params &a, const Params &b)
{
	// Comparación bit a bit: se busca detectar cualquier cambio del valor almacenado,
	// no una igualdad numérica con tolerancia (un NaN es igual a sí mismo).
	return (a.enable == b.enable) &&
	       same_bits(a.lat_deg, b.lat_deg) &&
	       same_bits(a.lon_deg, b.lon_deg) &&
	       same_bits(a.radius_m, b.radius_m) &&
	       same_bits(a.alt_min_m, b.alt_min_m) &&
	       same_bits(a.alt_max_m, b.alt_max_m) &&
	       same_bits(a.eph_max_m, b.eph_max_m) &&
	       same_bits(a.epv_max_m, b.epv_max_m) &&
	       (a.pos_timeout_ms == b.pos_timeout_ms) &&
	       (a.zone_hash == b.zone_hash) &&
	       (a.payload_deliverer_enabled == b.payload_deliverer_enabled);
}

void DropGuardLogic::update_arming(bool armed)
{
	if (armed && (_state == State::DISARMED)) {
		_snapshot = _live;
		_param_change_ignored = false;
		_state = validate_params(_snapshot) ? State::ARMED_LOCKED : State::ARMED_FAULT;

	} else if (!armed && (_state != State::DISARMED)) {
		_state = State::DISARMED;
		_param_change_ignored = false;

	} else {
		// Sin transición: nada que hacer.
	}
}

void DropGuardLogic::update_params(const Params &live)
{
	_live = live;

	if ((_state != State::DISARMED) && !params_equal(_live, _snapshot)) {
		_param_change_ignored = true;
	}
}

const Params &DropGuardLogic::active_params() const
{
	return (_state == State::DISARMED) ? _live : _snapshot;
}

Evaluation DropGuardLogic::evaluate(const Inputs &in) const
{
	Evaluation result{};
	const Params &p = active_params();

	if (!p.enable) {
		result.reason = Reason::DISABLED;

	} else if (_state == State::DISARMED) {
		result.reason = Reason::NOT_ARMED;

	} else if (_state == State::ARMED_FAULT) {
		result.reason = Reason::PARAMS_INVALID;

	} else if ((in.pos_timestamp_us > in.now_us) ||
		   ((in.now_us - in.pos_timestamp_us) > (static_cast<uint64_t>(p.pos_timeout_ms) * 1000U))) {
		// Una marca de tiempo futura indica un problema de reloj: se trata como no fresca.
		result.reason = Reason::POS_STALE;

	} else if (!in.pos_valid ||
		   !std::isfinite(in.lat_deg) || !std::isfinite(in.lon_deg) || !std::isfinite(in.alt_amsl_m) ||
		   !std::isfinite(in.eph_m) || !std::isfinite(in.epv_m) ||
		   (in.eph_m < 0.f) || (in.epv_m < 0.f)) {
		result.reason = Reason::POS_INVALID;

	} else if (in.eph_m > p.eph_max_m) {
		result.reason = Reason::EPH_HIGH;

	} else if (in.epv_m > p.epv_max_m) {
		result.reason = Reason::EPV_HIGH;

	} else if (!in.home_alt_valid || !std::isfinite(in.home_alt_amsl_m)) {
		result.reason = Reason::HOME_INVALID;

	} else {
		result.distance_m = horizontal_distance_m(p.lat_deg, p.lon_deg, in.lat_deg, in.lon_deg);
		result.rel_alt_m = in.alt_amsl_m - in.home_alt_amsl_m;

		if (!((result.distance_m + static_cast<double>(in.eph_m)) <= static_cast<double>(p.radius_m))) {
			result.reason = Reason::OUTSIDE_RADIUS;

		} else if (result.rel_alt_m < (p.alt_min_m + in.epv_m)) {
			result.reason = Reason::BELOW_BAND;

		} else if (result.rel_alt_m > (p.alt_max_m - in.epv_m)) {
			result.reason = Reason::ABOVE_BAND;

		} else {
			result.reason = Reason::OK;
			result.authorized = true;
		}
	}

	return result;
}

} // namespace drop_guard
