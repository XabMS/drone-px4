/**
 * @file DropGuardLogicTest.cpp
 *
 * Tests unitarios de DropGuardLogic (diseño de drop_guard §6).
 * Cada test indica el requisito de ítem (IR-DG-xxx) que verifica.
 */

#include "DropGuardLogic.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace drop_guard;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Centro de la zona de ejemplo: cerca de Toulouse (coordenadas ilustrativas).
constexpr double kLat0 = 43.6500;
constexpr double kLon0 = 1.3700;
constexpr float kHomeAlt = 150.f;

constexpr uint64_t kNow = 10'000'000U; // 10 s

/** Punto desplazado north_m / east_m respecto a (lat0, lon0) con la misma aproximación. */
void offset(double lat0, double lon0, double north_m, double east_m, double &lat, double &lon)
{
	lat = lat0 + (north_m / kEarthRadiusM) / kDegToRad;
	lon = lon0 + (east_m / (kEarthRadiusM * std::cos(lat0 * kDegToRad))) / kDegToRad;
}

double haversine_m(double lat0, double lon0, double lat, double lon)
{
	const double p0 = lat0 * kDegToRad;
	const double p1 = lat * kDegToRad;
	const double dp = (lat - lat0) * kDegToRad;
	const double dl = (lon - lon0) * kDegToRad;
	const double a = std::sin(dp / 2) * std::sin(dp / 2) +
			 std::cos(p0) * std::cos(p1) * std::sin(dl / 2) * std::sin(dl / 2);
	return 2.0 * kEarthRadiusM * std::asin(std::sqrt(a));
}

Params nominal_params()
{
	Params p{};
	p.enable = true;
	p.lat_deg = kLat0;
	p.lon_deg = kLon0;
	p.radius_m = 10.f;
	p.alt_min_m = 15.f;
	p.alt_max_m = 30.f;
	p.eph_max_m = 2.f;
	p.epv_max_m = 3.f;
	p.pos_timeout_ms = 200;
	p.zone_hash = 1234;
	p.payload_deliverer_enabled = false;
	return p;
}

/** Entrada nominal: en el centro de la zona, a 20 m sobre home, posición fresca y precisa. */
Inputs nominal_inputs()
{
	Inputs in{};
	in.now_us = kNow;
	in.pos_valid = true;
	in.pos_timestamp_us = kNow - 20'000U;
	in.lat_deg = kLat0;
	in.lon_deg = kLon0;
	in.alt_amsl_m = kHomeAlt + 20.f;
	in.eph_m = 0.5f;
	in.epv_m = 0.5f;
	in.home_alt_valid = true;
	in.home_alt_amsl_m = kHomeAlt;
	return in;
}

/** Lógica armada con parámetros nominales (estado ARMED_LOCKED). */
DropGuardLogic armed_logic(const Params &p = nominal_params())
{
	DropGuardLogic logic;
	logic.update_params(p);
	logic.update_arming(true);
	return logic;
}

void expect_denied(const Evaluation &e, Reason reason)
{
	EXPECT_FALSE(e.authorized);
	EXPECT_EQ(e.reason, reason);
}

} // namespace

// ---------------------------------------------------------------------------
// Distancia horizontal
// ---------------------------------------------------------------------------

TEST(Distance, ZeroAtSamePoint)
{
	EXPECT_DOUBLE_EQ(horizontal_distance_m(kLat0, kLon0, kLat0, kLon0), 0.0);
}

TEST(Distance, NorthAndEastOffsetsMatch)
{
	double lat, lon;
	offset(kLat0, kLon0, 100.0, 0.0, lat, lon);
	EXPECT_NEAR(horizontal_distance_m(kLat0, kLon0, lat, lon), 100.0, 1e-6);

	offset(kLat0, kLon0, 0.0, 100.0, lat, lon);
	EXPECT_NEAR(horizontal_distance_m(kLat0, kLon0, lat, lon), 100.0, 1e-6);

	offset(kLat0, kLon0, -30.0, 40.0, lat, lon);
	EXPECT_NEAR(horizontal_distance_m(kLat0, kLon0, lat, lon), 50.0, 1e-6);
}

TEST(Distance, AgreesWithHaversineBelow1km)
{
	// El error de la aproximación debe ser despreciable frente a eph (centímetros como mucho).
	double lat, lon;
	offset(kLat0, kLon0, 600.0, -700.0, lat, lon);
	const double ref = haversine_m(kLat0, kLon0, lat, lon);
	EXPECT_NEAR(horizontal_distance_m(kLat0, kLon0, lat, lon), ref, 0.05);
}

TEST(Distance, NegativeLongitudeDonostia)
{
	const double lat0 = 43.3183, lon0 = -1.9812;
	double lat, lon;
	offset(lat0, lon0, 5.0, -5.0, lat, lon);
	EXPECT_NEAR(horizontal_distance_m(lat0, lon0, lat, lon), std::sqrt(50.0), 1e-6);
}

TEST(Distance, CrossingGreenwichMeridian)
{
	double lat, lon;
	offset(kLat0, 0.00002, 0.0, -3.0, lat, lon); // de lon positiva a negativa
	EXPECT_LT(lon, 0.0);
	EXPECT_NEAR(horizontal_distance_m(kLat0, 0.00002, lat, lon), 3.0, 1e-6);
}

TEST(Distance, CrossingAntimeridianIsShort)
{
	EXPECT_LT(horizontal_distance_m(0.0, 179.99999, 0.0, -179.99999), 3.0);
	EXPECT_LT(horizontal_distance_m(0.0, -179.99999, 0.0, 179.99999), 3.0);
}

TEST(Distance, NonFiniteInputGivesNaN)
{
	const double nan = std::numeric_limits<double>::quiet_NaN();
	EXPECT_TRUE(std::isnan(horizontal_distance_m(nan, kLon0, kLat0, kLon0)));
	EXPECT_TRUE(std::isnan(horizontal_distance_m(kLat0, nan, kLat0, kLon0)));
	EXPECT_TRUE(std::isnan(horizontal_distance_m(kLat0, kLon0, nan, kLon0)));
	EXPECT_TRUE(std::isnan(horizontal_distance_m(kLat0, kLon0, kLat0, INFINITY)));
}

// ---------------------------------------------------------------------------
// Validación de parámetros (IR-DG-007)
// ---------------------------------------------------------------------------

TEST(ValidateParams, NominalIsValid)
{
	EXPECT_TRUE(validate_params(nominal_params()));
}

TEST(ValidateParams, RangeLimitsAreInclusive)
{
	Params p = nominal_params();
	p.radius_m = limits::kRadiusMin;
	EXPECT_TRUE(validate_params(p));
	p.radius_m = limits::kRadiusMax;
	EXPECT_TRUE(validate_params(p));

	p = nominal_params();
	p.pos_timeout_ms = limits::kTimeoutMinMs;
	EXPECT_TRUE(validate_params(p));
	p.pos_timeout_ms = limits::kTimeoutMaxMs;
	EXPECT_TRUE(validate_params(p));

	p = nominal_params();
	p.eph_max_m = limits::kEpMin;
	p.epv_max_m = limits::kEpMin;
	EXPECT_TRUE(validate_params(p));

	p = nominal_params();
	p.lat_deg = 90.0;
	p.lon_deg = -180.0;
	EXPECT_TRUE(validate_params(p));
}

TEST(ValidateParams, RejectsOutOfRangeOrNonFinite)
{
	const float fnan = std::numeric_limits<float>::quiet_NaN();
	const double dnan = std::numeric_limits<double>::quiet_NaN();

	struct Case {
		const char *name;
		void (*mutate)(Params &, float, double);
	};

	const Case cases[] = {
		{"lat > 90", [](Params & p, float, double) { p.lat_deg = 90.001; }},
		{"lat < -90", [](Params & p, float, double) { p.lat_deg = -90.001; }},
		{"lat NaN", [](Params & p, float, double n) { p.lat_deg = n; }},
		{"lon > 180", [](Params & p, float, double) { p.lon_deg = 180.001; }},
		{"lon < -180", [](Params & p, float, double) { p.lon_deg = -180.001; }},
		{"lon NaN", [](Params & p, float, double n) { p.lon_deg = n; }},
		{"radius too small", [](Params & p, float, double) { p.radius_m = 0.99f; }},
		{"radius too large", [](Params & p, float, double) { p.radius_m = 100.01f; }},
		{"radius NaN", [](Params & p, float n, double) { p.radius_m = n; }},
		{"alt_min too low", [](Params & p, float, double) { p.alt_min_m = 4.9f; }},
		{"alt_max too high", [](Params & p, float, double) { p.alt_max_m = 120.1f; }},
		{"alt_min NaN", [](Params & p, float n, double) { p.alt_min_m = n; }},
		{"alt_min >= alt_max", [](Params & p, float, double) { p.alt_min_m = 30.f; p.alt_max_m = 30.f; }},
		{"eph_max too small", [](Params & p, float, double) { p.eph_max_m = 0.4f; }},
		{"eph_max too large", [](Params & p, float, double) { p.eph_max_m = 10.1f; }},
		{"epv_max NaN", [](Params & p, float n, double) { p.epv_max_m = n; }},
		{"timeout too small", [](Params & p, float, double) { p.pos_timeout_ms = 49; }},
		{"timeout too large", [](Params & p, float, double) { p.pos_timeout_ms = 1001; }},
		{"payload_deliverer enabled", [](Params & p, float, double) { p.payload_deliverer_enabled = true; }},
	};

	for (const Case &c : cases) {
		Params p = nominal_params();
		c.mutate(p, fnan, dnan);
		EXPECT_FALSE(validate_params(p)) << c.name;
	}
}

TEST(ValidateParams, RejectsBandEmptiedByVerticalUncertainty)
{
	// Banda de 15 a 30 m (15 m de ancho). Con epv_max = 7.5 quedaría vacía: 15 - 2*7.5 = 0.
	Params p = nominal_params();
	p.epv_max_m = 7.5f;
	EXPECT_FALSE(validate_params(p));
	p.epv_max_m = 7.4f;
	EXPECT_TRUE(validate_params(p));
}

TEST(ParamsEqual, DetectsEveryField)
{
	const Params base = nominal_params();
	EXPECT_TRUE(params_equal(base, base));

	Params p = base; p.enable = false; EXPECT_FALSE(params_equal(base, p));
	p = base; p.lat_deg += 1e-7; EXPECT_FALSE(params_equal(base, p));
	p = base; p.lon_deg += 1e-7; EXPECT_FALSE(params_equal(base, p));
	p = base; p.radius_m += 0.1f; EXPECT_FALSE(params_equal(base, p));
	p = base; p.alt_min_m += 0.1f; EXPECT_FALSE(params_equal(base, p));
	p = base; p.alt_max_m += 0.1f; EXPECT_FALSE(params_equal(base, p));
	p = base; p.eph_max_m += 0.1f; EXPECT_FALSE(params_equal(base, p));
	p = base; p.epv_max_m += 0.1f; EXPECT_FALSE(params_equal(base, p));
	p = base; p.pos_timeout_ms += 1; EXPECT_FALSE(params_equal(base, p));
	p = base; p.zone_hash += 1; EXPECT_FALSE(params_equal(base, p));
	p = base; p.payload_deliverer_enabled = true; EXPECT_FALSE(params_equal(base, p));

	// Un NaN almacenado es igual a sí mismo: no genera un aviso de cambio en cada ciclo.
	Params n = base;
	n.radius_m = std::numeric_limits<float>::quiet_NaN();
	n.lat_deg = std::numeric_limits<double>::quiet_NaN();
	EXPECT_TRUE(params_equal(n, n));
	EXPECT_FALSE(params_equal(base, n));
}

// ---------------------------------------------------------------------------
// Estados y bloqueo de parámetros (IR-DG-006, IR-DG-007)
// ---------------------------------------------------------------------------

TEST(StateMachine, StartsDisarmedAndDenies)
{
	DropGuardLogic logic;
	logic.update_params(nominal_params());
	EXPECT_EQ(logic.state(), State::DISARMED);
	expect_denied(logic.evaluate(nominal_inputs()), Reason::NOT_ARMED);
}

TEST(StateMachine, ArmWithValidParamsLocks)
{
	DropGuardLogic logic = armed_logic();
	EXPECT_EQ(logic.state(), State::ARMED_LOCKED);
	const Evaluation e = logic.evaluate(nominal_inputs());
	EXPECT_TRUE(e.authorized);
	EXPECT_EQ(e.reason, Reason::OK);
}

TEST(StateMachine, ArmWithInvalidParamsFaults)
{
	Params p = nominal_params();
	p.payload_deliverer_enabled = true;
	DropGuardLogic logic = armed_logic(p);
	EXPECT_EQ(logic.state(), State::ARMED_FAULT);
	expect_denied(logic.evaluate(nominal_inputs()), Reason::PARAMS_INVALID);
}

TEST(StateMachine, DisarmReturnsToDisarmed)
{
	DropGuardLogic logic = armed_logic();
	logic.update_arming(false);
	EXPECT_EQ(logic.state(), State::DISARMED);
	expect_denied(logic.evaluate(nominal_inputs()), Reason::NOT_ARMED);

	// Desarmar desde ARMED_FAULT también vuelve a DISARMED.
	Params bad = nominal_params();
	bad.radius_m = 0.f;
	DropGuardLogic faulted = armed_logic(bad);
	faulted.update_arming(false);
	EXPECT_EQ(faulted.state(), State::DISARMED);
}

TEST(StateMachine, RepeatedDisarmNotificationIsNoOp)
{
	DropGuardLogic logic;
	logic.update_params(nominal_params());
	logic.update_arming(false); // ya desarmado
	EXPECT_EQ(logic.state(), State::DISARMED);
	EXPECT_FALSE(logic.param_change_ignored());
}

TEST(StateMachine, NoParamsReceivedBeforeArmingFaults)
{
	// Los parámetros por defecto (todo a cero) no son válidos: armar sin recibirlos da ARMED_FAULT.
	DropGuardLogic logic;
	logic.update_arming(true);
	EXPECT_EQ(logic.state(), State::ARMED_FAULT);
}

TEST(ParamLock, ChangeWhileArmedIsIgnoredAndFlagged)
{
	DropGuardLogic logic = armed_logic();

	// Alguien mueve la zona 1 km al norte con el dron armado.
	Params moved = nominal_params();
	double lat, lon;
	offset(kLat0, kLon0, 1000.0, 0.0, lat, lon);
	moved.lat_deg = lat;
	logic.update_params(moved);

	EXPECT_TRUE(logic.param_change_ignored());
	EXPECT_DOUBLE_EQ(logic.active_params().lat_deg, kLat0);

	// Sobre la zona original sigue autorizando; sobre la nueva, no.
	EXPECT_TRUE(logic.evaluate(nominal_inputs()).authorized);
	Inputs at_new = nominal_inputs();
	at_new.lat_deg = lat;
	expect_denied(logic.evaluate(at_new), Reason::OUTSIDE_RADIUS);
}

TEST(ParamLock, DisablingWhileArmedIsAlsoIgnored)
{
	DropGuardLogic logic = armed_logic();
	Params off = nominal_params();
	off.enable = false;
	logic.update_params(off);
	EXPECT_TRUE(logic.param_change_ignored());
	EXPECT_TRUE(logic.evaluate(nominal_inputs()).authorized);
}

TEST(ParamLock, IdenticalParamsWhileArmedDoNotFlag)
{
	DropGuardLogic logic = armed_logic();
	logic.update_params(nominal_params());
	EXPECT_FALSE(logic.param_change_ignored());
}

TEST(ParamLock, RepeatedArmNotificationDoesNotResnapshot)
{
	DropGuardLogic logic = armed_logic();
	Params moved = nominal_params();
	moved.radius_m = 50.f;
	logic.update_params(moved);
	logic.update_arming(true); // misma notificación de armado repetida
	EXPECT_FLOAT_EQ(logic.active_params().radius_m, 10.f);
	EXPECT_TRUE(logic.param_change_ignored());
}

TEST(ParamLock, DisarmClearsFlagAndNextArmTakesNewParams)
{
	DropGuardLogic logic = armed_logic();
	Params moved = nominal_params();
	moved.radius_m = 50.f;
	logic.update_params(moved);
	logic.update_arming(false);
	EXPECT_FALSE(logic.param_change_ignored());
	EXPECT_FLOAT_EQ(logic.active_params().radius_m, 50.f); // vivos al estar desarmado
	logic.update_arming(true);
	EXPECT_FLOAT_EQ(logic.active_params().radius_m, 50.f);
	EXPECT_FALSE(logic.param_change_ignored());
}

// ---------------------------------------------------------------------------
// Evaluación: condiciones individuales (IR-DG-001, 003, 004, 005)
// ---------------------------------------------------------------------------

TEST(Evaluate, DisabledDeniesEvenWhenArmed)
{
	Params p = nominal_params();
	p.enable = false;
	DropGuardLogic logic = armed_logic(p);
	expect_denied(logic.evaluate(nominal_inputs()), Reason::DISABLED);
}

TEST(Evaluate, NominalReportsDistanceAndRelativeAltitude)
{
	const Evaluation e = armed_logic().evaluate(nominal_inputs());
	EXPECT_TRUE(e.authorized);
	EXPECT_NEAR(e.distance_m, 0.0, 1e-9);
	EXPECT_FLOAT_EQ(e.rel_alt_m, 20.f);
}

TEST(Evaluate, PositionAgeAtTimeoutIsAcceptedOneMicrosecondMoreIsStale)
{
	DropGuardLogic logic = armed_logic();
	Inputs in = nominal_inputs();
	in.pos_timestamp_us = kNow - 200'000U; // exactamente DG_POS_TOUT
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.pos_timestamp_us = kNow - 200'001U;
	expect_denied(logic.evaluate(in), Reason::POS_STALE);
}

TEST(Evaluate, FutureTimestampIsStale)
{
	Inputs in = nominal_inputs();
	in.pos_timestamp_us = kNow + 1U;
	expect_denied(armed_logic().evaluate(in), Reason::POS_STALE);
}

TEST(Evaluate, InvalidOrNonFinitePositionDenied)
{
	DropGuardLogic logic = armed_logic();
	const float fnan = std::numeric_limits<float>::quiet_NaN();
	const double dnan = std::numeric_limits<double>::quiet_NaN();

	Inputs in = nominal_inputs(); in.pos_valid = false;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.lat_deg = dnan;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.lon_deg = dnan;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.alt_amsl_m = fnan;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.eph_m = fnan;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.epv_m = INFINITY;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.eph_m = -0.1f;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
	in = nominal_inputs(); in.epv_m = -0.1f;
	expect_denied(logic.evaluate(in), Reason::POS_INVALID);
}

TEST(Evaluate, HorizontalUncertaintyLimit)
{
	DropGuardLogic logic = armed_logic();
	Inputs in = nominal_inputs();
	in.eph_m = 2.f; // igual al máximo
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.eph_m = 2.01f;
	expect_denied(logic.evaluate(in), Reason::EPH_HIGH);
}

TEST(Evaluate, VerticalUncertaintyLimit)
{
	DropGuardLogic logic = armed_logic();
	Inputs in = nominal_inputs();
	in.epv_m = 3.f; // igual al máximo; la banda recortada es [18, 27] y la altura es 20
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.epv_m = 3.01f;
	expect_denied(logic.evaluate(in), Reason::EPV_HIGH);
}

TEST(Evaluate, HomeAltitudeRequired)
{
	DropGuardLogic logic = armed_logic();
	Inputs in = nominal_inputs();
	in.home_alt_valid = false;
	expect_denied(logic.evaluate(in), Reason::HOME_INVALID);
	in = nominal_inputs();
	in.home_alt_amsl_m = std::numeric_limits<float>::quiet_NaN();
	expect_denied(logic.evaluate(in), Reason::HOME_INVALID);
}

TEST(Evaluate, RadiusBoundaryIncludesUncertainty)
{
	DropGuardLogic logic = armed_logic(); // radio 10 m
	Inputs in = nominal_inputs();
	in.eph_m = 0.5f;
	double lat, lon;

	offset(kLat0, kLon0, 9.49, 0.0, lat, lon); // 9.49 + 0.5 = 9.99 <= 10
	in.lat_deg = lat; in.lon_deg = lon;
	EXPECT_TRUE(logic.evaluate(in).authorized);

	offset(kLat0, kLon0, 9.51, 0.0, lat, lon); // 9.51 + 0.5 = 10.01 > 10
	in.lat_deg = lat; in.lon_deg = lon;
	expect_denied(logic.evaluate(in), Reason::OUTSIDE_RADIUS);
}

TEST(Evaluate, LargerUncertaintyShrinksTheZone)
{
	DropGuardLogic logic = armed_logic();
	Inputs in = nominal_inputs();
	double lat, lon;
	offset(kLat0, kLon0, 0.0, 9.0, lat, lon); // 9 m al este
	in.lat_deg = lat; in.lon_deg = lon;

	in.eph_m = 0.9f; // 9.9 <= 10
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.eph_m = 1.5f; // 10.5 > 10
	expect_denied(logic.evaluate(in), Reason::OUTSIDE_RADIUS);
}

TEST(Evaluate, FarAwayIsOutside)
{
	Inputs in = nominal_inputs();
	double lat, lon;
	offset(kLat0, kLon0, 800.0, -600.0, lat, lon);
	in.lat_deg = lat; in.lon_deg = lon;
	const Evaluation e = armed_logic().evaluate(in);
	expect_denied(e, Reason::OUTSIDE_RADIUS);
	EXPECT_NEAR(e.distance_m, 1000.0, 0.01);
}

TEST(Evaluate, AltitudeBandBoundariesIncludeUncertainty)
{
	DropGuardLogic logic = armed_logic(); // banda [15, 30]
	Inputs in = nominal_inputs();
	in.epv_m = 0.5f;                       // banda efectiva [15.5, 29.5]

	in.alt_amsl_m = kHomeAlt + 15.5f;
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.alt_amsl_m = kHomeAlt + 15.25f;
	expect_denied(logic.evaluate(in), Reason::BELOW_BAND);

	in.alt_amsl_m = kHomeAlt + 29.5f;
	EXPECT_TRUE(logic.evaluate(in).authorized);
	in.alt_amsl_m = kHomeAlt + 29.75f;
	expect_denied(logic.evaluate(in), Reason::ABOVE_BAND);
}

TEST(Evaluate, WorksWithNegativeLongitudeZone)
{
	Params p = nominal_params();
	p.lat_deg = 43.3183;
	p.lon_deg = -1.9812;
	DropGuardLogic logic = armed_logic(p);
	Inputs in = nominal_inputs();
	double lat, lon;
	offset(p.lat_deg, p.lon_deg, 3.0, -4.0, lat, lon); // 5 m
	in.lat_deg = lat; in.lon_deg = lon;
	const Evaluation e = logic.evaluate(in);
	EXPECT_TRUE(e.authorized);
	EXPECT_NEAR(e.distance_m, 5.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Orden de evaluación: se informa la primera condición que falla (diseño §3)
// ---------------------------------------------------------------------------

TEST(Precedence, FirstFailingConditionIsReported)
{
	Inputs bad = nominal_inputs();
	bad.pos_timestamp_us = 0U;           // antigua
	bad.pos_valid = false;               // no válida
	bad.eph_m = 5.f;                     // imprecisa
	bad.home_alt_valid = false;          // sin home

	Params disabled = nominal_params();
	disabled.enable = false;
	expect_denied(armed_logic(disabled).evaluate(bad), Reason::DISABLED);

	DropGuardLogic disarmed;
	disarmed.update_params(nominal_params());
	expect_denied(disarmed.evaluate(bad), Reason::NOT_ARMED);

	expect_denied(armed_logic().evaluate(bad), Reason::POS_STALE);

	bad.pos_timestamp_us = kNow;
	expect_denied(armed_logic().evaluate(bad), Reason::POS_INVALID);

	bad.pos_valid = true;
	expect_denied(armed_logic().evaluate(bad), Reason::EPH_HIGH);

	bad.eph_m = 0.5f;
	bad.epv_m = 5.f;
	expect_denied(armed_logic().evaluate(bad), Reason::EPV_HIGH);

	bad.epv_m = 0.5f;
	expect_denied(armed_logic().evaluate(bad), Reason::HOME_INVALID);
}

TEST(Precedence, OutsideRadiusBeforeAltitudeBand)
{
	Inputs in = nominal_inputs();
	double lat, lon;
	offset(kLat0, kLon0, 50.0, 0.0, lat, lon);
	in.lat_deg = lat; in.lon_deg = lon;
	in.alt_amsl_m = kHomeAlt + 50.f; // también fuera de banda
	expect_denied(armed_logic().evaluate(in), Reason::OUTSIDE_RADIUS);
}
