/**
 * @file DropGuardLogic.hpp
 *
 * Lógica de autorización de suelta de drop_guard (IDAL C, ADR-007).
 *
 * Clase C++ pura, sin dependencias de PX4: la envuelve el módulo DropGuard,
 * que le pasa los datos de uORB y los parámetros DG_*.
 *
 * Reglas de codificación (DOC-09 §1, subconjunto MISRA C++):
 *  - sin memoria dinámica ni excepciones ni RTTI;
 *  - sin estado global; todo el estado vive en DropGuardLogic;
 *  - todas las entradas en coma flotante se comprueban con std::isfinite.
 *
 * Trazabilidad: IR-DG-001, 003, 004, 005, 006, 007 (diseño de drop_guard §6).
 */

#pragma once

#include <cstdint>

namespace drop_guard
{

/** Motivo del resultado. El orden coincide con el de evaluación (diseño §3). */
enum class Reason : uint8_t {
	OK = 0,
	DISABLED,
	NOT_ARMED,
	PARAMS_INVALID,
	POS_STALE,
	POS_INVALID,
	EPH_HIGH,
	EPV_HIGH,
	HOME_INVALID,
	OUTSIDE_RADIUS,
	BELOW_BAND,
	ABOVE_BAND,
};

/** Estados del módulo (diseño §3). */
enum class State : uint8_t {
	DISARMED = 0,
	ARMED_LOCKED,
	ARMED_FAULT,
};

/** Parámetros DG_* ya convertidos a unidades de trabajo. */
struct Params {
	bool enable{false};                  ///< DG_ENABLE
	double lat_deg{0.0};                 ///< DG_LAT, grados WGS84
	double lon_deg{0.0};                 ///< DG_LON, grados WGS84
	float radius_m{0.f};                 ///< DG_RADIUS
	float alt_min_m{0.f};                ///< DG_ALT_MIN, sobre el punto de despegue
	float alt_max_m{0.f};                ///< DG_ALT_MAX, sobre el punto de despegue
	float eph_max_m{0.f};                ///< DG_EPH_MAX
	float epv_max_m{0.f};                ///< DG_EPV_MAX
	int32_t pos_timeout_ms{0};           ///< DG_POS_TOUT
	int32_t zone_hash{0};                ///< DG_ZONE_HASH (solo se informa)
	bool payload_deliverer_enabled{false}; ///< PD_GRIPPER_EN != 0
};

/** Datos de entrada de una evaluación (de vehicle_global_position y home_position). */
struct Inputs {
	uint64_t now_us{0};                  ///< Instante de la evaluación
	bool pos_valid{false};               ///< Posición global declarada válida por el EKF
	uint64_t pos_timestamp_us{0};        ///< Marca de tiempo de la posición
	double lat_deg{0.0};
	double lon_deg{0.0};
	float alt_amsl_m{0.f};               ///< Altitud de la aeronave (misma referencia que home)
	float eph_m{0.f};                    ///< Incertidumbre horizontal (1 sigma)
	float epv_m{0.f};                    ///< Incertidumbre vertical (1 sigma)
	bool home_alt_valid{false};
	float home_alt_amsl_m{0.f};
};

/** Resultado de una evaluación. distance_m y rel_alt_m solo son significativos si se calcularon. */
struct Evaluation {
	Reason reason{Reason::NOT_ARMED};
	bool authorized{false};
	double distance_m{0.0};
	float rel_alt_m{0.f};
};

/** Límites de validez de los parámetros (coinciden con module.yaml). */
namespace limits
{
constexpr float kRadiusMin = 1.f;
constexpr float kRadiusMax = 100.f;
constexpr float kAltMin = 5.f;
constexpr float kAltMax = 120.f;
constexpr float kEpMin = 0.5f;
constexpr float kEpMax = 10.f;
constexpr int32_t kTimeoutMinMs = 50;
constexpr int32_t kTimeoutMaxMs = 1000;
} // namespace limits

/** Radio medio de la Tierra usado en la aproximación equirectangular [m]. */
constexpr double kEarthRadiusM = 6371000.0;

/**
 * Distancia horizontal entre dos puntos por aproximación equirectangular.
 * Válida para distancias cortas (< 1 km); normaliza la diferencia de longitud
 * a [-180, 180] grados. Devuelve NaN si alguna entrada no es finita.
 */
double horizontal_distance_m(double lat0_deg, double lon0_deg, double lat_deg, double lon_deg);

/**
 * Comprueba que un juego de parámetros es utilizable (IR-DG-007):
 * valores finitos y en rango, banda de altura no vacía una vez recortada
 * por DG_EPV_MAX, y payload_deliverer desactivado.
 */
bool validate_params(const Params &p);

/** true si los dos juegos de parámetros son idénticos campo a campo. */
bool params_equal(const Params &a, const Params &b);

class DropGuardLogic
{
public:
	DropGuardLogic() = default;

	/**
	 * Notifica el estado de armado. En la transición a armado captura una copia
	 * de los parámetros vigentes y decide ARMED_LOCKED o ARMED_FAULT (IR-DG-006/007).
	 * En la transición a desarmado vuelve a DISARMED y borra la marca de cambio.
	 */
	void update_arming(bool armed);

	/**
	 * Notifica los parámetros actuales. Siempre se guardan como parámetros vivos;
	 * con el dron armado no afectan a la evaluación y, si difieren de la copia,
	 * se marca el cambio ignorado (IR-DG-006).
	 */
	void update_params(const Params &live);

	/** Evalúa si una orden de apertura está autorizada con los datos dados (IR-DG-001). */
	Evaluation evaluate(const Inputs &in) const;

	State state() const { return _state; }
	bool param_change_ignored() const { return _param_change_ignored; }

	/** Parámetros que se usan en la evaluación: la copia si está armado, los vivos si no. */
	const Params &active_params() const;

private:
	State _state{State::DISARMED};
	Params _live{};
	Params _snapshot{};
	bool _param_change_ignored{false};
};

} // namespace drop_guard
