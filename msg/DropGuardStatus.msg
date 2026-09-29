# Estado del módulo drop_guard (filtro de suelta de carga por zona).
# Se publica a 5 Hz y se exporta a ROS 2 por uXRCE-DDS (/fmu/out/drop_guard_status).

uint64 timestamp                # time since system start (microseconds)

uint8 state                     # Estado del módulo
uint8 STATE_DISARMED = 0
uint8 STATE_ARMED_LOCKED = 1
uint8 STATE_ARMED_FAULT = 2

bool authorized                 # Una orden de apertura ahora mismo estaría autorizada
uint8 reason                    # Motivo de la evaluación actual (drop_guard::Reason, 0 = OK)
float32 distance_m              # Distancia al centro de la zona (solo válida si se llegó a calcular)
float32 rel_alt_m               # Altura sobre el punto de despegue (solo válida si se llegó a calcular)

bool param_change_ignored       # Se cambiaron parámetros DG_* con el dron armado y se ignoraron
int32 zone_hash                 # DG_ZONE_HASH de la zona en uso

uint8 last_result               # Resultado de la última orden de apertura
uint8 RESULT_NONE = 0
uint8 RESULT_RELEASED = 1
uint8 RESULT_DENIED = 2
uint8 last_deny_reason          # Motivo del último rechazo
uint32 release_count            # Sueltas autorizadas desde el arranque
uint32 deny_count               # Sueltas denegadas desde el arranque
