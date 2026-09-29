# drone-px4

Fork de [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) para el dron de reparto urbano de última milla
(proyecto de aprendizaje personal; proceso inspirado en ARP4754A/ARP4761A, sin certificación).
Nivel de control: **CC1** (DOC-09 §5).

## Ramas

- `main`: **espejo de upstream**. No se trabaja aquí; solo se sincroniza con PX4.
- `drone`: el trabajo propio. Parte del tag `v1.17.0` de PX4 (commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`)
  y añade el módulo `drop_guard` como commits reales.

## Qué añade `drone` sobre PX4 v1.17.0

- `src/modules/drop_guard/`: validación de las órdenes de suelta de carga, con tests unitarios.
- `msg/DropGuardStatus.msg` y su publicación por DDS (`dds_topics.yaml`).
- Integración en el arranque y en las placas `sitl` y `fmu-v6c`.

## Tags

`v1.17.0-drone.N` son tags de trabajo sobre la rama `drone`. **No son baselines** del plan de configuración
(las baselines usan el formato `vFASE.BASELINE.N` y se fijan en `drone.repos` de `drone-sim`).

## Uso

No se compila a mano: `drone-sim/scripts/setup_native.sh` clona este repo en el tag fijado en `drone.repos`.

## Integración continua

GitHub Actions está **desactivado** en este fork: heredaría los workflows de PX4, muy pesados. La verificación
la hace el `nightly` de `drone-sim`.

## Licencia

La de PX4 (BSD-3-Clause), sin cambios. Ver `LICENSE`.
