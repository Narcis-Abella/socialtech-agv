# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-09 · Rama: `feat/global-reloc` · HEAD: ver `git log -1` (commits locales, sin push) · Working tree: limpio tras el commit de la sesión 2.

## Objetivo
Pose inicial (x, y, yaw) automática en un mapa conocido, sin operador. Aceptar solo si XY ≤ 0.30 m y yaw ≤ 8°; rechazar ante la duda.

## Progreso
- Puntos 1–6 cerrados; AMCL global = `AMCL_WEAK_BASELINE`.
- Método v2: FAST-LIO2 → nivelación IMU → búsqueda FFT exhaustiva (x, y, z, yaw) → small_gicp → puerta sin cambios. KISS no interviene en v2 (nota 13).
- Fase actual: validación entre sesiones, seguridad, S3 certificado y handoff a AMCL (completados en DEVELOPMENT).

## Últimos resultados (DEVELOPMENT)
- **Entre sesiones (consultas eco03, mapa eco01):** 10/10 correct_accept (Wilson ≥0.72); XY mediana 0.010 m, máx. 0.040 m. Enlace de frames mapa a mapa con incertidumbre ~1–2 cm.
- S3 con arranque certificado por IMU: 17/18 correct_accept; 1 rechazo falso por la puerta de RMSE.
- Negativos: 0/88 aceptados (Wilson ≤4.2 %), más 0/50 perturbados.
- Handoff a AMCL sin referencia: 3/3 convergen en t = 0, 0 pérdidas (incluido eco03 sobre el PGM de eco01).

## Trabajo actual
Seguridad: en 17 de 136 negativos solo una puerta frenó la hipótesis (16 veces el margen). La confirmación temporal llega a coincidir en poses erróneas. El soporte por banda separa los casos (correctos ≥0.962, erróneos ≤0.715). Propuesta en `14-propuesta-puerta-banda.md`, sin aplicar.

## Pendientes / bloqueos
- Decisión de Narcís sobre la puerta por banda, sobre la puerta de RMSE y sobre los datos para calibrar el umbral.
- Extrínseca sensor→base y altura del sensor sin calibrar: la integración no está completa.
- Solo dos edificios; una sola sesión independiente (eco03). Hospital sin segunda sesión.

## Próximas acciones
1. Revisión de Narcís de las notas 13 y 14 y de las figuras.
2. Nuevo bag o sesión para calibrar T_band (datos no usados).
3. Nodo online: búsqueda dentro de la sesión de FAST-LIO2 → map→odom; latencia real y PSS en NX y Nano.
4. Calibración física del montaje y repetición tras el remontaje hacia abajo.
5. Más sesiones independientes (hospital, otras plantas).

## Referencias
- Notas `13-generalizacion-registro-3d.md`, `14-propuesta-puerta-banda.md`, protocolo `02`, AMCL `10`.
- Código `tools/global_reloc_3d/` (`search_v2.py`, `correlative_search.py`, `imu_ready.py`, `map_link.py`, `handoff_pose.py`, `batch_sessions.sh`).
- Resultados `results/13-*`, `results/14-*`; figura `figures/v2-band-overlays.png`.
- NX: `/home/jetson/work/global-reloc-3d-20261009/claude-v1/`; mapa eco03 en `/home/jetson/work/maps/eco03_1x/`.
