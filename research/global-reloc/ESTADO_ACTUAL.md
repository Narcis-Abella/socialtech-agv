# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-09 · Rama: `feat/global-reloc` · HEAD: ver `git log -1` (commits locales, sin push) · Working tree: limpio tras el commit v2.

## Objetivo
Pose inicial (x, y, yaw) automática en un mapa conocido, sin operador. Aceptar solo si XY ≤ 0.30 m y yaw ≤ 8°; rechazar ante la duda.

## Progreso
- Puntos 1–6 cerrados; AMCL global = `AMCL_WEAK_BASELINE`.
- Punto 7 v1 (KISS + small_gicp): 1 lugar correcto; falla en hospital por descriptores FPFH (≤1 % de pares correctos, V).
- Punto 7 v2: nivelación IMU + búsqueda correlativa exhaustiva (x, y, z, yaw) + small_gicp + la misma puerta (nota 13).

## Últimos resultados (DEVELOPMENT, mapas de la misma sesión)
- Mapa propio: 14/15 correct_accept (9 eco01 + 6 hospital, a ≥5 m); XY mediana 0.023 m, máx. 0.043; yaw máx. 1.6°.
- 1 rechazo (eco01 w08): FAST-LIO2 arrancado girando, consulta corrupta; rechazo correcto.
- Controles: mapa cruzado 15/15 y mapa con hueco de 8 m 15/15 rechazados. Wrong accepts 0/45 (Wilson ≤7.9 %).
- Ablación, mismos inputs: KISS v1 3/15 frente a v2 14/15; ningún wrong accept.
- Búsqueda: 9.7 s mediana (máx. 23 s), CPU 14 s, RSS ~90 MiB. v2 determinista (3/3).

## Trabajo actual
La puerta de soporte total (≥0.70) deja pasar hipótesis erróneas hasta 0.76; solo las frenó el margen. El soporte en la banda 0.10–1.80 m separa los casos (correctos ≥0.97, erróneos ≤0.65). Pendiente de enmienda.

## Pendientes / bloqueos
- Decisión de Narcís: puerta de soporte por banda (protocolo §145) y datos para calibrar su umbral.
- Ventanas S3 no certificadas como estacionarias al inicio (desviación documentada).
- Sin validación entre sesiones; sin handoff a AMCL; latencia integrada, PSS y Nano sin medir.

## Próximas acciones
1. Mapa de eco03 con `make_map.sh` → consultas eco03 sobre el mapa eco01 (independencia de sesión).
2. Implementar la puerta por banda como variante con nombre propio, tras la enmienda.
3. S3 desde arranques certificados casi estacionarios.
4. Handoff: pose aceptada → `/initialpose` → convergencia de AMCL en replay.
5. Coste: PSS/tegrastats, menos candidatos GICP, Nano.

## Referencias
- Protocolo `02-protocolo-evaluacion.md`; AMCL `10-amcl-cierre-exploratorio.md`; 3D `11`, `12`, `13-generalizacion-registro-3d.md`.
- Código `tools/global_reloc_3d/` (`search_v2.py`, `correlative_search.py`, `register_v2.cpp`, `batch_generalization.sh`).
- Resultados `research/global-reloc/results/13-v2-*.csv`; figura `figures/v2-band-overlays.png`.
- Datos NX `/home/jetson/work/global-reloc-3d-20261009/claude-v1/` (`search/`, `inputs/`, `caps/`, `frozen-*.sha256`).
