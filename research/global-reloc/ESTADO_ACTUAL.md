# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-10 · Rama: `feat/global-reloc` · HEAD: ver `git log -1` (commits locales, sin push) · Árbol limpio tras el commit de la sesión 4.

## Fase actual
S2 estacionario → S3 supervisado implementado y medido en replay 1x en la NX (DEVELOPMENT). Nivel: `VALIDATED_IN_REPLAY`. Nada probado en hardware.

## Último resultado (nota 16)
- Modo `s2s3`: un solo reloj de 60 s; S3 solo tras `S2_INSUFFICIENT` + autorización externa (`/global_reloc/s3_authorize`); sin autorización, TIMEOUT a 38 s.
- S2 3 s con arranque realmente parado: 8/10 aceptadas en S2 (5 bloques independientes), 2 resueltas por S3; todas correctas. ACCEPTED 16.3 s mediana (máx 18.9), LOCALIZED 21.8 s.
- S3 tras S2: 21 aceptaciones, ACCEPTED 42.0/47.4/47.6 s. Total: 50 poses puntuadas, 0 wrong_accept; 16/16 negativos rechazados, 0 poses.
- 30 ensayos de fallo: ninguna pose sin ACCEPTED, 0 procesos huérfanos. Watchdogs sin falsos disparos en 78 ejecuciones.
- CPU media 81 % de un núcleo (pico 257 %), PSS pico 319 MB (máx 430), GPU 0 %.
- Corregido: la "pérdida IMU 7-8 %" era un artefacto del estimador (flujo completo a 200 Hz). LiDAR: 2.2-2.8 % de tramas ausentes ya en los bags.
- Corregido: el "S2 N=1" de la nota 15 no era estacionario.

## Problemas principales
- Solo 5 bloques estacionarios, datos de desarrollo; el protocolo pide ≥30.
- FAST-LIO2 se colgó al arrancar 1 vez de ~100 (causa desconocida; terminó en ERROR sin pose).
- S3 con supervisor humano probablemente no cabe en 60 s: enmienda propuesta, sin aplicar.
- Puerta IMU: el criterio de muestras equivale a 99.3 %, no 95 % (propuesta, sin aplicar).
- Cold boot sin medir. Montaje, altura (0.53) y covarianza provisionales; sensor hacia el techo.
- Puerta por banda (nota 14): solo diagnóstico, sin activar.

## Siguientes acciones
1. Narcís: decidir enmienda S3 supervisado, criterio de muestras IMU, definición de `base_footprint`.
2. Medir el montaje (nota 17, sección C) y remontar; regrabar mapa.
3. Hardware parado: checklist H1 de la nota 18 (incluye cold boot y salud de flujos en vivo).
4. Grabar sesiones independientes (nota 18) y evaluar con parámetros congelados.

## Referencias
Notas `16-s2-s3-watchdogs-latencia.md`, `17-calibracion-montaje.md`, `18-plan-datos-independientes-y-validacion-fisica.md`, `15`, protocolo `02`. Tablas `results/16-s2s3/`, figura `figures/16-accept-time.png`. Código `tools/global_reloc_3d/` (`reloc_core.py`, `reloc_node.py`, `batch_s2s3.sh`, `batch_faults_s2s3.sh`). NX: `/home/jetson/work/reloc-s2s3-20261010/`.
