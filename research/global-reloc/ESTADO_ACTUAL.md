# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-10 · Rama: `feat/global-reloc` · HEAD: ver `git log -1` (commits locales, sin push) · Árbol limpio tras el commit de la sesión 3.

## Fase actual
Integración online ROS 2 completada y medida en la NX (replay 1x, DEVELOPMENT). Fases 1-5 hechas; sin push.

## Completado
- Nodo `reloc_node.py` + núcleo `reloc_core.py` (12 tests): puerta IMU → FAST-LIO2 en frío (lo lanza el nodo) → submapa S3 → worker `search_v2` (FFT + small_gicp + puerta congelada, con deadline 60 s matable) → `/initialpose` solo tras ACCEPTED → confirmación AMCL con odometría propia. Imagen `socialtech:reloc-online` (robot-verify + scipy).
- Resultados (nota 15): 18/18 aceptaciones correctas de 22 ventanas válidas (3 sin referencia, 1 error de arranque), 0 wrong_accept, 0/10 poses en negativos (mapa equivocado, hueco). Todas llegaron a LOCALIZED.
- Latencia S3 (first input→accept): mediana 39.8 s, p95 47.1, máx 47.8; →LOCALIZED 45.8/52.6/53.4. Etapas en nota 15. CPU media 69 % de un núcleo (pico 213 %), PSS ≈339 MB, GPU 0 %.
- Fallos: sensores ausentes, pérdidas 30 %, sellos erróneos, LIO caído/colgado, small_gicp roto, AMCL ausente, deadline, reinicio con SIGKILL, segundo intento: todos conservadores, ninguna pose falsa publicada.

## Limitaciones
- Datos de desarrollo; sin tasa de fiabilidad. S3 depende del movimiento (2.8 m) o del límite de 30 s; robot parado en el arranque ≈49 s (estimado, no medido). S2 solo N=1.
- Cold boot NO medido (17.1 s de arranque host; con pila ≈70 s estimado: no cabe en un único presupuesto de 60 s).
- Extrínseca, altura (0.53) y covarianza inicial provisionales; sensor mirando al techo; pérdida de mensajes del replay 7-8 %.
- Puerta por banda (nota 14) sin activar.

## Siguiente acción
1. Revisión de Narcís de la nota 15 (definición de reloj, cold boot, S2 vs S3 para arranque parado).
2. Repetir negativos y positivos tras añadir `lio_stalled`; más ventanas S2 y arranque parado.
3. Calibración física y repetición con montaje hacia abajo; medir en Nano.

## Referencias
Notas `13`, `14`, `15-integracion-online-latencia.md`, protocolo `02`; tablas `results/15-online/`; código `tools/global_reloc_3d/`. NX: `/home/jetson/work/reloc-online-20261009/runs/`.
