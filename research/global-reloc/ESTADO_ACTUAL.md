# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-11 · Rama: `feat/global-reloc` (PR hacia `feat/nav-bench-a`, abierta para revisión) · F1 congelado `GR-S2S3-F1` = `c026a7b`, intacto · Candidato `GR-S2S3-F2c` congelado (F2b + `4211c6b` + `6cb779a`; gate `8a88d16`).

## Fase actual
F1 y F2 **VERIFIED**. Fase 3 **VERIFIED — VALIDATED_IN_REPLAY_ON_JETSON_NX** (cerrada 2026-10-11, nota 23 sección 10), con exclusiones explícitas: Nano no medida, arranque en frío (reloj A) BLOCKED_PHYSICAL, sin Nav2 en paralelo. Sin validación física; seguridad completa del AGV no demostrada. F4/F5/F6 BLOCKED_PHYSICAL. F7 **VERIFIED como entrega de software** (documentación, sin validación física): entrada única `24-entrega-final.md`. F4 no iniciada. **R-REV-SW = 2,5 s adoptado** como requisito interno de software (replay), no límite de seguridad física. `can0` sin uso.

## Último resultado (nota 23)
- Recursos (99 ejecuciones del árbol F2c + 14 nuevas): RAM del sistema máx. 2,2 GB, swap constante, GPU 0 %, Tj máx. 67,2 °C, pico 14,05 W, ≤3,1 núcleos de 8 (p95).
- Tiempos: S2 acepta a 18,8–18,95 s, S3 a 41,8–41,96 s (reloj de 60 s: márgenes 41 s y 18 s). Ruido de repeticiones idénticas: 0,15–0,29 s.
- Cuota de CPU: 4 núcleos sin cambio; 2 núcleos +2,1–2,4 s (S3 44,25 s), mismo resultado. Revocación a 2 núcleos 0,01–0,65 s; control sin fallo sin retiradas.
- NVMe: 145 timeouts de kernel desde el 5/10 (51 hoy); 8 de 113 ejecuciones coincidieron, solo una con hueco real (h25, ya repetida); ninguna de las 14 nuevas.
- Sin cambios funcionales; solo harness (`DOCKER_CPUS`, `f3_summary.py`).

## Problemas principales
- NVMe: causa desconocida; falta `sudo nvme smart-log /dev/nvme0`.
- Latencia de revocación hasta 1,46 s; R-REV-SW = 2,5 s adoptado (software), parada real/firmware/CAN/movimiento pendientes de Narcís.
- Paradas espurias ~0,6 s (solver); sin verificar timeout del firmware, CAN físico, entrega en movimiento.

## Decisiones pendientes de Narcís
1. Valorar 0,5–0,85 m de recorrido a ciegas y la parada física (R-REV-SW ya adoptado como requisito de software).
2. Periodo de comprobación 0,5 s o 1 s.
3. Ejecutar `sudo nvme smart-log /dev/nvme0`; decidir si la Nano es opción de despliegue (habría que medirla).
4. Celda CAN y decisiones de las notas 20 y 21.

## Siguientes acciones
1. Revisión de Narcís de `24-entrega-final.md` (resumen en sección 9) y de las notas 22 y 23.
2. F4 solo tras su visto bueno (requiere hardware).

## Referencias
Notas `23-fase3-rendimiento.md`, `22-fase2-latencia-revocacion.md`; plan `19-plan-cierre-y-entrega.md`; NX `/home/jetson/work/reloc-f3-20261011/`.
