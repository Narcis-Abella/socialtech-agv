# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-11 · Rama: `feat/global-reloc` (commits locales, sin push) · F1 congelado `GR-S2S3-F1` = `c026a7b`, intacto · Candidato F2 `GR-S2S3-F2c` = `F2b` + `4211c6b` + `6cb779a`; gate `8a88d16` sin cambios.

## Fase actual
Fase 1 **VERIFIED**. Fase 2 **VERIFIED — VALIDATED_IN_REPLAY** (notas 21 y 22). Fase 3 no iniciada. Sin validación física; seguridad completa del AGV no demostrada; F4-F6 pendientes. `can0` sin uso.

## Último resultado (nota 22: latencia de revocación)
- **Causa:** una comprobación contra el mapa cada 3 s y cada una tardaba 1 s (proceso nuevo). Tras divergir FAST-LIO2 el permiso sobrevivía 1,6-2,5 s.
- **Corrección:** trabajador persistente, comprobación cada ~0,55 s, permiso solo con una comprobación pasada sobre escaneos de ≤2 s, y regla adicional sobre los 0,5 s más recientes. Ningún umbral de aceptación cambiado.
- **Medido (árbol final, 23 revocaciones):** 13 con 0 s; resto 0,35-1,46 s. Controles sin fallo: 0 retiradas.
- Regresiones: S2 y S3 correctos, 14/14 negativos rechazados, 41 casos de la matriz como en la nota 21, h27 6/7 sin publicar (1 pose correcta publicada, sin permiso), celda del gate en vcan correcta.
- Coste: +17,5 puntos de un núcleo (pila ~95 % de un núcleo de 8); memoria igual.
- Los 50 reinicios no se repitieron: son evidencia de `F2b` (justificación en nota 22, §8).

## Problemas principales
- Latencia residual hasta 1,46 s: un error a lo largo de un pasillo pasa la regla congelada ~1 s. Menos exige odometría de ruedas.
- Paradas espurias de ~0,6 s: 3 en 9,4 min en hospital/eco03 (GICP en su límite de iteraciones; regla congelada, no tocada).
- NVMe: 12 timeouts hoy; una ejecución inválida por un bloqueo de 26,9 s (repetida).
- Sin verificar: timeout del firmware, CAN físico, entrega en movimiento (nota 21).

## Decisiones pendientes de Narcís
1. Aprobar o cambiar el requisito propuesto **R-REV = 2,5 s** (pose fuera de 0,30 m / 8° → salida cero).
2. Periodo de comprobación: 0,5 s (actual) o 1 s (~la mitad de CPU, hasta +0,5 s).
3. Despliegue de la celda CAN, NVMe (`sudo nvme smart-log /dev/nvme0`) y las cuatro de la nota 20.

## Siguientes acciones
1. Revisión de Narcís de la nota 22 (§5, §6, §8).
2. Fase 3 solo tras su visto bueno.

## Referencias
Notas `22-fase2-latencia-revocacion.md`, `21-fase2-robustez.md`; plan `19-plan-cierre-y-entrega.md`; identidad `results/22-f2-latency/f2l/candidate-identity.txt`. NX: `/home/jetson/work/reloc-f2l-20261011/` (final), `reloc-f2i`, `f2j`, `f2k`, `f2m`, `reloc-f2h-20261011/` (F2b).
