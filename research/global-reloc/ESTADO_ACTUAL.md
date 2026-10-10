# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-10 · Rama: `feat/global-reloc` · Candidato congelado `GR-S2S3-F1` = commit `c026a7b` (commits locales, sin push).

## Fase actual
Fase 1 (auditoría y congelación) **VERIFIED**. Fase 2 no iniciada: espera revisión. Nivel: `VALIDATED_IN_REPLAY`, datos de desarrollo. Nada probado en hardware.

## Último resultado (nota 20)
- Auditoría de `reloc_core.py` / `reloc_node.py`: una sola publicación de `/initialpose`, solo desde `HANDOFF` tras `ACCEPTED`; ningún código del método lee la referencia; un reloj de 60 s sin reinicio S2/S3.
- 2 bugs demostrados y corregidos con regresión: (B1) un salto de sello IMU bloqueaba el nodo y el deadline (5,5 s por cada 1e5 s de salto); (B2) el contenedor del método montaba el directorio del mapa, que contiene `alidarState.txt` (nunca leído; ahora no se monta y hay guarda).
- Nota 16 recalculada desde las tablas: coincide, con 1 errata (el máx. XY 0,068 m es de S3; S2 máx. 0,052 m).
- Reproducción NX 1x con el árbol congelado: S2 eco01 z03 correcto (18,9 s, 0,014 m); S3 hospital z00 correcto (41,8 s, 0,067 m; primer intento no puntuable por hueco de referencia, conservado); mapa equivocado rechazado, sin pose.
- Tests: 43 pasan, 1 omitido (portátil e imagen NX); `nav_bench` completo en la imagen.

## Problemas principales
- Fiabilidad desconocida: 5 bloques estacionarios, mapas propios.
- Hallazgos abiertos O1-O9 (nota 20): sin sensores no hay estado terminal; tras `HANDOFF_FAILED` AMCL conserva la pose; sin supervisión de AMCL tras `LOCALIZED`; memoria del nodo sin acotar; `register_v2` sin receta de compilación.
- Cuelgue de arranque de FAST-LIO2 (~1/100), causa desconocida.
- Montaje, altura y covarianza provisionales; sensor hacia el techo; cold boot sin medir.

## Decisiones pendientes de Narcís (nota 20, sección 7)
1. Deadline único 60 s primer sensor → `ACCEPTED`; `LOCALIZED` y cold boot aparte.
2. Regla de muestras IMU: 95 % escrito, 99,3-99,5 % efectivo (se mantiene congelada).
3. Convenio `base_footprint` (provisional).
4. Puerta de aceptación congelada; puerta por banda solo diagnóstica.

## Siguientes acciones
1. Revisión humana de la Fase 1.
2. Fase 2 (robustez): O1-O4, fallos no probados, 200 arranques de FAST-LIO2.
3. Fases 4-6 bloqueadas por medición física y grabación de datos.

## Referencias
Plan `19-plan-cierre-y-entrega.md`; auditoría `20-auditoria-fase1-congelacion.md`; identidad `results/19-f1-audit/candidate-identity.txt`; notas 15-18. NX: `/home/jetson/work/reloc-f1-audit-20261010/` (auditoría), `reloc-s2s3-20261010/` (nota 16, intacta).
