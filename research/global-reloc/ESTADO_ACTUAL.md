# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-10 · Rama: `feat/global-reloc` (commits locales, sin push) · F1 congelado `GR-S2S3-F1` = `c026a7b`, intacto · Candidato F2 `GR-S2S3-F2` = método `98926f7`, arnés `d8c2e05`.

## Fase actual
Fase 1 **VERIFIED**. Fase 2 (robustez) **PARTIAL**: un defecto crítico abierto. Fase 3 no iniciada. Nivel: `VALIDATED_IN_REPLAY`, datos de desarrollo. Nada probado en hardware; el robot no se ha movido; `can0` sin uso.

## Último resultado (nota 21)
- O1-O4 corregidos y probados: timeout sin sensores (30 s), permiso de navegación con latido + gate de fallo cerrado, `LOCALIZED` revocable (`LOCALIZATION_LOST`), buffers acotados. Añadido: rechazo de marcas de tiempo desordenadas (caso h15, aceptación errónea real, corregida).
- Matriz de fallos: 46 casos válidos; 43 conformes, 2 predicciones mías erróneas con comportamiento correcto (h44, h45), **1 inseguro (h27)**.
- 200 reinicios software: 188 `LOCALIZED` correctos, 12 timeouts seguros (dos ventanas), 0 aceptación errónea, 0 fallos de arranque de FAST-LIO2.
- Gate contra `tracer_base` real en vcan: bloquea, revoca en 0,39 s; riesgos medidos (bypass, último comando retenido).
- Inyector de F1 no entregaba a FAST-LIO2: `g03`/`f03` de las notas 15-16 eran artefacto (dan `LOCALIZED`); `g05`/`f05` confirmados.

## Problemas principales
- **Abierto, crítico (nota 21 §4b):** si FAST-LIO2 diverge durante la búsqueda S2, la pose se propaga por odometría corrupta y sale un `/initialpose` ~3,5 m erróneo (2 de 3 ejecuciones de h27). Sin permiso de navegación en ninguna. Requiere decisión de diseño (reverificar contra el mapa al publicar).
- El NVMe de la NX bloquea E/S hasta 30 s (75 timeouts de kernel desde el 5-oct).
- Timeout de comandos del firmware de la base sin verificar; el gate a nivel de topic se puede saltar.
- Memoria solo medida 8 min; cold boot y driver real sin probar.

## Decisiones pendientes de Narcís
1. Corrección de §4b: opción A (reverificación), B (S2 solo si sigue parado) o C.
2. Gate dentro del driver o SROS2.
3. Las cuatro de la nota 20 (deadline, regla IMU, `base_footprint`, puerta).

## Siguientes acciones
1. Decidir e implementar la corrección de §4b; repetir matriz, regresiones F1 y 200 reinicios.
2. Revisar el NVMe de la NX.
3. No avanzar a Fase 3 hasta cerrar F2.

## Referencias
Nota `21-fase2-robustez.md`; plan `19-plan-cierre-y-entrega.md`; identidad y tablas `results/21-f2/`. NX: `/home/jetson/work/reloc-f2c-20261010/` (final), `reloc-f2-20261010/` (anteriores e inválidas, `f1ref/`). Raíces F1 intactas.
