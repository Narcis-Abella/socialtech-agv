# SOCIALTECH — ESTADO ACTUAL

Fecha: 2026-10-11 · Rama: `feat/global-reloc` (commits locales, sin push) · F1 congelado `GR-S2S3-F1` = `c026a7b`, intacto · Candidato F2 `GR-S2S3-F2b` = método `5bb95ef` + `f524d04`, gate `8a88d16`.

## Fase actual
Fase 1 **VERIFIED**. Fase 2 **VERIFIED — VALIDATED_IN_REPLAY** (nota 21, parte B). Fase 3 no iniciada. Sin validación física; seguridad completa del AGV no demostrada; F4-F6 pendientes. `can0` sin uso.

## Último resultado (nota 21)
- **h27 cerrado:** la pose se arrastraba ~13 s por odometría sin comprobar. Ahora se comprueba contra el mapa con los 3 s de escaneos más recientes antes de `/initialpose`, antes de `LOCALIZED` y cada 3 s después. h27 5/5 sin publicar.
- **Bypass del gate cerrado en vcan:** driver en celda de red aislada, gate por socket privado, relleno de ceros y vigilante CAN (solo `vcan`). Si muere gate, fuente o driver: ceros en ≤0,4 s.
- Matriz: 56 casos válidos, 51 conformes, 4 predicciones mías erróneas con comportamiento correcto, 1 fallo `O1` no reproducido (h35, proceso sin recoger al apagar). 0 poses erróneas, 0 permisos sobre pose incorrecta al concederse.
- 14/14 negativos rechazados; S2 y S3 correctos.
- **50 reinicios (protocolo revisado desde 200):** 41 `LOCALIZED` correctos, 9 rechazos seguros, 0 aceptación errónea (cota Wilson 7,1 %).
- Coste: `LOCALIZED` ~4 s más tarde (mediana 27,8 s), +16 MB, +10 % de un núcleo.

## Problemas principales
- Tras un fallo de odometría posterior a `LOCALIZED`, el permiso sobrevive 1,6-2,3 s a la pose correcta (cota ~4 s). No hay requisito definido.
- 2 de 50 rechazos nuevos: si la plataforma se mueve >~0,5 m/s en la entrega, AMCL arranca retrasado y no confirma.
- NVMe de la NX: 131 timeouts de kernel desde el 5-oct, causa sin verificar; SMART requiere root.
- Timeout de comandos del firmware de la base sin verificar; CAN físico en la celda sin hacer.

## Decisiones pendientes de Narcís
1. Latencia de revocación aceptable (¿odometría de ruedas?).
2. Despliegue de la celda: `can0` en su espacio de red, vigilante CAN en bus real.
3. NVMe: `sudo nvme smart-log /dev/nvme0`, sustitución o ajuste.
4. Las cuatro de la nota 20.

## Siguientes acciones
1. Revisión de Narcís de la nota 21 (B7, B10, B13).
2. Fase 3 (rendimiento y timeout) solo tras su visto bueno.

## Referencias
Nota `21-fase2-robustez.md`; plan `19-plan-cierre-y-entrega.md`; identidad `results/21-f2/f2h/candidate-identity.txt`. NX: `/home/jetson/work/reloc-f2h-20261011/` (final), `reloc-f2g-20261011/` (árbol intermedio y pruebas de celda), `reloc-f2c-20261010/` (anterior).
