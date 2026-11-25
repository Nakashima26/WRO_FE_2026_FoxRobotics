---
name: revisa-run
description: Revisa la(s) última(s) corrida(s) del carro en la Pi 5 — trae telemetría y video, resume resultado/INICIO/mapa/localizador/volante, saca cuadros del video donde algo se ve raro y reporta corto. Úsala cuando el usuario diga "revisa la run", "cómo le fue", "checa la última corrida", "/revisa-run [n|orillasNNN]".
---

# /revisa-run

Argumento opcional: número de corridas (por defecto 1) o un nombre `orillasNNN`.
"Última run" = la más nueva en la Pi, nunca la que tengas en contexto.

## 1. Traer (un solo ssh + scp)

```bash
S=<scratchpad>/runs; mkdir -p "$S"
ssh -o ConnectTimeout=15 raspi5 'ls -t ~/FoxRobotics/videos_orillas/*_tele.jsonl | head -<n>'
scp -q 'raspi5:FoxRobotics/videos_orillas/orillasNNN{_tele.jsonl,.avi}' "$S/"
```
Si ssh falla (timeout), dilo y pide que prendan la Pi; no reintentes en bucle.

## 2. Resumen

```bash
cd /c/001Escul/WRO2026/WRO2026/src/RASPI/cam/pure_pursuit/twin/tools
PYTHONIOENCODING=utf-8 python resumen_run.py "$S"/orillasNNN_tele.jsonl [--seg] | grep -v "par lateral"
```
Da: dur/tc/estado final (tc=12 y est=E = completa; `ATORADO` = choque), rumbo del tramo
recto de INICIO (salida adentro debe quedar ~85-95°), eventos de mapa/plan,
LOCUPD con |dx| o |dy| ≥ 60 mm, y ventanas con el volante cambiando de lado ≥3 veces/s.
Usa `--seg` y `awk '$1>a && $1<b'` para el detalle del tramo sospechoso.
Compara contra 1-2 corridas anteriores con el mismo acomodo (mismas `señales=`).

## 3. Cuadros del video (solo de lo sospechoso)

```bash
python cuadros_run.py "$S"/orillasNNN.avi "$S"/orillasNNN_tele.jsonl 18.4:20.9 4:5 -o "$S"/hoja.jpg
```
Luego `Read` la hoja. Máximo 2 rangos por hoja (cada uno ≤9 cuadros).

## 4. Reporte (corto, sin narrar)

- Resultado en una línea (completa / chocó en tc=N, pose).
- Lo que el usuario preguntó primero, con la causa concreta (tiempo, sensor, valor).
- Si falta saber el acomodo real de conos para decidir, pregúntalo en vez de adivinar.
- Fix propuesto en una línea; aplicarlo según las reglas de memoria (deploy por git,
  reiniciar wro-runtime con stop + sleep 4 + start, no mover el carro).

## Notas

- Rectas CCW desde el cajón W: W → S → E → N. Esquinas tc 0..12.
- No correr el gemelo en la Pi (lento); en esta PC no hay c++ para el gemelo.
- `lv` en el ACK no es voltaje de batería.
