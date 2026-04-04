# Progreso de documentación — Auto repo

Archivo de control para el loop de documentación automática.  
Última actualización: 2026-04-03

---

## Estado: COMPLETADO

Todas las carpetas relevantes del repositorio cuentan con README.md.

---

## READMEs creados

### Raíz y carpetas principales
- [x] `README.md` — descripción general, tabla de labs, estructura completa
- [x] `MatLab_Projects/README.md`
- [x] `PSoC_Projects/README.md`
- [x] `Repaso2Parcial/README.md`

### Labs 1–9
- [x] `Lab1/README.md` — PID
- [x] `Lab2/README.md` — Lugar de raíces
- [x] `Lab3/README.md` — Bode
- [x] `Lab4/README.md` — Truxal–Ragazzini / Dead-beat
- [x] `Lab5/README.md` — Ubicación de polos
- [x] `Lab6/README.md` — Estimadores
- [x] `Lab7/README.md` — Realimentación + integrador
- [x] `Lab8/README.md` — LQR / LQI
- [x] `Lab9/README.md` — Kalman / LQG

### TPF — Helicóptero Vertical
- [x] `TPF-HelicopteroVertical/README.md` — descripción completa del TPF
- [x] `TPF-HelicopteroVertical/Matlab/README.md` — todos los scripts y carpetas
- [x] `TPF-HelicopteroVertical/PSoC Creator/README.md` — historial de versiones firmware
- [x] `TPF-HelicopteroVertical/Arduino/README.md`
- [x] `TPF-HelicopteroVertical/3d models/README.md`
- [x] `TPF-HelicopteroVertical/Matlab/ProtocoloDeComunicacion/README.md` — (ya existía, completo)

---

## Carpetas sin README (intencional)

| Carpeta | Motivo |
|---------|--------|
| `third_party/` | Librerías externas (CMSIS), tienen sus propios READMEs |
| `Lab*/Codigos/`, `Lab*/c/`, `Lab*/matlab/` | Subcarpetas de código simples, documentadas en el README del lab padre |
| `TPF-HelicopteroVertical/Matlab/FaltaOrdenar/` | Carpeta temporal, se documenta en `Matlab/README.md` |
| `TPF-HelicopteroVertical/Planta/` | Ya tenía README propio |
| `TPF-HelicopteroVertical/Workspace03/` | PSoC workspace temporal de pruebas |

---

## Instrucción para el loop

**El repositorio está completamente documentado. El loop puede detenerse.**

Para cancelar el loop ejecutar:
```
CronDelete 4cee35d1
```
