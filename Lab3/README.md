# Lab 3 — Diseño por Respuesta en Frecuencia (Bode)

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar compensadores digitales utilizando el **diagrama de Bode** en tiempo discreto, especificando el sistema por sus márgenes de estabilidad y error en estado estacionario ante rampa.

---

## Contenido

```
Lab3/
├── document.tex           # Documento principal LaTeX
├── document.pdf           # Informe compilado
├── intro.tex
├── modelado.tex
├── desarrollo.tex
├── discretizacion.tex
├── implementacion.tex
├── resultados.tex
├── anexo.tex
├── modificacion.tex
├── Codigos/               # Código C para PSoC
└── img/                   # Figuras del informe
```

---

## Metodología

1. **Análisis de la planta en frecuencia** — diagrama de Bode de la planta discretizada
2. **Diseño del compensador** — adelanto/atraso de fase para cumplir margen de fase y ganancia
3. **Verificación de error ante rampa** — uso de constante de error de velocidad `Kv`
4. **Simulación en MATLAB** — validación con `bode()`, `margin()`, `step()`
5. **Implementación en PSoC** — ecuación en diferencias del compensador
6. **Comparación** simulación vs. experimento

> **Referencia teórica:** K. Ogata, *Sistemas de Control en Tiempo Discreto*, págs. 204–225.

---

## Resultados clave

- Compensador diseñado con margen de fase especificado
- Respuesta al escalón y a la rampa: simulación vs. experimento
- Análisis de sensibilidad ante variaciones del modelo
