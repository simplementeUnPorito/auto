# Lab 1 — Controlador PID

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar e implementar un controlador **PID (Proporcional–Integral–Derivativo)** digital sobre la planta analógica de laboratorio, analizando la influencia de cada parámetro (Kp, Ki, Kd) en la respuesta transitoria y en el esfuerzo de control.

Se busca lograr:
- Estabilidad en lazo cerrado
- Error en estado estacionario igual a cero ante referencia escalón
- Respuesta transitoria aceptable (sobreimpulso, tiempo de establecimiento)

---

## Contenido

```
Lab1/
├── document.tex           # Documento principal LaTeX (informe)
├── introduccion.tex
├── modelado.tex
├── desarrollo.tex
├── resultados.tex
├── windup.tex             # Anti-windup del integrador
├── zn_step_method.tex     # Método de Ziegler–Nichols por escalón
├── diseno_pid.m           # Script MATLAB de diseño
├── Codigos/               # Código C para PSoC
├── img/                   # Figuras del informe
└── referencias.bib
```

---

## Metodología

1. **Modelado de la planta** — identificación del modelo analógico y su discretización
2. **Sintonía PID** — método de Ziegler–Nichols por respuesta al escalón
3. **Simulación en MATLAB** — validación del diseño con `pid()` o manualmente
4. **Implementación en PSoC** — ecuación en diferencias (forma posicional o incremental)
5. **Anti-windup** — saturación del integrador para evitar desbordamiento

---

## Resultados clave

- Comparación de respuestas con distintos valores de Kp, Ki, Kd
- Efecto del anti-windup en la respuesta ante saturación del actuador
- Diferencias entre simulación (MATLAB) y experimento (PSoC)
