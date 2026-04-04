# Lab 2 — Lugar de Raíces en Tiempo Discreto

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar un controlador digital mediante el **método del lugar de raíces en el plano-z**, partiendo de la discretización de la planta analógica por ZOH. El diseño debe cumplir especificaciones de respuesta transitoria y garantizar error nulo ante escalón.

Especificaciones:
- Estabilidad: todos los polos dentro del círculo unitario
- Factor de amortiguamiento: `ζ = 0.7`
- Tiempo de subida: 8 muestras en `tr`
- Error en estado estacionario: `ESS = 0`

---

## Contenido

```
Lab2/
├── document.tex           # Documento principal LaTeX (informe)
├── document.pdf           # Informe compilado
├── intro.tex
├── modelado.tex
├── desarrollo.tex
├── resultados.tex
├── Codigos/               # Código C para PSoC
├── Img/                   # Figuras del informe
└── referencias.bib        # (si aplica)
```

---

## Metodología

1. **Discretización de la planta** — ZOH con tiempo de muestreo seleccionado
2. **Determinación de polos deseados** — mapeo de especificaciones continuas al plano-z
3. **Diseño por lugar de raíces** — se ajusta el compensador para que los polos deseados pertenezcan al lugar
4. **Simulación en MATLAB** — validación con `rlocus()`, `zgrid()`, `step()`
5. **Implementación en PSoC** — ecuación en diferencias del compensador
6. **Comparación** MATLAB vs. experimento

---

## Resultados clave

- Polos de lazo cerrado ubicados según especificaciones en plano-z
- Respuesta al escalón simulada vs. experimental
- Análisis de discrepancias entre modelo y planta real
