# Lab 5 — Ubicación Arbitraria de Polos (Espacio de Estados)

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar un controlador digital por **realimentación de estados** usando la técnica de **ubicación arbitraria de polos** (pole placement). Se parte de la representación en espacio de estados de la planta discretizada y se calcula la ganancia de realimentación `K` para ubicar los polos de lazo cerrado en posiciones deseadas.

---

## Contenido

```
Lab5/
├── lab5Auto_EAyTR.tex     # Documento principal LaTeX
├── lab5Auto_EAyTR.pdf     # Informe compilado
├── intro.tex
├── desarrollo.tex
├── Implementacion.tex
├── resultados.tex
├── anexo.tex
├── matlab/                # Scripts MATLAB de diseño
├── c/                     # Código C para PSoC
├── exp/                   # Datos experimentales
└── Otros/                 # Material adicional
```

---

## Metodología

1. **Modelo en espacio de estados** — obtención de matrices `F`, `G`, `H`, `J` de la planta
2. **Discretización** — `Φ = e^(F·Ts)`, `Γ`, `Ts = 1 ms`
3. **Polos deseados** — especificaciones en `ts` y `ζ`, mapeados al plano-z
4. **Cálculo de K** — fórmula de Ackermann (`place()` / `acker()` en MATLAB)
5. **Pre-filtro N** — para seguimiento de referencia con ganancia estacionaria unitaria
6. **Simulación en MATLAB** — validación de la respuesta
7. **Implementación en PSoC** — actualización de estados y cálculo de `u = -K·x + N·r`
8. **Comparación** simulación vs. experimento

> **Nota:** se asume **estados completamente medibles** (sin estimador). El estimador se introduce en Lab 6.

---

## Resultados clave

- Ganancia `K` calculada para distintas especificaciones de polos
- Respuesta al escalón: simulación vs. experimento
- Sensibilidad ante errores de modelado (todos los estados son medidos directamente)
