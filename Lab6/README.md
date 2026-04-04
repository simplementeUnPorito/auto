# Lab 6 — Estimadores de Estado

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar controladores digitales para la planta analógica utilizando **estados estimados** en lugar de estados medidos directamente. Se implementan y comparan dos tipos de observador de Luenberger:

- **Estimador de predicción** (`x̂(k+1|k)`) — el estimado se calcula antes de conocer la medición actual
- **Estimador de actualización** (`x̂(k|k)`) — el estimado se corrige con la medición actual

---

## Contenido

```
Lab6/
├── document.tex           # Documento principal LaTeX
├── document.pdf           # Informe compilado
├── intro.tex
├── desarrollo.tex
├── implementacion.tex
├── resultados.tex
├── anexo.tex
├── matlab/                # Scripts MATLAB de diseño
├── c/                     # Código C para PSoC
└── img/                   # Figuras del informe
```

---

## Metodología

1. **Diseño del controlador base** — ganancia `K` por ubicación de polos (Lab 5)
2. **Diseño del estimador** — ganancia `L` por ubicación de polos del observador
   - Polos del observador ~5× más rápidos que los polos de lazo cerrado
3. **Estimador de predicción** — ecuaciones de actualización en `k+1`
4. **Estimador de actualización** — corrección con medición en `k`
5. **Principio de separación** — verificación de que controlador y estimador se diseñan independientemente
6. **Simulación en MATLAB** — comparación de ambos estimadores
7. **Implementación en PSoC** — bucle de control con estimación de estados
8. **Comparación** simulación vs. experimento, predicción vs. actualización

---

## Resultados clave

- Convergencia del estimador ante condiciones iniciales desconocidas
- Comparación predicción vs. actualización: velocidad y precisión
- Respuesta del sistema de lazo cerrado con estados estimados
