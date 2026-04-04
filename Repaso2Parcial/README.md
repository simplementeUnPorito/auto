# Repaso 2do Parcial

Scripts MATLAB de repaso y simulación para preparar el segundo parcial. Cubre los temas de espacio de estados, estimadores y control óptimo (Labs 5–9).

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `ayudaDiosito.m` | Script principal de repaso — simulaciones integradas |
| `AyudaDios2.m` | Versión extendida / iteración siguiente |
| `sim_current_int.m` | Simulación del estimador de actualización con integrador |
| `sim_pred_int.m` | Simulación del estimador de predicción con integrador |
| `plot_comparacion_estimadores.m` | Comparación gráfica predictor vs. actualizador |
| `XD.m` | Script misceláneo de pruebas rápidas |
| `*.asv` | Archivos de auto-guardado de MATLAB (ignorar) |

## Temas cubiertos

- Representación en espacio de estados (discreta)
- Ubicación de polos por Ackermann
- Estimador de Luenberger (predicción y actualización)
- Control con integrador (estado aumentado)
- LQR y LQI (`dlqr`)
- Filtro de Kalman (`dlqe`)
