# TPF — Arduino

Código Arduino para pruebas de sensores y monitoreo del sistema.

## Proyectos

### `PruebasSensoresAuto/PruebasSensoresAuto.ino`
Pruebas iniciales de los sensores del helicóptero vertical. Permite leer y verificar el sensor TFmini Plus y otros periféricos antes de integrarlos al firmware PSoC.

---

## Notas

- El Arduino **no** corre el controlador — ese rol lo tiene el PSoC 5LP.
- Uso típico: monitoreo externo, visualización de datos, verificación de sensores durante puesta a punto.
- Compilar con **Arduino IDE** (o compatible).
