# rollover

Permite que el coche del jugador vuelque en Gran Turismo PSP. El juego original no puede: limita los ángulos de inclinación del coche a +-35 grados y pone a cero sus
velocidades. Este mod levanta el límite y, ante un **impacto lateral fuerte**, hace que el coche ruede. Desactivado por defecto (`enabled = false`); se activa y desactiva
en vivo desde el menú F9. Combina con `reset-to-track` (un botón devuelve a la carretera un coche que ha acabado sobre el techo).

## Qué hace

1. **El límite.** `0x0893E0FC` limita los ángulos de inclinación (`K+0x6FC`, `K+0x700`) salvo que la palabra de `0x08CF2F08` sea distinta de cero; `0x0894CCB8` lee
   esa palabra (su escritor no tiene ningún llamador en el binario). El mod engancha ese lector; no se escribe nada en el juego, y cuando el mod se
   desactiva el host deja de llamar al hook, así que el juego vuelve a leer su palabra real (0).
2. **La bandera se levanta solo donde el balanceo la necesita.** El lector tiene seis llamadores, todos `jal`. El mod devuelve 1 solo a los lectores del interior del
   paso de movimiento del chasis `0x08956DC8` que el balanceo necesita (el límite de inclinación y el ajuste del ángulo de Euler) y la palabra real (0) a los demás: el
   reinicio de la normal de contacto `0x08956EF4`, `0x08945F6C` (dentro de ese paso, pero levantarla cambia fotogramas con `--decomp on`), el límite contra el suelo del paso
   y, sobre todo, `0x08956930`. Con la bandera activada, esta última
   despierta una máquina de estados dormida de "coche volcado" que se salta el pipeline de fuerzas del coche mientras yace de lado: el coche se hunde a través de
   la carretera y vuelve a salir empujado ~6 m (medido, `docs/FACTS.md` del repositorio del host, "Rollovers"). Una primera versión levantaba la bandera para todos los lectores y
   cambiaba unos pocos píxeles de un coche asentándose en la parrilla; con la bandera acotada, una ejecución con el mod activado y sin golpes es idéntica fotograma a fotograma a una
   sin él (la comprobación en el juego lo demuestra, con `--decomp off` y `on`).
3. **El impulso.** Un hook previo al paso en `0x0894DD9C` (solo el coche del jugador, nunca una repetición) compara la velocidad en el mundo del coche con la del paso anterior. Un cambio a lo largo
   del eje lateral del coche (fila `K+0x28`) superior a `threshold` en UN solo paso es un impacto lateral fuerte: se hace rodar al coche hacia el lado golpeado
   (`K+0x6F4`, la velocidad de balanceo) a `gain` x (cambio - threshold) rad/s, como máximo `cap`, mantenida durante 40 pasos con un decaimiento lineal (un empujón de un solo paso
   lo deshacen los muelles). La conducción normal cambia la velocidad lateral unos ~0.2 m/s por paso: no ocurre nada. Un salto del coche (> 3 m en
   un paso: reset-to-track, una reaparición) o de su velocidad (> 60 m/s) no es un impacto y descarta lo que se estaba manteniendo.

## Ajustes (`mod.toml`)

| clave | por defecto | significado |
| --- | --- | --- |
| `style` | `"realistic"` | `realistic`: solo los impactos grandes hacen rodar el coche. `arcade`: el umbral se reduce a la mitad y la ganancia se multiplica por 1.5, así que golpes menores lo hacen rodar |
| `threshold` | `3.0` | cambio de velocidad lateral en un paso (m/s) por encima del cual un golpe vuelca el coche (0.1 - 50) |
| `gain` | `2.5` | velocidad de balanceo (rad/s) por cada m/s por encima del umbral (0 - 50) |
| `cap` | `12.0` | mayor velocidad de balanceo (rad/s) que pide el mod (0 - 30) |

Un valor inválido se registra en el log y se usa el valor por defecto. Puntos de referencia (medidos en la ruta de mando `rollover`): la barrera al final de la
recta a ~38 m/s da ~8.3 m/s de cambio lateral (velocidad 12, una voltereta más allá de boca abajo); el coche aterriza, los muelles lo recuperan y
sigue conduciendo.

## Límites conocidos

- En el juego no hay colisión del techo: un coche que rueda conserva el modelo de contacto de sus ruedas y siempre se recupera; no se queda sobre el techo.
- El límite se levanta para todos los coches: un coche de la IA que se incline más de 35 grados ya no se retiene. Los coches de la IA no reciben impulsos y no se vio que
  ninguno se inclinara tanto en las pruebas, pero no está demostrado para todos los circuitos.
- Solo el jugador recibe impulsos, y el jugador se identifica como exactamente un humano (vtable del piloto `0x08CC1F60`, no una repetición, con entrada). Con dos humanos
  (ad hoc / pantalla dividida) nadie recibe un impulso mientras el límite sigue levantado.
- El balanceo es una velocidad de balanceo mantenida, no una simulación física del impacto; los 40 pasos de empuje, la ganancia y el tope se ajustaron con un solo golpe contra una barrera.
- `GT_MOD_ROLLOVER_FAKE=<step>:<dv>` añade `<dv>` m/s de cambio lateral en la vista del rastreador en ese paso del coche del jugador (lo usa `mods_together_check`);
  `GT_MOD_ROLLOVER_LOG=1` registra el estado del coche (lo usa `tools/mod_rollover_check.sh`).

## Tests

`ctest --test-dir build` ejecuta `rollover_test` (ajustes, umbral, tope, estilo, signos, decaimiento, protecciones ante teletransportes). La comprobación en el juego está en el
repositorio decomp: `tools/mod_rollover_check.sh` (véase `docs/MODS.md` allí).
