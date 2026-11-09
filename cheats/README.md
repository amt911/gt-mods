# cheats

Trucos para carrera de Gran Turismo PSP, como filas del menú de mods F9 (teclado o mando): **cámara lenta** (0.5x / 0.25x, el coche del jugador y los coches de la IA)
y **congelar la IA** (los coches de la IA se saltan su paso de física). Opt-in doble: el mod se entrega con `enabled = false`, y una vez activado ambos trucos empiezan desactivados.
Necesita la API de mods v2 (`api = 2`: `add_menu_row`); el `docs/MODS.md` del repositorio del host describe la API y el menú del mando (combinación SELECT+START).

## Uso

F9 (o la combinación del mando SELECT+START) abre la página MODS: tras las filas propias de los mods aparecen `Slow motion  OFF|0.5x|0.25x` y `Freeze AI  OFF|ON`.
Derecha / Enter avanzan, Izquierda retrocede (mando: cruceta derecha / cruz, cruceta izquierda). Ambos solo actúan en una carrera en vivo (exactamente un coche humano, nunca una repetición);
fuera de una no se toca nada. Desactivar el mod en F9 lo detiene todo al instante (el host pasa el hook directamente al juego).

## Cómo funciona

`0x0894DD9C(world)` avanza cada coche una vez por paso de física. Con `--decomp off` la unidad recompilada ejecuta el cuerpo de esa función **después** de que el hook
retorne, así que ahí un hook no puede leer "el estado tras el paso". El mod trabaja en la siguiente llamada: a su entrada el estado de cada coche es el resultado
completo del paso anterior, y el mod reescribe `base + f * (state - base)` sobre los campos cinemáticos (posición, filas de la base, rapidez, velocidad, velocidades angulares,
ángulos de Euler), donde `base` es lo que dejó tras la llamada anterior. El método de Euler explícito es lineal en dt, así que esto es la misma física a una fracción del reloj:
las velocidades también cambian `f` veces más despacio (un simple multiplicador de velocidad frenaría el coche en su lugar). `f` = 0.5 / 0.25 para la cámara lenta, 0 para un coche
de la IA mientras la IA está congelada (conserva su rapidez y sigue desde ella cuando desactivas el truco). Con `f = 1` no se escribe nada: con el mod activado e inactivo
se obtienen fotogramas idénticos byte a byte a no tener el mod (la comprobación en el juego lo demuestra). Los ángulos de Euler toman el camino corto en su salto de +-pi, las filas de la base
se reortonormalizan, un coche que se movió más de 8 m entre dos llamadas (reset-to-track, una reaparición) se deja en paz, y las bases se reinician tras 8 vblanks
sin llamada (el contador del propio juego, `vblank_index`).

Por qué no saltarse pasos enteros (que un paso del mundo de cada dos no se ejecute): el juego avanza a ~30 Hz, así que 0.25x dibujaría 7 pasos por segundo. Por qué no escalar
un paso de tiempo: el paso del mundo no recibe dt (su llamador pasa uno, `0x0894DD9C` lo ignora: paso fijo).

## Límites

- **No se ralentizan:** los relojes del juego (cronómetro de la carrera, contador de vueltas, cuenta atrás), el estado propio de los pilotos de la IA (el piloto de un coche de la IA congelado sigue funcionando contra un coche
  que no se mueve), el estado de neumáticos y motor fuera del cuerpo (giro de ruedas, rpm), el sonido y la cámara. Las colisiones durante la cámara lenta se resuelven en más pasos.
- La imagen dibujada entre dos llamadas muestra cada coche un paso completo por delante de la base del mod: una ventaja constante, así que el movimiento en pantalla es `f` veces más rápido.
- Un fantasma de contrarreloj puede tratarse como un coche de la IA (ralentizado o congelado); sin probar, igual que
   dos jugadores humanos (el mod entonces no hace nada) y las repeticiones (ignoradas a propósito).

## Ajustes (`mod.toml`)

| clave | por defecto | significado |
| --- | --- | --- |
| `slow_motion` | `"off"` | estado inicial: `off`, `0.5` o `0.25` |
| `freeze_ai` | `false` | estado inicial |

Ayudas de depuración que usa la comprobación en el juego (`tools/mod_cheats_check.sh` en el repositorio del host): `GT_MOD_CHEATS_SLOWMO=<off|0.5|0.25>`, `GT_MOD_CHEATS_FREEZE_AI=1`,
`GT_MOD_CHEATS_FROM=<player-car step>`, `GT_MOD_CHEATS_LOG=1` (+ `GT_MOD_CHEATS_LOG_EVERY=<steps>`): `[mod:cheats] st=<step> slow=.. freeze=.. P=<player pos> spd acc A<i>=<AI pos>`.
