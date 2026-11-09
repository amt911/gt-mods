# gt-mods

> Versión en español. Proyecto principal: [gran-turismo-decomp](https://github.com/amt911/gran-turismo-decomp) · mapa de progreso en línea: <https://claude.ai/artifact/V93Bh8ZWDTAUGfWaoZa5hq>

Mods nativos para la versión de PC de la decompilación de Gran Turismo (PSP),
[gran-turismo-decomp](https://github.com/amt911/gran-turismo-decomp). Cada mod es una biblioteca compartida más un
manifiesto `mod.toml`, que `gt_play` carga a través del SDK con ABI de C (`include/gt_mod/gt_mod_api.h`).

Aquí no hay nada derivado del juego: solo código de mods escrito a mano.

## Mods

- `analog-pedals` - gatillos analógicos / teclas del teclado asignados al acelerador y al freno (0..1). Desactivado por defecto.
  Ajustes en su `mod.toml` (`deadzone` en [0,1), `curve` > 0, `ramp_ms` >= 0; un valor inválido se registra en el log y se usa el
  valor por defecto). Mando: el gatillo derecho del primer mando de juego de SDL = acelerar, el izquierdo = frenar. Teclado: `throttle_keys`
  / `brake_keys` (nombres de tecla de SDL, por defecto `Up,W` / `Down,S`), con rampa durante `ramp_ms`. Los botones del propio mando siguen
  funcionando: gana la pulsación mayor, salvo mientras hay un gatillo pulsado: entonces decide el gatillo (gt_play también asigna un gatillo
  pulsado más allá de ~25 % a Cruz/Cuadrado, lo que de otro modo convertiría media pulsación en una completa). Funciona en carreras y en
  contrarreloj (Time Trial). El pedal sigue al gatillo exactamente (el mod escribe el valor del registro cuya respuesta en el juego es ese pedal),
  salvo por debajo de 0.05 (no se inyecta) y entre 0.05 y 0.1 (la pulsación más pequeña del juego es 0.1). Solo se toca el coche en vivo del jugador (no los coches de la IA, ni durante una repetición).
  `GT_MOD_PEDALS_FAKE=0.5,0.3` sustituye la entrada por un acelerador/freno fijos (lo usa la comprobación en el juego);
  `GT_MOD_PEDALS_LOG=1` registra en el log los pedales del jugador y de un coche de la IA una vez por segundo.
- `draw-distance` - aleja el plano de recorte lejano (*far clip plane*) del mundo de la carrera, de modo que el decorado lejano (edificios, colinas, gradas) se
  vea en lugar de aparecer de golpe. El juego no descarta geometría por ese plano (envía la misma geometría y la GPU recorta), así que un
  far mayor revela lo que ya se enviaba; el coste son píxeles de GPU, no CPU del juego. Post-hook en el constructor de profundidad `0x08AF4B4C` (llamado desde `0x0890E41C`
  con near = `self+4`, far = `self+0xC` = 1200 en las carreras vistas): los términos de la matriz m10/m14 se reescriben para `far * far_scale`
  (nunca se escriben datos de la escena; un hook en `0x0890E41C` que escale y restaure `self+0xC` no hace nada, porque `call_original`
  retorna antes de que se ejecute el resto de ese cuerpo). `far_scale` en `mod.toml`, por defecto `2.5`
  (1200 -> 3000), rango 1.0-6.0 (se limita al rango y se registra un aviso; los valores no numéricos mantienen el valor por defecto); 1.0 = fotogramas idénticos a no tener
  el mod (el mod no escribe nada). Valores mayores adelgazan el búfer de profundidad (unas pocas motas en la geometría lejana con 8000). El pase del cielo y
  el pase de reflejos del coche no usan esta función y no se tocan. Desactivado por defecto; se activa y desactiva en vivo desde el menú F9.
  `GT_MOD_DRAWDIST_SCALE=<n>` sustituye el ajuste (lo usa la comprobación en el juego, `tools/mod_drawdist_check.sh`);
  `GT_MOD_DRAWDIST_LOG=1` registra near/far cada 120 ediciones.
- `orbit-camera` - cámara orbital al estilo de Gran Turismo 7 en el stick derecho. La dirección del stick elige la dirección de la mirada alrededor del
  coche del jugador (derecha = +90 deg, izquierda = -90 deg, adelante = 0); tirando hacia atrás (más allá de `back_threshold`) o pulsándolo (R3) = vista trasera
  (180 deg). La cámara gira alrededor del coche en las vistas de persecución y mira alrededor desde el asiento en las vistas de parachoques/cabina (el pivote es
  el coche, que ahí está ~a la altura del ojo). El stick gira en sentido contrario en todas las vistas: `invert_outside_chase` (parachoques,
  cabina, persecución cercana; por defecto true, como GT7) e `invert_in_chase` (la persecución lejana, tercera persona completa; por defecto true; se
  reconoce por la distancia del coche al ojo, > 6.5); sin inclinación desde el stick (`pitch_deg`, por defecto 0, añade una inclinación constante). Suavizado con una
  constante de tiempo `smooth_ms` (80) y vuelve a 0 al soltar; zona muerta `deadzone` (0.25). Teclado: `look_left_keys` /
  `look_right_keys` / `look_back_keys` (nombres de tecla de SDL, por defecto `J` / `L` / `K`, libres en los atajos por defecto de gt_play). Solo mientras se
  conduce (coche del jugador en vivo, no una repetición, no un menú); sin entrada no escribe nada. Desactivado por defecto; se activa y desactiva en vivo desde el
  menú F9. `GT_MOD_CAMERA_FAKE=<yaw>[,<pitch>]` fija el ángulo (lo usa la comprobación en el juego, `tools/mod_camera_check.sh`);
  `GT_MOD_CAMERA_LOG=1` registra yaw, escena y pivote una vez por segundo. Al tirar del stick hacia atrás el ángulo se mezcla suavemente con la vista
  trasera (180 completos solo con el stick del todo atrás). La órbita está activa mientras sigan llegando pasos de física (un hueco de 10 fotogramas, contados por las
  subidas del HUD, la termina). La órbita se aplica a la vista de la cámara cuando el renderizador de la escena la carga para cada capa (0x08AF1524 ->
  0x08B568DC), así la cúpula del cielo y las capas lejanas se colocan y se descartan según el rumbo orbitado (girar solo las matrices de mundo de los
  objetos dejaba el cielo sin dibujar detrás del coche, y el juego nunca borra el búfer de color en carrera: los fotogramas anteriores se veían a través
  como un efecto de espejos); la cámara del retrovisor no se toca. Límites conocidos: en las vistas de parachoques/cabina el pivote es el coche, que no está exactamente a la altura del ojo (la cámara
  gira un poco en lugar de girar sobre sí misma); el mod abre su propio manejador de mando SDL junto al del host; las teclas de mirada
  (J/K/L) también actúan mientras el menú de mods F9 está abierto (el SDK no puede saber si el menú está abierto).
- `reset-to-track` - un botón devuelve el coche del jugador a la carretera, derecho y mirando hacia donde iba, como "recuperar coche" en los
  juegos de carreras modernos. Botón: Select del mando mantenido 0.5 s (`hold_ms`), o la tecla `reset_keys` (por defecto `R`). Mientras se conduce guarda un
  anillo de poses "buenas" (las cuatro ruedas sobre asfalto, vector arriba z > 0.9, más rápido que 5 m/s, velocidad de guiñada <= 0.6 rad/s, velocidad dentro de ~25 deg de
  la dirección; una cada 15 pasos); el reinicio restaura la pose más reciente con al menos `rewind_ms` (2500) de antigüedad, pone a cero velocidad, rapidez y todas las
  velocidades de rotación (el coche arranca desde parado) e ignora nuevas pulsaciones durante `cooldown_ms` (1000). No ocurre nada si todavía no
  existe ninguna pose buena. El anillo se vacía cuando el coche salta más de 15 m en un paso (una reaparición, un nuevo intento), tras 2 s sin pasos (el mod
  se desactivó en F9, una pausa larga) y con un coche nuevo; las poses de más de 8 s nunca se restauran. Permitido en todas partes (incluida la
  contrarreloj); solo el coche en vivo del jugador, nunca una repetición. Desactivado por defecto; se activa y desactiva en vivo desde el menú F9.
  **Select y la cámara:** el juego usa Select para cambiar la vista, así que durante la carrera el mod oculta al juego un Select mantenido; una pulsación
  más corta que `hold_ms` se reproduce como un toque al soltar (la vista de cámara sí cambia), una más larga reinicia y la vista no cambia.
  Fuera de una carrera (menús, pausa) Select no se toca. **Límites conocidos:** la marcha, las rpm y los campos de posición previa no se restauran (no están
  localizados; la caja de cambios automática vuelve a arrancar bien desde parado, la comprobación en el juego exige > 15 m/s con el acelerador pulsado tras un reinicio);
  el contador de vueltas y el cronómetro no se rebobinan (medido en un reinicio a mitad de vuelta: HUD 0'24.000 -> 0'27.000 en 3 s de juego, vuelta 1/2 sin cambios), y un reinicio que cruce la línea de salida/meta no se probó (ninguna ruta completa una vuelta) y puede
  contar la vuelta dos veces; la grabación de un fantasma / repetición de contrarreloj mostrará el salto (sin probar); no se usa geometría del circuito, el coche vuelve
  donde estaba, no al asfalto más cercano. `GT_MOD_RESET_FAKE=<step>` mantiene el botón desde ese paso del coche del jugador (una ayuda de depuración); `GT_MOD_RESET_LOG=1` registra el estado del coche (lo usa `tools/mod_reset_check.sh`).
- `cheats` - trucos para carrera como filas del menú F9 (teclado o mando): cámara lenta (0.5x / 0.25x, coches del jugador y de la IA) y congelar la IA. Necesita la API de mods v2
  (`api = 2`). Desactivado por defecto y ambos trucos empiezan desactivados. Véase `cheats/README.md`; comprobación: `tools/mod_cheats_check.sh` en el repositorio del host.
- `rollover` - permite que el coche del jugador vuelque, algo que el juego original no puede (limita la inclinación a 35 grados). Engancha el lector de la bandera
  `0x0894CCB8` (un interruptor dormido) para levantar el límite y añade una velocidad de balanceo ante un impacto lateral fuerte (un cambio de velocidad lateral superior a `threshold`
  en un solo paso de física; `gain`, `cap`, `style = realistic|arcade` en `mod.toml`); la máquina de estados de vuelco dormida que la bandera también despertaría
  (el coche se hunde a través de la carretera y vuelve a salir) se mantiene dormida. Opt-in: `enabled = false` por defecto; se activa y desactiva en vivo desde el menú F9.
  Véase `rollover/README.md`. Comprobación en el juego: `tools/mod_rollover_check.sh`.

## Compilación

Necesita una copia (*checkout*) de `gran-turismo-decomp` para la cabecera del SDK (por defecto `../gran-turismo-decomp/include`):

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++ [-DGT_SDK_DIR=/path/to/gran-turismo-decomp/include]
cmake --build build -j4
ctest --test-dir build
```

## Instalación

Copia el directorio del mod (`analog-pedals/` con su `mod.toml` y la `libanalog_pedals.so` compilada, de
`build/analog-pedals/`) en el directorio de mods del juego (`<build>/mods/` junto a `gt_play`, o `GT_MODS_DIR`) y pon
`enabled = true` en `mod.toml`. `gt_play` imprime `[mods] loaded: ...` al arrancar. Ejecuta la comprobación en el juego desde la copia de
gran-turismo-decomp: `tools/mod_pedals_check.sh` / `tools/mod_camera_check.sh` / `tools/mod_reset_check.sh` / `tools/mod_rollover_check.sh`.

## Publicaciones (*releases*)

Una publicación es una etiqueta de git; la CI (`.github/workflows/release.yml`) compila todos los mods contra el `include/` fijado de `gran-turismo-decomp`
(SHA del commit en el `SDK_REF` del workflow), ejecuta los tests de las herramientas, empaqueta los archivos y los sube junto con `index.json`:

```bash
git tag vYYYY.MM.DD        # a second release the same day: vYYYY.MM.DD.1
git push origin vYYYY.MM.DD
```

Ejecuta el workflow a mano (Actions -> release -> Run workflow) para una prueba en seco: compila y empaqueta pero no sube nada
(los archivos quedan como el artefacto de workflow `release-dry-run`). Sube `version` en el `mod.toml` de un mod (`x.y.z`) cuando el mod cambie;
el índice lista lo que diga el commit etiquetado.

En local, los mismos archivos (necesita una compilación en `build/`):

```bash
python3 tools/make_index.py --build-dir build --platform linux-x86_64 \
    --base-url https://github.com/amt911/gt-mods/releases/download/vYYYY.MM.DD --out-dir dist
python3 -m pytest tests --cov=tools --cov-fail-under=80
```

La salida es determinista (entradas de tar ordenadas, mtime/uid/gid fijos, mtime de gzip 0): las mismas entradas dan archivos idénticos byte a byte.
Cada mod se convierte en `<id>-<version>-<platform>.tar.gz` con una carpeta `<id>/` que contiene `mod.toml` y la biblioteca; descomprímelo en el
directorio de mods del juego. Las descripciones de una línea de cada mod están en `tools/descriptions.toml` (el lector de `mod.toml` del host rechaza las claves desconocidas).

### `index.json`

```json
{
  "schema": 1,
  "mods": [
    {
      "id": "analog-pedals",
      "name": "Analog pedals",
      "version": "0.1.0",
      "api": 2,
      "description": "Analog triggers and keyboard keys mapped to throttle and brake (0..1).",
      "platforms": {
        "linux-x86_64": {
          "url": "https://github.com/amt911/gt-mods/releases/download/v2026.11.08/analog-pedals-0.1.0-linux-x86_64.tar.gz",
          "sha256": "<hex sha256 of the archive>",
          "size": 29508
        }
      }
    }
  ]
}
```

## Licencia

MIT, véase `LICENSE`.
