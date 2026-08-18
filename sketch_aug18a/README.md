# Arduino: missing `forward_control.h`

Put **both** files in the same sketch folder:

- `sketch_aug18a.ino` (your main sketch)
- `forward_control.h` (this folder)

Arduino IDE only finds `#include "forward_control.h"` if the header is next to the `.ino`.

## Quick fix on your PC

1. In the sketch folder  
   `C:\Users\Acer\AppData\Local\Temp\.arduinoIDE-unsaved...\sketch_aug18a\`
2. Create a new file named exactly `forward_control.h`
3. Paste the contents of `forward_control.h` from this repo
4. Save, then Compile again

Better: use **File → Save As** to a permanent folder (not Temp), and keep `.ino` + `.h` together there.
