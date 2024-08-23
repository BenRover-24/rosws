#!/bin/bash

TARGET_FILE="install/benrover_assembly/share/benrover_assembly/launch/launch_simulation.py"
SOURCE_FILE="launch/launch_simulation.py"

if [ ! -L "$TARGET_FILE" ]; then
  echo "Copie du fichier launch..."
  install -D "$SOURCE_FILE" "$TARGET_FILE"
else
  echo "Lien symbolique existant, pas besoin de copier."
fi