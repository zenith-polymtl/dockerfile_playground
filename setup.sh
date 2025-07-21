#!/usr/bin/env bash
set -euo pipefail

# 1) Détection de l’utilisateur « humain » si on passe par sudo
if [ "$EUID" -ne 0 ]; then
  if command -v sudo &>/dev/null; then
    SUDO="sudo"
    TARGET_USER="${SUDO_USER:-$USER}"
  else
    echo "⛔ Ce script nécessite sudo ou d’être root."
    exit 1
  fi
else
  SUDO=""
  TARGET_USER="$USER"
fi

# 2) Installation de Docker si nécessaire
if ! command -v docker &>/dev/null; then
  echo "⚙️  Docker non trouvé. Installation en cours…"
  curl -fsSL https://get.docker.com | $SUDO sh
  echo "✔ Docker installé : $(docker --version)"
  echo "➕ Ajout de '$TARGET_USER' au groupe docker…"
  $SUDO usermod -aG docker "$TARGET_USER"
  NEED_REFRESH_GROUP=true
else
  echo "✔ Docker déjà présent : $(docker --version)"
  NEED_REFRESH_GROUP=false
fi

# 3) Lancement de docker compose up --build
echo
if id -nG "$TARGET_USER" | grep -qw docker && [ "$NEED_REFRESH_GROUP" = false ]; then
  # l’utilisateur est déjà membre du groupe : appel direct
  echo "🚀 Lancement direct de 'docker compose up --build'…"
  docker compose up --build
else
  # on profite de sg pour cette invocation seule
  echo "🔄 Lancement de 'docker compose up --build' via sg docker pour la session courante…"
  sg docker -c "docker compose up --build"
  echo
  echo "ℹ️  Pour les prochaines sessions, déconnectez-vous/reconnectez-vous ou exécutez 'newgrp docker'."
fi
