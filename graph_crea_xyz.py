import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
"""import numpy as np
from np import core
from core import multiarray"""


# 🔍 Trouver le dernier fichier CSV du dossier "data"
# def get_latest_csv(folder_path="dockerfile_playground/ros2_ws/data"):

def get_latest_csv(folder_path="ros2_ws/data"):
    list_of_files = glob.glob(os.path.join(folder_path, "*.csv"))
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file

# 📦 Charger les données
# csv_path = get_latest_csv("data")
csv_path = get_latest_csv()
df = pd.read_csv(csv_path)
csv_filename = os.path.basename(csv_path)
timestamp = os.path.getmtime(csv_path)
dt_string = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S")


# 🕰️ Option de filtrage temporel
start_time = float(input("⏱️ Entrez le temps de départ (ex: 10.0) : "))
end_time = float(input("⏱️ Entrez le temps de fin (ex: 42.0) : "))
#start_time = 10
#end_time = 42

# 🧼 Filtrer le DataFrame selon le temps
df_filtered = df[(df["time"] >= start_time) & (df["time"] <= end_time)]

# 🎨 Créer les graphiques
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
axes_titles = ["x", "y", "z"]
pos_cols = ["pos_x", "pos_y", "pos_z"]
tar_cols = ["tar_x", "tar_y", "tar_z"]

for i in range(3):
    axes[i].plot(df_filtered["time"], df_filtered[pos_cols[i]], label=f"Position {axes_titles[i]}", color="blue")
    axes[i].plot(df_filtered["time"], df_filtered[tar_cols[i]], label=f"Cible {axes_titles[i]}", color="red", linestyle="--")
    axes[i].set_ylabel(f"{axes_titles[i]}")
    axes[i].legend()
    axes[i].grid(True)

axes[2].set_xlabel("Temps (s)")
fig.suptitle("Comparaison Position vs Cible dans les axes x, y, z")

plt.tight_layout()
# plt.show()
# plt.savefig(f"dockerfile_playground/ros2_ws/graph/graph_xyz_{dt_string}.png")
# print(f"✅ Graphique sauvegardé : graph_xyz_{dt_string}.png")




# 💡 Affichage (optionnel dans un script non interactif)
# plt.show()  # Tu peux commenter cette ligne si tu es en mode script

# 🔧 Créer le dossier de sauvegarde
os.makedirs("ros2_ws/graph", exist_ok=True)

# 💾 Sauvegarder le graphique
plt.savefig(f"ros2_ws/graph/graph_xyz_{dt_string}.png")
print(f"✅ Graphique sauvegardé : graph_xyz_{dt_string}.png")