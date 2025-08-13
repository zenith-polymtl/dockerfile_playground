import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# 🔍 Trouver le dernier fichier CSV du dossier "data"
def get_latest_csv(folder_path="ros2_ws/data"):
    list_of_files = glob.glob(os.path.join(folder_path, "*.csv"))
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file

# 📦 Charger les données
csv_path = get_latest_csv() # Choisir entre les deux lignes selon le csv à analyser voulu
#csv_path = "ros2_ws/graph/pos_xyz_20250803_212020.csv"
df = pd.read_csv(csv_path)
csv_filename = os.path.basename(csv_path)
timestamp = os.path.getmtime(csv_path)
dt_string = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S")


# 🕰️ Option de filtrage temporel
start_time = float(input("⏱️ Entrez le temps de départ (ex: 10.0) : "))
end_time = float(input("⏱️ Entrez le temps de fin (ex: 42.0) : "))
axis_wanted = input("Entrez les axes à analyser (ex: x ou ou yz ou ENTER pour xyz) : ") or "xyz"

# 🧼 Filtrer le DataFrame selon le temps
df_filtered = df[(df["time"] >= start_time) & (df["time"] <= end_time)]

# 🎨 Créer les graphiques
nbr_graph = len(axis_wanted)
fig, axes = plt.subplots(nrows=int(nbr_graph), ncols=1, figsize=(10, 8), sharex=True)
if nbr_graph == 1:
    axes = [axes]  # Assurer que axes est une liste même pour un seul graphique
axes_titles = list(axis_wanted)
pos_cols, tar_cols = [], []
for i in range(nbr_graph):
    pos_cols.append(f"pos_{axes_titles[i]}")
    tar_cols.append(f"tar_{axes_titles[i]}")

for i in range(nbr_graph):
    axes[i].plot(df_filtered["time"], df_filtered[pos_cols[i]], label=f"Position {axes_titles[i]}", color="blue")
    axes[i].plot(df_filtered["time"], df_filtered[tar_cols[i]], label=f"Cible {axes_titles[i]}", color="red", linestyle="--")
    axes[i].set_ylabel(f"{axes_titles[i]}")
    axes[i].legend()
    axes[i].grid(True)

axes[nbr_graph-1].set_xlabel("Temps (s)")
fig.suptitle("Comparaison Position vs Cible")

plt.tight_layout()

# 🔧 Créer le dossier de sauvegarde
os.makedirs("ros2_ws/graph", exist_ok=True)

# 💾 Sauvegarder le graphique
plt.savefig(f"ros2_ws/graph/graph_{axis_wanted}_{dt_string}.png")
print(f"✅ Graphique sauvegardé : graph_{axis_wanted}_{dt_string}.png")