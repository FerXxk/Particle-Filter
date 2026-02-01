import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# Ruta base donde están los CSV
csv_dir = os.path.expanduser('~/pf_logs')

# Leer archivo con posiciones reales de balizas
ref_path = os.path.join(csv_dir, 'beacon_positions.csv')
ref_df = pd.read_csv(ref_path, index_col='beacon_id')  # Asegúrate de que "beacon_id" sea la columna clave

# Buscar archivos del filtro de partículas y EKF
pf_files = sorted(glob.glob(os.path.join(csv_dir, 'pf_means_*.csv')))
ekf_files = sorted(glob.glob(os.path.join(csv_dir, 'pf_means_ekf_*.csv')))

# Filtrar solo los archivos de PF (que no son EKF)
pf_files = [f for f in pf_files if 'ekf' not in f]

# Asociar balizas por ID
beacon_ids = [int(os.path.splitext(os.path.basename(f))[0].split('_')[-1]) for f in pf_files]

# Crear subplots
num_beacons = len(beacon_ids)
fig, axs = plt.subplots(num_beacons, 1, figsize=(12, 4 * num_beacons), sharex=True)

if num_beacons == 1:
    axs = [axs]

# Procesar cada baliza
for i, beacon_id in enumerate(beacon_ids):
    pf_file = os.path.join(csv_dir, f'pf_means_{beacon_id}.csv')
    ekf_file = os.path.join(csv_dir, f'pf_means_ekf_{beacon_id}.csv')

    if not os.path.exists(pf_file) or not os.path.exists(ekf_file):
        print(f"[!] Archivos faltantes para beacon {beacon_id}")
        continue

    # Cargar datos
    pf_df = pd.read_csv(pf_file)
    ekf_df = pd.read_csv(ekf_file)

    # Tiempo donde comienza EKF
    ekf_start_time = ekf_df['timestamp'].iloc[0]

    # Concatenar y ordenar
    df = pd.concat([pf_df, ekf_df], ignore_index=True).sort_values(by='timestamp')

    t = df['timestamp'].to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()

    # Referencia real
    if beacon_id not in ref_df.index:
        print(f"[!] Beacon ID {beacon_id} no encontrado en beacon_positions.csv")
        continue
    x_ref = ref_df.loc[beacon_id, 'x']
    y_ref = ref_df.loc[beacon_id, 'y']
    z_ref = ref_df.loc[beacon_id, 'z']

    # Graficar
    ax = axs[i]
    ax.plot(t, x, label='X estimado', color='r')
    ax.plot(t, y, label='Y estimado', color='g')
    ax.plot(t, z, label='Z estimado', color='b')

    ax.axvline(x=ekf_start_time, color='purple', linestyle='--', label='Inicio EKF')
    ax.hlines(x_ref, t[0], t[-1], color='r', linestyle='dashed', label='X real')
    ax.hlines(y_ref, t[0], t[-1], color='g', linestyle='dashed', label='Y real')
    ax.hlines(z_ref, t[0], t[-1], color='b', linestyle='dashed', label='Z real')

    ax.set_title(f'Estimación Beacon {beacon_id}')
    ax.set_ylabel('Coordenada (m)')
    ax.grid(True)
    ax.legend(loc='upper right')

axs[-1].set_xlabel('Tiempo (s)')
plt.tight_layout()
plt.show()
