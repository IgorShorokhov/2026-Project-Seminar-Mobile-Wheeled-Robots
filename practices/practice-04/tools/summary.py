import os

import numpy as np

results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
run_names = [
    "global_p1000",
    "global_p15000",
    "initial_small_p1000",
    "initial_small_p15000",
    "initial_large_p1000",
    "initial_large_p15000",
]

print("| запуск | время сходимости, с | e_p после 60 с (среднее), м | e_yaw после 60 с (среднее), рад |")
print("|---|---|---|---|")
for run_name in run_names:
    data = np.load(os.path.join(results_dir, f"{run_name}.npz"))
    after = data["time"] > 60
    convergence_time = float(data["convergence_time"])
    if np.isnan(convergence_time):
        convergence_text = "нет"
    else:
        convergence_text = f"{convergence_time:.1f}"
    mean_position = data["errors_position"][after].mean()
    mean_yaw = data["errors_yaw"][after].mean()
    print(f"| `{run_name}` | {convergence_text} | {mean_position:.3f} | {mean_yaw:.3f} |")
