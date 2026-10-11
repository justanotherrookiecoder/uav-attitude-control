import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("results/disturbance_sweep_results.csv")
controllers = ["Fixed PID", "Gain-Scheduled PID", "MPC"]

for axis in ["Roll", "Pitch"]:
    plt.figure(figsize=(8, 5))

    for controller in controllers:
        data = df[df["Controller"] == controller].sort_values(
            "Disturbance_Nm"
        )

        plt.plot(
            data["Disturbance_Nm"],
            data[f"{axis}MSE"],
            marker="o",
            linewidth=2,
            label=controller
        )

    plt.xlabel("Disturbance Strength (N·m)")
    plt.ylabel(f"{axis} Mean Squared Error (deg²)")
    plt.title(f"{axis} MSE vs Disturbance Strength")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    filename = f"{axis.lower()}_mse_corrected.png"
    plt.savefig(filename, dpi=300)
    plt.show()

    print(f"Saved: {filename}")