
import pandas as pd
import matplotlib.pyplot as plt

# Load simulation results
df = pd.read_csv("disturbance_sweep_results.csv")

controllers = ["Fixed PID", "Gain-Scheduled PID", "MPC"]

plots = [
    ("RollMSE", "Roll MSE (deg²)", "Roll MSE vs Disturbance Strength",
     "rollmse_vs_disturbance.png"),
    ("PitchMSE", "Pitch MSE (deg²)", "Pitch MSE vs Disturbance Strength",
     "pitchmse_vs_disturbance.png"),
]

for column, ylabel, title, filename in plots:
    fig, ax = plt.subplots(figsize=(8, 5))

    for controller in controllers:
        subset = df[df["Controller"].str.strip() == controller]

        ax.plot(
            subset["Disturbance_Nm"],
            subset[column],
            marker="o",
            linewidth=2,
            label=controller,
        )

    ax.set_xlabel("Disturbance Strength (N·m)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks([0.08, 0.12, 0.16])
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()

    fig.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {filename}")

print("\nAll graphs generated successfully.")
