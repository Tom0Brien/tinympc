import matplotlib.pyplot as plt
import numpy as np


def plot_history(filename, n, m):
    data = np.loadtxt(filename, delimiter=',')

    # Assuming states and controls are concatenated in each row
    states = data[:, :n]
    controls = data[:, n:n+m]

    time = range(len(states))

    # Plotting states
    plt.figure(figsize=(10, 6))
    for i in range(n):
        plt.plot(time, states[:, i], label=f'State {i + 1}')
    plt.title("States Over Time")
    plt.xlabel("Time step")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plotting controls
    plt.figure(figsize=(10, 6))
    for i in range(m):
        plt.plot(time, controls[:, i], label=f'Control {i + 1}')
    plt.title("Controls Over Time")
    plt.xlabel("Time step")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    filename = "history.csv"  # Update with your filename
    n, m = 12, 6              # Update with your dimensions for state and control
    plot_history(filename, n, m)
