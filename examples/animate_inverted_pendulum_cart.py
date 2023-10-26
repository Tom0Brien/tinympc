import matplotlib.pyplot as plt
import matplotlib.animation as animation
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


def animate_cartpole(filename):
    data = np.loadtxt(filename, delimiter=',')
    states = data[:, :4]
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim([-3, 3])
    ax.set_ylim([-1, 1.5])

    cart, = ax.plot([], [], 's-', markersize=20, color='black')
    pole, = ax.plot([], [], 'o-', linewidth=4, color='blue')

    def init():
        cart.set_data([], [])
        pole.set_data([], [])
        return cart, pole

    def update(i):
        x = states[i, 0]
        theta = states[i, 2]

        cart_x = [x - 0.2, x + 0.2]
        cart_y = [0, 0]
        pole_x = [x, x + np.sin(theta)]
        pole_y = [0, np.cos(theta)]

        cart.set_data(cart_x, cart_y)
        pole.set_data(pole_x, pole_y)
        return cart, pole

    ani = animation.FuncAnimation(fig, update, frames=len(
        states), init_func=init, blit=True, interval=100)
    plt.show()


if __name__ == "__main__":
    filename = "history.csv"  # Update with your filename
    n, m = 4, 1             # Update with your dimensions for state and control
    plot_history(filename, n, m)
    animate_cartpole(filename)
