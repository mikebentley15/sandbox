import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np

def animate_plot_2d(generator, interval=200):
    '''
    Plots between 0 and 1 for both dimensions.  Will plot one additional point
    from the generator every time_delta seconds.  

    @param generator: generates the next point to plot (numpy.array)
    @param interval: milliseconds between replotting with the new point
    '''
    fig, ax = plt.subplots()
    scatter = ax.plot([], [], 'bo', ms=7)[0]
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    def plot_points(point):
        data = scatter.get_data()
        scatter.set_data(np.concatenate((data[0], [point[0]])),
                         np.concatenate((data[1], [point[1]])))
    ani = animation.FuncAnimation(fig, plot_points, generator,
                                  interval=interval)
    plt.show()
