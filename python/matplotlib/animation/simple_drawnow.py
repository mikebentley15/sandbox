'''
Largely taken (and simplified) from https://github.com/stsievert/python-drawnow
which can be installed with `pip install drawnow`.

This function simply wraps around matplotlib.pyplot.draw_all()
'''

from matplotlib import pyplot as plt

def drawnow(draw_func, fig=None, *args, **kwargs):
    '''Refresh the current figure

    Depends on matplotlib's interactive mode.  Similar functionality to
    MATLAB's drawnow.

    Parameters
    ----------
    draw_func : callable
        The function that draws the figure you want to update.
    fig : matplotlib.Figure or None
        The figure to update.  If None, will update all of them.
    *args : list
        A list of parameters to pass to ``draw_fig()``.
    **kwargs : dict
        The keywords to pass to ``draw_fig()``.
    '''

    draw_func(*args, **kwargs)
    if fig:
        fig.canvas.draw_idle()
    else:
        plt.draw_all()
    plt.pause(1e-3)  # allow time to draw

