import numpy as np


def smoothstep_sample(x, order=1):
    if order == 0:
        f = lambda x: x
    elif order == 1:
        f = lambda x: x**2 * (3 - 2*x)
    elif order == 2:
        f = lambda x: x**3 * (10 - 15*x + 6*x**2)
    elif order == 3:
        f = lambda x: x**4 * (35 - 84*x + 70*x**2 - 20*x**3)
    elif order == 4:
        f = lambda x: x**5 * (126 - 420*x + 540*x**2 - 315*x**3 + 70*x**4)
    elif order == 5:
        f = lambda x: x**6 * (462 - 1980*x + 3465*x**2 - 3080*x**3 + 1386*x**4 - 252*x**5)
    elif order == 6:
        f = lambda x: x**7 * (1716 - 9009*x + 20020*x**2 - 24024*x**3 + 16380*x**4 - 6006*x**5 + 924*x**6)

    return f(x)


def smoothstep(x0, y0, dx, dy, order=1, npts=10):
    """Generate smoothstep function.

    Args:
        x0 (np.ndarray): starting x point
        y0 (np.ndarray): starting y point
        dx (np.ndarray): span interval along x
        dy (np.ndarray): span interval along y
        order (int): order of the required smoothstep funciton (defaults to 1)
        npts (int): number of points to be computed (defaults to 10)

    Returns:
        x (np.ndarray): smoothstep x points
        y (np.ndarray): smoothstep y points
    """
    if order == 0:
        f = lambda x: x
    elif order == 1:
        f = lambda x: x**2 * (3 - 2*x)
    elif order == 2:
        f = lambda x: x**3 * (10 - 15*x + 6*x**2)
    elif order == 3:
        f = lambda x: x**4 * (35 - 84*x + 70*x**2 - 20*x**3)
    elif order == 4:
        f = lambda x: x**5 * (126 - 420*x + 540*x**2 - 315*x**3 + 70*x**4)
    elif order == 5:
        f = lambda x: x**6 * (462 - 1980*x + 3465*x**2 - 3080*x**3 + 1386*x**4 - 252*x**5)
    elif order == 6:
        f = lambda x: x**7 * (1716 - 9009*x + 20020*x**2 - 24024*x**3 + 16380*x**4 - 6006*x**5 + 924*x**6)
    x = np.linspace(0, 1, npts)
    y = f(x)
    x = [x0 + i * dx for i in x]
    y = [y0 + i * dy for i in y]
    return x, y