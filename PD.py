

class Controller(object):
    """docstring for Controller."""
    def __init__(self, omega=1, zeta=0.7):
        super(Controller, self).__init__()
        self.omega = omega
        self.zeta  = zeta
        self.gainP = omega ** 2
        self.gainD = 2 * zeta * omega

    def getInput(self, x, xDot, xNom=0, xDotNom=0):
        return - self.gainP * (x - xNom) - self.gainD * (xDot - xDotNom)
