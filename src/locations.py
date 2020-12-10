coordinates = {
    "Attrium": (3, -13),
    "Attrium Door 1": (13.26, -10.26),
    "Attrium Door 2": (-7.3, -10.26),
    "CS Office": (11.51, -5.83),
    "CS Office Door": (14.63, -7.77),
    "East Room": (6, 4),
    "East Room Door 1": (11.5, 0),
    "East Room Door 2": (1.6, 0),
    "West Room": (-4.4, 4),
    "West Room Door 1": (-0.6, 0),
    "West Room Door 2": (-8, 0),
    "Electrical Lab": (0, 6),
    "Electrical Lab Door 1": (5.65, -4),
    "Electrical Lab Door 2": (-4.4, 4.1),
    "CS Department": (-16.2, 0),
    "CS Department Door": (-11.75, -1.3),
    "Test": (0, 5)
}

import os

commentaryMap = {
    "Attrium": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/Atrium.wav",
    "CS Office": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSOffice.wav",
    "East Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/EastRoom.wav",
    "West Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/WestRoom.wav",
    "Electrical Lab": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/ElectricalLab.wav",
    "CS Department": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSDepartment.wav",
    "Test": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/ElectricalLab.wav"
}