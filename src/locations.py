coordinates = {
    "Atrium": (3, -13),
    "Atrium Door 1": (13.26, -10.26),
    "Atrium Door 2": (-7.3, -10.26),
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
}

adjacency = {
    "Atrium" : [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "Atrium Door 1": [1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    "Atrium Door 2": [1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    "CS Office": [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "CS Office Door": [0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    "East Room": [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    "East Room Door 1": [0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1],
    "East Room Door 2": [0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    "West Room": [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    "West Room Door 1": [0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1],
    "West Room Door 2": [0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1],
    "Electrical Lab": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
    "Electrical Lab Door 1": [0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
    "Electrical Lab Door 2": [0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1],.
    "CS Department": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    "CS Department Door": [0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0]
}

import os

commentaryMap = {
    "Atrium": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/Atrium.wav",
    "CS Office": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSOffice.wav",
    "East Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/EastRoom.wav",
    "West Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/WestRoom.wav",
    "Electrical Lab": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/ElectricalLab.wav",
    "CS Department": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSDepartment.wav",
    "Test": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/ElectricalLab.wav"
}
