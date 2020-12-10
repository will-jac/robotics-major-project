coordinates = {
    "Atrium": (-2, -9),
    "CS Office": (11.51, -3.83),
    "East Room": (6, 6),
    "West Room": (-4.4, 6),
    "Electrical Lab": (0, -3),
    "CS Department": (-15.2, 2),
}

adjacency = {
    "Atrium" : [0, 1, 2, 2, 1, 3],
    "CS Office": [1, 0, 1, 2, 2, 3],
    "East Room": [2, 1, 0, 1, 1, 2],
    "West Room": [2, 2, 1, 0, 1, 1],
    "Electrical Lab": [1, 2, 1, 1, 0, 2],
    "CS Department": [3, 3, 2, 1, 2, 0],
}

index = {
    "Atrium" : 0,
    "CS Office": 1,
    "East Room": 2,
    "West Room": 3,
    "Electrical Lab": 4,
    "CS Department": 5,
}

import os

commentaryMap = {
    "Atrium": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/Atrium.wav",
    "CS Office": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSOffice.wav",
    "East Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/EastRoom.wav",
    "West Room": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/WestRoom.wav",
    "Electrical Lab": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/ElectricalLab.wav",
    "CS Department": os.path.dirname(os.path.realpath(__file__)) + "/../commentary/CSDepartment.wav"
}
