from locations import *


def read_input(prompt):
    x = input(prompt)
    while x:
        yield x
        x = input(prompt)


if __name__ == '__main__':
    print("Please enter which rooms you would like to visit")
    print("Accepted commands are all, atrium, office, east, west, electrical, and department")
    print("Enter blank line when you are done selecting")
    input_text = list(read_input("->"))

    curr_coordinates = []

    if 'all' in input_text:
        curr_coordinates.append(coordinates["Attrium"])
        curr_coordinates.append(coordinates["Attrium Door 1"])
        curr_coordinates.append(coordinates["CS Office Door"])
        curr_coordinates.append(coordinates["CS Office"])
        curr_coordinates.append(coordinates["CS Office Door"])
        curr_coordinates.append(coordinates["East Room Door 1"])
        curr_coordinates.append(coordinates["East Room"])
        curr_coordinates.append(coordinates["East Room Door 2"])
        curr_coordinates.append(coordinates["West Room Door 1"])
        curr_coordinates.append(coordinates["West Room"])
        curr_coordinates.append(coordinates["West Room Door 2"])
        curr_coordinates.append(coordinates["Electrical Lab Door 1"])
        curr_coordinates.append(coordinates["Electrical Lab"])
        curr_coordinates.append(coordinates["Electrical Lab Door 2"])
        curr_coordinates.append(coordinates["CS Department Door"])
        curr_coordinates.append(coordinates["CS Department"])
        curr_coordinates.append(coordinates["CS Department Door"])
        curr_coordinates.append(coordinates["Attrium Door 2"])
    else:
        if 'atrium' in input_text:
            curr_coordinates.append(coordinates["Attrium"])
            curr_coordinates.append(coordinates["Attrium Door 1"])
            curr_coordinates.append(coordinates["Attrium Door 2"])
        if 'office' in input_text:
            curr_coordinates.append(coordinates["CS Office Door"])
            curr_coordinates.append(coordinates["CS Office"])
        if 'east' in input_text:
            curr_coordinates.append(coordinates["East Room Door 1"])
            curr_coordinates.append(coordinates["East Room"])
            curr_coordinates.append(coordinates["East Room Door 2"])
        if 'west' in input_text:
            curr_coordinates.append(coordinates["West Room Door 1"])
            curr_coordinates.append(coordinates["West Room"])
            curr_coordinates.append(coordinates["West Room Door 2"])
        if 'electrical' in input_text:
            curr_coordinates.append(coordinates["Electrical Lab Door 1"])
            curr_coordinates.append(coordinates["Electrical Lab"])
            curr_coordinates.append(coordinates["Electrical Lab Door 2"])
        if 'department' in input_text:
            curr_coordinates.append(coordinates["CS Department Door"])
            curr_coordinates.append(coordinates["CS Department"])
            curr_coordinates.append(coordinates["CS Department Door"])

    print(curr_coordinates)
