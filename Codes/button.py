import pygame

# Initialize Pygame
pygame.init()
pygame.joystick.init()

def print_joystick_info():
    """Print joystick button and axis numbers when they are pressed or moved."""
    # Keep track of initialized joysticks
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        joystick.init()
        print(f"Joystick '{joystick.get_name()}' initialized.")
    
    while True:
        # Check for joystick events
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                joystick = pygame.joystick.Joystick(event.device_index)
                joystick.init()
                print(f"Joystick '{joystick.get_name()}' added.")
                print_joystick_details(joystick)
            elif event.type == pygame.JOYAXISMOTION:
               # joystick = pygame.joystick.Joystick(event.device_index)
                axis_number = event.axis
                axis_value = joystick.get_axis(axis_number)
                print(f"Axis {axis_number} moved: {axis_value:.2f}")
            elif event.type == pygame.JOYBUTTONDOWN:
                joystick = pygame.joystick.Joystick(event.device_index)
                button_number = event.button
                print(f"Button {button_number} pressed")
            elif event.type == pygame.QUIT:
                pygame.quit()
                return

        # Poll joystick state for all connected joysticks
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            print_joystick_state(joystick)

def print_joystick_details(joystick):
    """Print details of joystick axes and buttons."""
    print(f"Joystick Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print("Axes:")
    for i in range(joystick.get_numaxes()):
        print(f"  Axis {i}: {joystick.get_axis(i)}")
    print("Buttons:")
    for i in range(joystick.get_numbuttons()):
        print(f"  Button {i}: {joystick.get_button(i)}")

def print_joystick_state(joystick):
    """Print the current state of joystick axes and buttons."""
    print("Current Joystick State:")
    print("Axes:")
    for i in range(joystick.get_numaxes()):
        print(f"  Axis {i}: {joystick.get_axis(i):.2f}")
    print("Buttons:")
    for i in range(joystick.get_numbuttons()):
        print(f"  Button {i}: {joystick.get_button(i)}")
    print("------------------------------")

if __name__ == '__main__':
    print_joystick_info()
    pygame.quit()