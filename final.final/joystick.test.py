import pygame

# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check for connected joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a joystick and try again.")
    pygame.quit()
    exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick detected: {joystick.get_name()}")

try:
    while True:
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # Detect which button is pressed
                for button in range(joystick.get_numbuttons()):
                    if joystick.get_button(button):
                        print(f"Button {button} pressed.")

            elif event.type == pygame.JOYBUTTONUP:
                # Detect button release
                for button in range(joystick.get_numbuttons()):
                    if not joystick.get_button(button):
                        print(f"Button {button} released.")

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
