import time
import numpy as np
from environment import GazeboEnv
from action import Controller
import pygame
seed = 0  # Random seed number
max_ep = 500  # maximum number of steps per episode
file_name = "TD3_velodyne"  # name of the file to load the policy from

# Create the testing environment
environment_dim = 20
robot_dim = 4
env = GazeboEnv("multi_robot_scenario.launch", environment_dim)
time.sleep(20)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

done = False
episode_timesteps = 0
state = env.reset()



# Initialize Pygame
pygame.init()

# Set the dimensions of the window
window_size = [400, 400]
screen = pygame.display.set_mode(window_size)

# Set the title of the window
pygame.display.set_caption("Robot Navigation")

kv_p, kv_i, kv_d = 0, 0, 0
kw_p, kw_i, kw_d = 0, 0, 0

controller = Controller((kv_p, kv_i, kv_d), (kw_p, kw_i, kw_d))


# Begin the testing loop
while True:
    # action = network.get_action(np.array(state))

    # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
    # a_in = [(action[0] + 1) / 2, action[1]]

    # action_0: linear
    # action_1: angular
    # ROS_MASTER_URI=http://10.10.10.112:11311/
    # Set the background color
    background_color = (255, 255, 255)
    screen.fill(background_color)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        
    # Instructions for the user
    # print("Press 'r' to reset the simulation")
    # print("Press 'UP' to increase kv_p")
    # print("Press 'DOWN' to decrease kv_p")
    # print("Press 'RIGHT' to increase kv_i")
    # print("Press 'LEFT' to decrease kv_i")
    # print("Press 'PAGEUP' to increase kv_d")
    # print("Press 'PAGEDOWN' to decrease kv_d")
    # print("Press 'w' to increase kw_p")
    # print("Press 's' to decrease kw_p")
    # print("Press 'd' to increase kw_i")
    # print("Press 'a' to decrease kw_i")
    # print("Press 'e' to increase kw_d")
    # print("Press 'q' to decrease kw_d")

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:  # 'r' key for reset
                state = env.reset()
            
            elif event.key == pygame.K_c:
                controller.CONTROLLER_V.reset()
                controller.CONTROLLER_W.reset()
                kv_p, kv_i, kv_d = 0, 0, 0
                kw_p, kw_i, kw_d = 0, 0, 0
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)
            
            elif event.key == pygame.K_p:
                kv_p, kv_i, kv_d = 0.6, 0.0, 0.02
                kw_p, kw_i, kw_d = 5.0, 0.33, 0.02
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_UP:  # 'UP' key to increase kv_p
                kv_p += 0.1
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)
            elif event.key == pygame.K_DOWN:  # 'DOWN' key to decrease kv_p
                kv_p -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_RIGHT:  # 'RIGHT' key to increase kv_i
                kv_i += 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_LEFT:  # 'LEFT' key to decrease kv_i
                kv_i -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)
                
            elif event.key == pygame.K_PAGEUP:  # 'PAGEUP' key to increase kv_d
                kv_d += 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)
            elif event.key == pygame.K_PAGEDOWN:  # 'PAGEDOWN' key to decrease kv_d
                kv_d -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_w:  # 'w' key to increase kw_p
                kw_p += 0.1
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_s:  # 's' key to decrease kw_p
                kw_p -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_d:  # 'd' key to increase kw_i
                kw_i += 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_a:  # 'a' key to decrease kw_i
                kw_i -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_e:  # 'e' key to increase kw_d
                kw_d += 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)

            elif event.key == pygame.K_q:  # 'q' key to decrease kw_d
                kw_d -= 0.01
                controller.CONTROLLER_V.tunings = (kv_p, kv_i, kv_d)
                controller.CONTROLLER_W.tunings = (kw_p, kw_i, kw_d)
                
            print("kv_p: ", kv_p, "|", "kv_i: ", kv_i, "|", "kv_d: ", kv_d)
            print("kw_p: ", kw_p, "|", "kw_i: ", kw_i, "|", "kw_d: ", kw_d)

            # Render the controller tunings and action to the pygame window
            

    font = pygame.font.Font(None, 15)
    text = font.render("Instructions:", 1, (10, 10, 10))
    screen.blit(text, (20, 20))
    instructions = [
        "Press 'r' to reset the simulation",
        "Press 'c' to reset the controller",
        "Press 'p' to set the controller to default values",
        "Press 'UP' to increase kv_p",
        "Press 'DOWN' to decrease kv_p",
        "Press 'RIGHT' to increase kv_i",
        "Press 'LEFT' to decrease kv_i",
        "Press 'PAGEUP' to increase kv_d",
        "Press 'PAGEDOWN' to decrease kv_d",
        "Press 'w' to increase kw_p",
        "Press 's' to decrease kw_p",
        "Press 'd' to increase kw_i",
        "Press 'a' to decrease kw_i",
        "Press 'e' to increase kw_d",
        "Press 'q' to decrease kw_d"
    ]
    for i, instruction in enumerate(instructions):
        text = font.render(instruction, 1, (0, 0, 0))
        screen.blit(text, (20, 40 + i * 10))

    text = font.render("kv_p: " + str(kv_p) + " | kv_i: " + str(kv_i) + " | kv_d: " + str(kv_d), 1, (10, 10, 10))
    screen.blit(text, (20, 60 + len(instructions) * 10))
    text = font.render("kw_p: " + str(kw_p) + " | kw_i: " + str(kw_i) + " | kw_d: " + str(kw_d), 1, (10, 10, 10))
    screen.blit(text, (20, 100 + len(instructions) * 10))


    # a_in = [0.3, 0.0]
    action = controller.get_action(state)
    a_in = action
    next_state, done, target = env.step(a_in)


    print("distance: ", next_state[0], "|", "theta: ", next_state[1])
    print("action: ", action)
    # Render distance and theta to the pygame window
    text = font.render("distance: " + str(next_state[0]) + " | theta: " + str(next_state[1]), 1, (10, 10, 10))
    screen.blit(text, (20, 140 + len(instructions) * 10))

    # Render action to the pygame window
    text = font.render("action: " + str(action), 1, (10, 10, 10))
    screen.blit(text, (20, 180 + len(instructions) * 10))
    pygame.display.flip()
    
    done = int(done)

    # On termination of episode
    if done:
        state = env.reset()
        done = False
        episode_timesteps = 0
    # continue
    else:
        state = next_state
        episode_timesteps += 1

