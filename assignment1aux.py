from os import system
from time import sleep

def visualise(state):
    map = state[0]
    position = state[1]
    move = state[2]
    height = len(map)
    width = len(map[0])
    print('\n ', *['\u2581' for _ in range(width)], ' ', sep='')
    for i in range(height):
        print('\u2595', end='')
        for j in range(width):
            if map[i][j] == 'rock':
                print('\u2588', end='')
            elif map[i][j] == 'left':
                print('\u25c2', end='')
            elif map[i][j] == 'up':
                print('\u25b4', end='')
            elif map[i][j] == 'right':
                print('\u25b8', end='')
            elif map[i][j] == 'down':
                print('\u25be', end='')
            elif position == (i, j):
                if move == 'right':
                    print('\u2520', end='')
                elif move == 'left':
                    print('\u2528', end='')
                elif move == 'down':
                    print('\u252f', end='')
                elif move == 'up':
                    print('\u2537', end='')
                else:
                    print('\u25cb', end='')
            elif not map[i][j]:
                print(' ', end='')
            else:
                print()
                print(f"Unexpected tile representation encountered: '{map[i][j]}'.")
                print("Accepted tile representations are '', 'rock', 'left', 'right', 'up', and 'down'.")
                raise ValueError(map[i][j])
        print('\u258f')
    print(' ', *['\u2594' for _ in range(width)], ' \n', sep='')

def animate(node):
    for node_i in node.path()[:-1]:
        visualise(node_i.state)
        sleep(2)
        for _ in range(len(node_i.state[0]) + 4):
            system('tput cuu1')
    visualise(node.state)
