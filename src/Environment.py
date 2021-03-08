import pygame


def environment2D(pt):
    pygame.init()

    res = 1.0  # resolution of grid
    scale = 40  # scale of grid

    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    yellow = (255, 255, 0)

    size_x = 20
    size_y = 20
    gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))
    gameDisplay.fill(white)
    pygame.draw.rect(gameDisplay, black, [int(scale * 8), int(scale * 9), int(scale * 4.5), int(scale * 0.5)])  # plus
    pygame.draw.rect(gameDisplay, black, [int(scale * 10), int(scale * 7), int(scale * 0.5), int(scale * 4.5)])  # plus
    pygame.draw.rect(gameDisplay, black, [int(scale * 4), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
    pygame.draw.rect(gameDisplay, black, [int(scale * 1.5), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
    pygame.draw.rect(gameDisplay, black, [int(scale * 16), int(scale * 8), int(scale * 0.25), int(scale * 2.5)])  # |
    pygame.draw.rect(gameDisplay, black, [int(scale * 17), int(scale * 9), int(scale * 1.5), int(scale * 0.25)])  # -
    pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 3), int(scale * 2.5), int(scale * 0.25)])  # -
    pygame.draw.rect(gameDisplay, black, [int(scale * 10.15), int(scale * 0.8), int(scale * 0.25), int(scale * 1.5)])  # |
    pygame.draw.rect(gameDisplay, black, [int(scale * 9), int(scale * 15), int(scale * 2.5), int(scale * 0.25)])  # -
    pygame.draw.rect(gameDisplay, black, [int(scale * 10.15), int(scale * 16), int(scale * 0.25), int(scale * 1.5)])  # |

    pygame.draw.circle(gameDisplay, red, (pt[0] * scale, pt[1] * scale), 0.08 * scale)
    pygame.display.update()
    pygame.time.delay(5000)
    pygame.quit()


# def environment3D():
#     pass
#
#
# def main():
#     environment2D()
#
#
# if __name__ == "__main__":
#     main()
