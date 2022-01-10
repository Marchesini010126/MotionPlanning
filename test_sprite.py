import pygame
import sys,os
import numpy as np

pygame.init() # initialize pygame
clock = pygame.time.Clock()
screen = pygame.display.set_mode((600,500))

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255, 50)  # This color contains an extra integer. It's the alpha value.
PURPLE = (255, 0, 255)
gray1 = (192,192,192)
gray2 = (247,247,247)

class Ship(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)
        
        self.image_file = image_file
        
        self.image_original = pygame.image.load(image_file).convert()
        self.image          = pygame.image.load(image_file).convert()
        
        self.image.set_colorkey(gray2)

        self.image_original = pygame.transform.scale(self.image_original, (85, 65)) 
        self.image          = pygame.transform.scale(self.image, (85, 65)) 
        
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location
        self.angle = 90
        
    def update(self) :
        
        self.image  = pygame.transform.rotate(self.image_original, self.angle)
        self.image.set_colorkey(gray2)
        
        self.angle += 0.011 % 360  # Value will reapeat after 359. This prevents angle to overflow.
        x, y = self.rect.center  # Save its current center.
        self.rect = self.image.get_rect()  # Replace old rect with new rect.
        self.rect.center = (x, y)  # Put the new rect's center at old center.

ship = Ship("car_sprite.png",(0,0))

running = True

while running:
    # check if quit was pressed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    screen.fill(BLACK)     
    ship.update()
    screen.blit(ship.image,ship.rect)
    pygame.display.update()
    
    