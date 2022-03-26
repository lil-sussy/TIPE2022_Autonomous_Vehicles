import pygame
import numpy as np
import MapRendering

#(x, y) _______________ x (le référentiel de l'écran)
#      |
#      |
#      y
class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y, masse, A):
        pygame.sprite.Sprite.__init__(self)
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        picture_path = MapRendering.WorldRenderer.RESSOURCES_PATH + "Car/car.png"
        self.image = pygame.image.load(picture_path) # The image of the sprite (the car)
        self.sprite_image = pygame.image.load(picture_path) # The image scaled and rotated to be drawn on the map
        self.sprite_origin = (0, 0) # The position of the sprite to be drawn on the map
        self.rect = pygame.Rect(x, y, self.image.get_rect().height, self.image.get_rect().height)
        #Vehicle Dynamic
        self.m = masse #kg
        self.A = A #mud * g
        self.fm = 0 #force motrice divisé par m
        self.frottements = 0 #Coefficient de frottements visqueux de l'air (et peut être de la route)
        self.volant = 0 #Direction du volant (angle des roues avec la vitesse) en degrés
                        #Il est compris entre 70° et -70° (arbitraire)
                        #Angle entre l'accélaration et la vitesse
        self.OM = (x, y) #Vecteur position (coordonées dans (x, y))
        self.v = (0, 0) #Vecteur vitesse dans le repere (x, y)
        self.teta = 0
        self.a = (0, 0) #Vecteur Accéleration dans le repere (x, y)
        #Neat AI
        self.sensors = []
        self.distance = 0
    
    def UpdatePosistion(self, dt):
        #a(t + dt) = a(t + dt)
        #v(t + dt) = v(t) + dt*a(t + dt)
        (vx, vy) = self.v; (ax, ay) = self.a
        self.v = (vx + dt*ax, vy + dt*ay)
        self.UpdateAcceleration(dt)
        #dOM(t + dt) = dt*v(t + dt)
        (vx, vy) = self.v; (x, y) = self.OM
        self.OM = (x+dt*vx, y+dt*vy)
        (x, y) = self.OM
        self.rect.center = self.OM

    def SetAcceleration(self, f, frottements, volant):
        # Volant : angle du volant (entre la vitesse et l'accélération)
        if(volant > 70): #On ne peut pas tourner trop le volant
            volant = 70
        if(volant <= -70):
            volant = -70
        self.volant = volant
        fm = f/self.m
        self.fm = fm
        self.frottements = frottements
        self.UpdateAcceleration(0)

    def UpdateAcceleration(self, dt):
        # Voir PFD et projections de a dans (x, y)
        (vx, vy) = self.v
        v = np.sqrt(vx**2 + vy**2)
        self.distance += v*dt #On met à jour la distance total parcourue
        dteta = 5*self.A*sin(self.volant) #dteta = -2*
        teta = self.teta = self.EvaluateTeta()
        self.a = (self.fm*cos(self.volant+teta+dteta) - self.frottements*v*cos(teta) 
            , -self.fm*sin(self.volant+teta+dteta) + self.frottements*v*sin(teta))
        return

    def EvaluateTeta(self):
        (vx, vy) = self.v
        if (vx == 0):
            if(vy >= 0):
                return 0
            else:
                return 180
        return np.rad2deg(np.arctan(vy/vx))


    def update(self, map, screen, dt):
        pygame.sprite.Sprite.update(self)
        self.CheckRoadCollisions(map, screen, dt)
        self.UpdateSensors(map, screen, dt)
        #Moving
        self.UpdatePosistion(dt)
        #rotating the sprite
        imgRec = self.image.get_rect()
        #The sprite is being drawed in the direction of the velocity (teta)
        self.sprite_image, self.sprite_origin = blitRotate(screen, self.image, (self.rect.x+self.rect.w//2, self.rect.y+self.rect.h//2), (imgRec.w/2, imgRec.h/2), self.volant)

    def render(self, map, screen):
        #draw the rotated sprite
        screen.blit(self.sprite_image, self.sprite_origin)
        #draw hitbox
        pygame.draw.rect(screen, (0, 255, 0), (*self.rect.topleft, *self.rect.size), 2)
        #draw rotations
        (x, y) = self.rect.center; 
        coef = 1
        #pygame.draw.line(screen, (200, 40, 40), (x, y), (x+length*cos(self.teta+self.volant), y-length*sin(self.teta+self.volant)), 1) #Draw 'Volant' Orientation
        (vx, vy) = self.v; (ax, ay) = self.a
        pygame.draw.line(screen, (40, 200, 40), (x, y), (x+vx*coef, y+vy*coef), 2) #Draw velocity vect
        pygame.draw.line(screen, (40, 40, 200), (x, y), (x+ax*coef, y+ay*coef), 2) #Draw acceleration vect


    def CheckRoadCollisions(self, map, screen, dt):
        for (x, y) in [self.rect.bottomleft, self.rect.topright, self.rect.bottomright, self.rect.topleft]:
            if x < map.raw_map.get_width() and y < map.raw_map.get_height() and x >= 0 and y >= 0:
                if not SameColor(map.raw_map.get_at((x, y)), map.road_color, 0.05):  #5%, on regarde la couleur de la route sur la carte
                    self.kill()
            else: self.kill()
    
    def UpdateSensors(self, map, screen, dt):
        cx = self.rect.centerx
        cy = self.rect.centery
        for sensor_orientation in range(-90, 120, 45):
            detection_size = 0
            x = int(cx + cos(360 - (self.teta + sensor_orientation)) * detection_size)
            y = int(cy + sin(360 - (self.teta + sensor_orientation)) * detection_size)

            # While We Don't Hit BORDER_COLOR AND length < 300 (just a max) -> go further and further
            if x >= 0 and x < map.raw_map.get_width() and y >= 0 and y < map.raw_map.get_height():
                while not SameColor(map.raw_map.get_at((x, y)), map.road_color, 0.05) and detection_size < 300:
                    detection_size += 1
                    x = int(cx + cos(360 - (self.teta + sensor_orientation)) * detection_size)
                    y = int(cy + sin(360 - (self.teta + sensor_orientation)) * detection_size)

            # Calculate Distance To Border And Append To Radars List
            dist = int(np.sqrt((x - cx)**2 + (y - cy)**2))
            self.sensors.append([(x, y), dist])
    
    def GetInputsData(self, divider = 30):
        radars_detections = [0, 0, 0, 0, 0]
        for i, radar in enumerate(self.sensors):
            radars_detections[i] = int(radar[1] / divider)
        return radars_detections
    
    def GetReward(self, divider):
        reward = self.distance/divider
        self.distance = 0
        return reward


def sin(teta):
    return np.sin(np.deg2rad(teta))
def cos(teta):
    return np.cos(np.deg2rad(teta))

def blitRotate(screen, image, pos, originPos, angle):

    # calcaulate the axis aligned bounding box of the rotated image
    w, h       = image.get_size()
    box        = [pygame.math.Vector2(positions) for positions in [(0, 0), (w, 0), (w, -h), (0, -h)]]
    box_rotate = [vectors.rotate(angle) for vectors in box]
    min_box    = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
    max_box    = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

    # calculate the translation of the pivot 
    pivot        = pygame.math.Vector2(originPos[0], -originPos[1])
    pivot_rotate = pivot.rotate(angle)
    pivot_move   = pivot_rotate - pivot

    # calculate the upper left origin of the rotated image
    origin = (pos[0] - originPos[0] + min_box[0] - pivot_move[0], pos[1] - originPos[1] - max_box[1] + pivot_move[1])

    # get a rotated image
    rotated_image = pygame.transform.rotate(image, angle)
    return rotated_image, origin

def SameColor(color1, color2, delta):
    delta = int(delta*255)
    if(abs(color1.r - color2.r) <= delta):
        if(abs(color1.g - color2.g) <= delta):
            if(abs(color1.b - color2.b) <= delta):
                return True
    return False