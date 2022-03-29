import pygame
import numpy as np
import Packages.MapComponents.ImageManager as imm

#(x, y) _______________ x (le référentiel de l'écran)
#      |
#      |
#      y
id = 0
class Vehicle(pygame.sprite.Sprite):
    def Copy(car):
        x, y = car.OM
        return Vehicle(x, y, car.m, car.J, car.R, car.JR, car.GA, car.E, car.roadColor, car.ressourcePath, car.debugLevel)
    
    def __init__(self, x, y, masse, J, R, JR, GA, L, roadColor, ressourcePath, debugLevel):
        pygame.sprite.Sprite.__init__(self)
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        picture_path = ressourcePath + "Car/car.png"
        self.image = pygame.image.load(picture_path) # The image of the sprite (the car)
        self.sprite_image = pygame.image.load(picture_path) # The image scaled and rotated to be drawn on the map
        self.sprite_origin = (0, 0) # The position of the sprite to be drawn on the map
        w = self.image.get_rect().width
        h = self.image.get_rect().height
        (x, y) = (x-w/2, y-h/2)
        self.rect = pygame.Rect(x, y, h, h)
        global id
        id += 1
        self.id = id
        self.debugLevel = debugLevel
        self.roadColor = roadColor # Couleur de la route (pixel)
        self.ressourcePath = ressourcePath # Utile uniquement pour la copie de voiture
        
        #Physics
        self.alpha = 0 #deg, orientation de la voiture par rapport à l'axe x
        self.a = (0, 0)
        self.na = 0
        self.v = (0, 0)
        self.OM = (x, y)
        
        self.m = masse #kg
        self.J = J #kg m²
        self.JR = JR # Utile uniquement pour la copie de voiture
        self.roues = R/JR #kg-1 m-1
        self.R = R #m rayon des roues
        self.E = L #m (longeur de la voiture)
        self.GA = GA #m distance entre les roues arrières et le centre de gravité G
        self.coupleM = 0
        self.coupleF = 0
        
        self.teta = 0 #deg angle du virage
        self.lbda = 0
        self.tetapoint = 0 #deg s-1vitesse angulaire du virage
        self.tetapointzero = 0
        self.tetazero = 0
        self.r = 0 #m rayon du virage
        self.omega = (0, 0) #m centre du virage
        
        #Neat
        self.distance = 0 #Distance parcourue
        self.sensors = []
        self.vehicle_outputs = 5 #Le nombre de radars
        self.vehicle_inputs = 2 #Le volant et le coupleM
    
    def SetCarAcceleration(self, coupleM, coupleF, lbda):
        # Volant : angle du volant (entre la vitesse et l'accélération)
        MAX_VOLANT = 70
        if(lbda > MAX_VOLANT): #On ne peut pas tourner trop le volant
            lbda = MAX_VOLANT
        if(lbda <= -MAX_VOLANT):
            lbda = -MAX_VOLANT
        
        if (lbda == self.lbda and coupleM == self.coupleM and coupleF == self.coupleF):
            return #Rien n'a changé
        lb = self.lbda = lbda
        self.coupleM = coupleM
        self.coupleF = coupleF
        if (np.abs(lb) < 0.02):
            a = self.roues*(self.coupleM - self.coupleF)
            self.na = a
            self.a = (a*cos(self.alpha), -a*sin(self.alpha))
            self.tetapoint = 0
            self.r = 0
        else :
            self.a = (0, 0) #Uniquement la vitesse est calculée
            r = self.r = self.E/np.deg2rad(np.abs(lb))
            (x, y) = self.rect.center
            if lb < 0: #On tourne à gauche, donc omega est r*ealpha plus loin
                self.teta = 90 - self.alpha
                self.teta = self.teta
                self.omega = (x-r*cos(self.teta), y-r*sin(self.teta))
            else: #On tourne à droite, donc omega est -r*ealpha plus loin
                self.teta = 90 - self.alpha
                #xomega/r = cos(teta); yomega/r = sin(teta)
                self.omega = (x+r*cos(self.teta), y+r*sin(self.teta))
            self.tetapointzero = np.sqrt(np.abs(self.J/(self.R*r*self.m*(cos(lb)*self.J + self.GA*self.r*self.m))))
            if (self.coupleM - self.coupleF) < 0:#Pas de marche arrière possible (frein = freinage)
                self.tetapointzero = 0
            else:
                coef = self.na
                if(lb > 0):
                    self.tetapoint = np.sqrt(np.abs(sin(lb))*(self.coupleM - self.coupleF))*self.tetapointzero*coef
                else:
                    self.tetapoint = -np.sqrt(np.abs(sin(lb))*(self.coupleM - self.coupleF))*self.tetapointzero*coef
            #Juste pour dessiner :
            self.tetazero = self.alpha + 90
            self.positionzero = self.rect.center
    
    def update(self, map, screen, dt): # Overrided, fonction appelé par pygame lors de l'update de la carte
        def UpdateCarPosition(dt):
            if (np.abs(self.lbda) < 0.02): # Si la voiture n'est pas en mode virage
                (vx, vy) = self.v; (ax, ay) = self.a
                self.v = (vx + ax*dt, vy + ay*dt)
            else:
                self.teta += np.rad2deg(self.tetapoint)*dt # On met à jour l'angle de la voiture par rapport à l'horizontale
                rtetapoint = self.r*self.tetapoint # mise à jour de la vitesse angulaire
                if(self.lbda > 0): # Virage à gauche
                    self.alpha = 90 - self.teta
                    self.v = (rtetapoint*cos(self.alpha), -rtetapoint*sin(self.alpha))
                else: # Virage à droite
                    self.alpha = 90 - self.teta
                    self.alpha = self.alpha
                    self.v = (-rtetapoint*cos(self.alpha), rtetapoint*sin(self.alpha))
            (x, y) = self.OM; (vx, vy) = self.v # Mise à jour de la vitesse
            self.OM = (x + vx*dt, y + vy*dt) # Mise à jour de la position
            (x, y) = self.OM
            self.distance += x + y # On met à jour la distance parcourue SANS LA NORME EUCLIDIENNE
            self.rect.center = (int(x), int(y)) # On déplace la hitbox de la voiture
        pygame.sprite.Sprite.update(self) # Rendu du sprite de la voiture sur la carte ??
        self.CheckCarRoadCollisions(map)
        self.UpdateCarSensors(map)
        UpdateCarPosition(dt)
        imgRec = self.image.get_rect()
        x, y = (self.rect.x, self.rect.y)
        carMiddle = (x + self.rect.w//2, y + self.rect.h//2) # Milieu de la voiture
        spriteMiddle = (imgRec.w/2, imgRec.h/2)
        self.sprite_image, self.sprite_origin = blitRotate(self.image, carMiddle, spriteMiddle, self.alpha) # Position du centre de l'image : centre de la voiture, rotation autour du centre de l'image d'un angle alpha
    
    def Render(self, map, screen): # Rendu de la hitbox ect..., fonction appelé par ScreenRenderer
        level = self.debugLevel # Niveau de débuggage, niveau de de détails supplémentaires
        if level >= 0: # Niveau 0 est le niveau par défaut, niveau -1 : il est possible de ne pas faire le rendu des voitures
            screen.blit(self.sprite_image, self.sprite_origin) # Rendu du sprite de la voiture
        if level >= 1:
            pygame.draw.rect(screen, (0, 255, 0), (*self.rect.topleft, *self.rect.size), 2) # Dessin de la HitBox de la voiture
        if level >= 2: # Niveau max : niveau 3
            GREEN = tuple(imm.Config.Get("virage debug color"))
            RED = tuple(imm.Config.Get("speed debug color"))
            BLUE = tuple(imm.Config.Get("acceleration debug color"))
            (x, y) = self.rect.center; length = 500
            coef = 1
            (vx, vy) = self.v; (ax, ay) = self.a
            pygame.draw.line(screen, RED, (x, y), (x+vx*coef, y+vy*coef), 2) # Représentation du vecteur vitesse
            pygame.draw.line(screen, BLUE, (x, y), (x+ax*coef, y+ay*coef), 2) # Représentation du vecteur accélération
            (ox, oy) = self.omega
            if(np.abs(self.lbda) >= 0.02): # Si la voiture est en mode virage
                pygame.draw.line(screen, GREEN, (ox-5, oy), (ox +5, oy), 2) # Représentation sur l'écran du centre du virage
                pygame.draw.line(screen, GREEN, (ox, oy-5), (ox, oy+5), 2)
                pygame.draw.line(screen, GREEN, self.omega, (x, y), 2)
                pygame.draw.line(screen, GREEN, self.omega, self.positionzero, 2)
                if(ox-self.r > 0 and oy-self.r > 0 and ox+self.r < screen.get_width() and oy+self.r < screen.get_height()):
                    rect = pygame.Rect(int(ox-self.r), int(oy-self.r), int(self.r*2), int(self.r*2))
                    if (self.lbda > 0):
                        begin = 180-self.teta; end = self.tetazero
                    else:
                        end = -self.teta; begin = -self.tetazero
                        end = -self.teta; begin = 180 + self.tetazero
                        end = end; begin = begin
                    #begin = np.fmod(np.deg2rad(begin), np.pi*2) #Modulo 2pi
                    #end = np.fmod(np.deg2rad(end), np.pi*2)
                    begin = (begin) % (360) #Modulo 2pi
                    end = (end) % (360)
                    pygame.draw.arc(screen, GREEN, rect, np.deg2rad(begin), np.deg2rad(end), 2)
    
    def GetSpeciesReward(self, divider = 1):
        return self.distance/divider #Reward divider
    
    def GetCarSensorsState(self, divider = 30):
        radars_detections = self.vehicle_outputs*[0]
        for i, radar in enumerate(self.sensors):
            radars_detections[i] = int(radar[1] / divider)
        return radars_detections
    
    def CheckCarRoadCollisions(self, map):
        for (x, y) in [self.rect.bottomleft, self.rect.topright, self.rect.bottomright, self.rect.topleft]: # On regarde les 4 coins de la voiture
            if x < map.mapPicture.get_width() and y < map.mapPicture.get_height() and x >= 0 and y >= 0:
                if not SameColor(map.mapPicture.get_at((x, y)), self.roadColor, 0.15):  #5%, on regarde la couleur de la route sur la carte
                    self.kill()
            else: self.kill()
    
    def UpdateCarSensors(self, map):
        cx = self.rect.centerx
        cy = self.rect.centery
        self.sensors = []
        for sensor_orientation in [0, -90, 90, -45, 45]: #0, -pi/2, pi/2, -pi/4, pi/4
            detection_size = 0
            x = int(cx + cos(360 - (self.teta + sensor_orientation)) * detection_size)
            y = int(cy + sin(360 - (self.teta + sensor_orientation)) * detection_size)

            # While We Don't Hit BORDER_COLOR AND length < 300 (just a max) -> go further and further
            if (x >= 0 and x < map.mapPicture.get_width()) and (y >= 0 and y < map.mapPicture.get_height()):
                while not SameColor(map.mapPicture.get_at((x, y)), self.roadColor, 0.05) and detection_size < 300:
                    detection_size += 1
                    x = int(cx + cos(360 - (self.teta + sensor_orientation)) * detection_size)
                    y = int(cy + sin(360 - (self.teta + sensor_orientation)) * detection_size)
                    if (x >= 0 and x < map.mapPicture.get_width()) and (y >= 0 and y < map.mapPicture.get_height()):
                        break

            # Calculate Distance To Border And Append To Radars List
            dist = int(np.sqrt((x - cx)**2 + (y - cy)**2))
            self.sensors.append([(x, y), dist])


def sin(teta):
    return np.sin(np.deg2rad(teta))
def cos(teta):
    return np.cos(np.deg2rad(teta))

def blitRotate(image, pos, originPos, angle):
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
    x = pos[0] - originPos[0] + min_box[0] - pivot_move[0]
    y = pos[1] - originPos[1] - max_box[1] + pivot_move[1]
    origin = (x, y)

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