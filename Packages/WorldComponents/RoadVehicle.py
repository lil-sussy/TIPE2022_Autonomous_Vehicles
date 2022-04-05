import pygame
import numpy as np
import Packages.App.PygameComponents.Component as comp
import Packages.WorldComponents.WorldRoads as cv
import Packages.RessourcesManagers.FileManager as fm


def cos(theta):
    return np.cos(np.deg2rad(theta))

def sin(theta):
    return np.sin(np.deg2rad(theta))

def Carthesian2Polar(position):
    def Angle(pos, hypothenuse):
            if pos[1] <= 0:
                return -np.rad2deg(np.arccos(pos[0] / hypothenuse))
            else:
                return np.rad2deg(np.arccos(pos[0] / hypothenuse))
    polarDistance = cv.Segment(position, [0, 0]).Length()
    polarAngle = Angle(position, polarDistance)
    return polarDistance, polarAngle

def Polar2Carthesian(theta, distance):
    x = distance *cos(theta)
    y = distance *sin(theta)
    return x, y


class Vehicle(comp.Component):
    ID = 0
    def __init__(self, position, mass, wheelbase, width, wheelradius, debuglevel = 3):
        def Precalculations():  # calculés en amont pour économiser du temps :
            self.computations = {}
            staticFrictionCoef = fm.Config.Get("static friction coefficient")
            self.computations["wheight"] = staticFrictionCoef *9.81 *mass  # Poid du véhicule *muStatic
            self.computations["wheel force"] = 0  # Force des roues en N
            self.computations["wheel force sin"] = 0  
            self.computations["wheel force cos"] = 0  
            self.computations["wheel force sin squared"] = 0  # Inutile
            self.computations["wheel force cos squared"] = 0
            self.computations["1 + cos(lambdaa)"] = 0
        comp.Component.__init__(self)
        self.mass = mass
        self.wheelbase = wheelbase
        self.width = width
        self.wheelradius = wheelradius
        self.enginecouple = 0  # Le couple moteur en N.m
        self.wheelangle = 0  # L'angle entre les roues avant et l'axe de direction du véhicule
        self.acceleration = 0 # L'acceleration du véhicule, quand elle n'effectue pas de virage
        self.speed = 0  # La vitesse du véhicule, quand elle n'esffectue pas de virage
        self.theta = 0 # L'angle entre la direction du véhicule et l'absisse de la carte (en degré)
        self.thetap = 0  # Vitesse angulaire dans le plan de la carte
        self.thetapp = 0  # Accélération angulaire dans le plan de la carte
        self.distance = 0  # Distance à l'origine de la carte
        self.distancep = 0  # Vitesse raidial du véhicule
        self.distancepp = 0  # Accélération radial du véhicule
        self.position = position  # Coordonées du centre du véhicule dans le plan de la carte
        self.distance, self.theta = Carthesian2Polar(self.position)
        self.viragecenter = position  # Coordonées du centre du virage en cours du vhéhicule
        self.isPerformingTurn = False  # Booléen, est ce que la voiture effectue un virage
        # Elements pygame
        self.spriteposition = self.position
        self.debuglevel = debuglevel
        path = fm.Config.Get("vehicle sprite path")
        self.rawimage = pygame.image.load(path)  # L'image du véhicule sur la carte, indéformée
        self.spriteimage = pygame.image.load(path)  # L'image du véhicule sur la carte
        imageWidth = self.spriteimage.get_rect().width
        imageHeight = self.spriteimage.get_rect().height
        x, y = (self.position[0] - imageWidth/2, self.position[1] - imageHeight/2)
        self.rect = pygame.Rect(x, y, imageWidth, imageHeight)  # Hitbox du véhicule sur la carte
        Vehicle.ID += 1
        self.id = Vehicle.ID  # identifiant unique de la voiture
        Precalculations()

    def ChangeOrders(self, wheelangle, enginecouple):
        def Precalculations():  # calculés en amont pour améliorer les performances :
            dict = self.computations
            dict["wheel force"] = enginecouple/self.wheelradius
            dict["wheel force squared"] = dict["wheel force"]**2
            dict["wheel force sin"] = dict["wheel force"] *sin(wheelangle)
            dict["wheel force cos"] = dict["wheel force"] *cos(wheelangle)
            dict["wheel force sin squared"] = dict["wheel force sin"]**2  # Inutile
            dict["wheel force cos squared"] = dict["wheel force cos"]**2
            dict["1 + cos(lambdaa)"] = 1 + cos(wheelangle)
        Precalculations()  # On précalcule les valeurs pour économiser les performances de la fonction Update
        OFFSET = fm.Config.Get("wheel angle offset")  # Annulation de l'angle s'il est trop petit
        if np.abs(self.wheelangle) < OFFSET:
            self.wheelangle = 0
        if self.isPerformingTurn and self.wheelangle == 0:  # Lorsque le véhicule arrête de tourner
            self.wheelangle = 0
            self.isPerformingTurn = False  # on effectue plus de virage
            self.acceleration = self.thetapp *self.distance  # Continuité de l'accélération et de la vitesse
            self.speed = self.thetap *self.distance
            self.distance += - cv.Segment(self.viragecenter, [0, 0]).Length()  # Changement de repère
        if wheelangle != 0:
            cotan = cos(wheelangle)/sin(wheelangle)
            self.distance = np.abs(cotan *self.wheelbase/2) + self.width/2  # Calcul de la distance du centre du virage : |cotan(lambdaa)*E/2| + F/2
            if wheelangle > 0:  # Le véhicule tourne à droite
                centerX, centerY = Polar2Carthesian(self.theta + 90, self.distance)  # Le point d'une distance {virageradius} à partir du centre du véhicule à la droite de la direction du véhicule
            elif wheelangle < 0:  # Le véhicule tourne à gauche
                centerX, centerY = Polar2Carthesian(self.theta - 90, self.distance)  # Le point d'une distance {virageradius} à partir du centre du véhicule à la gauche de la direction du véhicule
            self.viragecenter = [centerX + self.position[0], centerY + self.position[1]]  # Calculs des coordonées du centre du virage
            if (not self.isPerformingTurn) and wheelangle != 0:  # Le véhicule ammorce un virage
                self.isPerformingTurn = True
                self.distancep = 0  # Conditions de Continuités
                self.thetap = self.speed/self.distance  # Continuité de l'accélération et de la vitesse
                self.thetapp = self.acceleration/self.distance
                self.distancepp = self.distance *self.thetap**2
        self.enginecouple = enginecouple  # Changement de vitesse (négatif pour reculer)
        self.wheelangle = wheelangle

    def Update(self, app, dt):
        def ComputeDifferentialEquations():
            def SlidingCOndition(dict, inertia):
                rearwheelForce = np.sqrt(dict["wheel force squared"] + inertia**2)
                frontwheelForce = np.sqrt(dict["wheel force cos squared"] + (inertia + dict["wheel force sin"])**2)
                return max(rearwheelForce, frontwheelForce) <= dict["wheight"]
            dict = self.computations
            if not self.isPerformingTurn:
                self.acceleration = dict["wheel force"] /self.mass
                self.speed += self.acceleration *dt
                speedX, speedY = Polar2Carthesian(self.theta, self.speed)
                self.position = cv.Segment(self.position, [speedX, speedY]).LinearTransformation(1, dt)
            else:
                inertia = self.mass *self.distance *self.theta**2  # Force centrifuge du véhicule
                slidingCondition = SlidingCOndition(self.computations, inertia)  # Condition de glissement
                if slidingCondition:
                    thetapp = dict["wheight"]/(self.mass *self.distance)  # Calcul de l'accélération angulaire du véhicule
                    thetapp = thetapp*dict["1 + cos(lambdaa)"]
                    thetapp += -2 *(self.distancep/self.distance) *self.thetap
                    self.thetapp = thetapp
                    distancepp = -dict["wheight"]/self.mass  # Calcul de l'accélération radial du véhicule, grâce aux équations différentielles
                    distancepp += 2 *self.distance *self.thetap**2
                    self.distancepp = distancepp
                    self.thetap += self.thetapp *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.theta += self.thetap *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.distancep += self.distancepp *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.distance += self.distancep *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                else:
                    thetapp = dict["wheel force"]/(self.mass *self.distance)  # Calcul de l'accélération angulaire du véhicule
                    thetapp = thetapp*dict["1 + cos(lambdaa)"]
                    thetapp += -2 *(self.distancep/self.distance) *self.thetap
                    self.thetapp = thetapp
                    distancepp = -dict["wheel force sin"]/self.mass  # Calcul de l'accélération radial du véhicule, grâce aux équations différentielles
                    distancepp += 2 *self.distance *self.thetap**2
                    self.distancepp = distancepp
                    self.thetap += self.thetapp *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.theta += self.thetap *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.distancep += self.distancepp *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                    self.distance += self.distancep *dt  # Méthode de résolution d'équation différentiel ittérative d'Euler
                positionX, positionY = Polar2Carthesian(self.theta, self.distance)  # Calcul de la position du véhicule
                positionX += self.viragecenter[0]  # Changement de repère
                positionY += self.viragecenter[1]
                self.position = [positionX, positionY]
        def BlitRotate(image, rotationCenter, imageCenter, angle):
            # calcaulate the axis aligned bounding box of the rotated image
            w, h       = image.get_size()
            box        = [pygame.math.Vector2(positions) for positions in [(0, 0), (w, 0), (w, -h), (0, -h)]]
            box_rotate = [vectors.rotate(angle) for vectors in box]
            min_box    = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
            max_box    = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])
            # calculate the translation of the pivot 
            pivot        = pygame.math.Vector2(imageCenter[0], -imageCenter[1])
            pivot_rotate = pivot.rotate(angle)
            pivot_move   = pivot_rotate - pivot
            # calculate the upper left origin of the rotated image
            x = rotationCenter[0] - imageCenter[0] + min_box[0] - pivot_move[0]
            y = rotationCenter[1] - imageCenter[1] - max_box[1] + pivot_move[1]
            origin = (x, y)
            # get a rotated image
            rotated_image = pygame.transform.rotate(image, angle)
            return rotated_image, origin
        comp.Component.Update(self, app, dt)
        dt = dt/1000  # Conversion en secondes
        ComputeDifferentialEquations()
        self.rect.center = (int(self.position[0]), int(self.position[1]))  # On déplace la hitbox du véhicule
        rotationCenter = [self.position[0] + self.rect.w//2, self.position[1] + self.rect.h//2]  # Centre de rotation du sprite (centre du véhicule)
        imageCenter = [self.spriteimage.get_width()/2, self.spriteimage.get_height()/2]  # Centre de l'image
        self.spriteimage, self.spriteposition = BlitRotate(self.rawimage, rotationCenter, imageCenter, self.theta + 90)

    def Render(self, world):
        comp.Component.Render(self, world)
        level = self.debuglevel # Niveau de débuggage, niveau de de détails supplémentaires
        if level >= 0: # Niveau 0 est le niveau par défaut, niveau -1 : il est possible de ne pas faire le rendu des voitures
            world.worldMap.blit(self.spriteimage, self.spriteposition) # Rendu du sprite de la voiture
        if level >= 1:
            pygame.draw.rect(world.worldMap, (0, 255, 0), (self.rect.topleft, self.rect.size), 2) # Dessin de la HitBox de la voiture
        if level >= 2: # Niveau max : niveau 3
            GREEN = tuple(fm.Config.Get("virage debug color"))
            RED = tuple(fm.Config.Get("speed debug color"))
            BLUE = tuple(fm.Config.Get("acceleration debug color"))
            (x, y) = self.rect.center
            coef = 1
            # (vx, vy) = self.speed; (ax, ay) = self.acceleration
            # pygame.draw.line(world.worldMap, RED, (x, y), (x+vx*coef, y+vy*coef), 2) # Représentation du vecteur vitesse
            # pygame.draw.line(world.worldMap, BLUE, (x, y), (x+ax*coef, y+ay*coef), 2) # Représentation du vecteur accélération
            (ox, oy) = tuple(self.viragecenter)
            if(np.abs(self.wheelangle) >= 0.02): # Si la voiture est en mode virage
                pygame.draw.line(world.worldMap, GREEN, (ox-5, oy), (ox +5, oy), 2) # Représentation sur l'écran du centre du virage
                pygame.draw.line(world.worldMap, GREEN, (ox, oy-5), (ox, oy+5), 2)
                pygame.draw.line(world.worldMap, GREEN, tuple(self.viragecenter), (x, y), 2)
                # pygame.draw.line(world.worldMap, GREEN, tuple(self.viragecenter), self.positionzero, 2)
                if(ox-self.distance > 0 and oy-self.distance > 0 and ox+self.distance < world.width and oy+self.distance < world.height):
                    rect = pygame.Rect(int(ox-self.distance), int(oy-self.distance), int(self.distance*2), int(self.distance*2))
                    # if (self.wheelangle > 0):
                    #     begin = 180-self.theta; end = self.tetazero
                    # else:
                    #     end = -self.theta; begin = -self.tetazero
                    #     end = -self.theta; begin = 180 + self.tetazero
                    #     end = end; begin = begin
                    # begin = (begin) % (360) #Modulo 2pi
                    # end = (end) % (360)
                    # pygame.draw.arc(world.worldMap, GREEN, rect, np.deg2rad(begin), np.deg2rad(end), 2)
                    

import pygame
import numpy as np
import Packages.RessourcesManagers.FileManager as fm

#(x, y) _______________ x (le référentiel de l'écran)
#      |
#      |
#      y
id = 0
class OLDVehicle(pygame.sprite.Sprite):
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
            GREEN = tuple(fm.Config.Get("virage debug color"))
            RED = tuple(fm.Config.Get("speed debug color"))
            BLUE = tuple(fm.Config.Get("acceleration debug color"))
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