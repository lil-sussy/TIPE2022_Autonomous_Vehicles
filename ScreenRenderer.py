import pygame
import numpy as np
import ctypes
import sys
import os
ctypes.windll.user32.SetProcessDPIAware()
import Packages.MapComponents.ImageManager as imm
import Packages.VehiculesComponents.VehicleCylindrique as vh

# Les collisions entre voitures sont gérer par la class Sprite de pygame
# Les collisions avec les bords de la route sont gérer par des détections de couleurs des coins de la voiture
class WorldRenderer:
    def __init__ (self, screenW, screenH):
        self.path = os.path.realpath(imm.Config.Get("ressource path") + imm.Config.Get("maps path"))
        self.screenW = screenW # La longueur de l'écran
        self.screenH = screenH # La largeur de l'écran
        self.sprites = [] # Possibilité de faire le rendu d'éléments sur la carte
        self.fov_width = screenW # Longueur du "rectangle caméra", longueur de la carte découpé et représentée sur l'écran
        self.fov_height = screenH # Largeur de la découpe de la carte.

    def LoadWorldFiles(self, mapName, mapDataXML = ""):
        self.mapName = mapName # Le nom de la carte (inutile)
        path = os.path.realpath(imm.Config.Get("ressource path") + imm.Config.Get("maps path") + self.mapName)
        self.mapPicture = pygame.image.load(path).convert() # Convert() améliore les performances
        self.worldMap = pygame.image.load(path).convert()
        self.empty_background = pygame.Surface((self.screenW, self.screenH)) # Nécéssaire pour copier un morceau de la carte dessus
        self.cameraRect = pygame.Rect(0, 0, self.fov_width, self.fov_height) # Position de la camera SUR LA CARTE
        self.road_color = None

    def InitWorldRenderer(self, mapName):
        self.LoadWorldFiles(mapName) # Initialisation de la carte
        self.Sprites = pygame.sprite.Group([]) # Initialisation des Sprites
        self.previous_cameraRect = pygame.Rect(0, 0, 1, 1) # Pour détecter si le zoom a changé
        self.camera = None # Initialisation

    def MapPygameCommands(self, events = None):  # Commandes de déplacement de la caméra sur la carte
        # Pan-box moves up/down/left/right, Zooms with p and m
        keys = pygame.key.get_pressed()
        speed =  imm.Config.Get("camera move speed") # Vitesse de la caméra (mouvement)
        zoomSpeed = imm.Config.Get("camera zoom speed")
        if ( keys[pygame.K_UP] ): # Déplacement de la caméra vers le haut
            y = self.cameraRect.y - speed
            self.cameraRect.y = max(0, y) # On ne doit pas dépasser y = 0
        
        if ( keys[pygame.K_DOWN] ): # Déplacement de la caméra verrs le bas
            y = self.cameraRect.y + speed
            bottomBorder = self.worldMap.get_height() - self.cameraRect.height - 1
            self.cameraRect.y = min(bottomBorder, y) # Le bas du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        
        if ( keys[pygame.K_LEFT] ): # Déplacement de la caméra vers la gauche
            x = self.cameraRect.x - speed
            self.cameraRect.x = max(0, x) # On ne doit pas dépasser x = 0
        
        if ( keys[pygame.K_RIGHT] ): # Déplacement de la caméra vers la droite
            x = self.cameraRect.x + speed
            rightBorder = self.worldMap.get_width() - self.cameraRect.width - 1
            self.cameraRect.x = min(rightBorder, x) # Le bord droit du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        
        if ( keys[pygame.K_p]): # Zoom de la caméra sur la carte
            w  = self.cameraRect.width *(1 + zoomSpeed)
            h = self.cameraRect.height *(1 + zoomSpeed)
            self.cameraRect.width = min(w, self.screenW) # On ne zoom pas plus que : pixel(carte) = pixel(écran)
            self.cameraRect.height = min(h, self.screenH)
        
        if ( keys[pygame.K_m]): # Dé-zoom de la caméra sur la carte
            w = int(self.cameraRect.width * (1 - zoomSpeed))
            h = int(self.cameraRect.height * (1 - zoomSpeed))
            self.cameraRect.width = max(w, 0) # On ne dézoome pas plus que la carte ne le permet
            self.cameraRect.height = max(h, 0)
        # self.fov_width  = min(self.fov_width, self.actual_map.get_width())
        # self.fov_height = min(self.fov_height, self.actual_map.get_height())

    def RenderMapOnScreen(self, screen):  # Rendu des informations additionelles des sprites
        self.worldMap.blit(self.mapPicture, (0, 0)) # On efface les anciens dessin sur la carte
        for sprite in self.Sprites:
            if isinstance(sprite, vh.Vehicle):
                sprite.RenderCarOnScreen(self, self.worldMap) # On dessine sur la CARTE (pas sur l'écran)
            else:
                sprite.Render(self)
        screen.blit(self.empty_background, (0, 0)) # Dans l'ordre mapPicture -> worldmap -> camera -> screen

    def UpdateMap(self, screen, dt):
        def Zoom(self):
            if ( self.cameraRect != self.previous_cameraRect ): # Si le zoom a changé (taille du rectangle FOV)
                if ( self.cameraRect.width != self.previous_cameraRect.width or self.cameraRect.height != self.previous_cameraRect.height ):
                    self.camera = pygame.Surface( ( self.cameraRect.width, self.cameraRect.height ) ) # On change le zoom (la taille de zom)
                self.previous_cameraRect = self.cameraRect.copy() # on copie la taille du nouveau rectangle FOV (zoom)
        self.Sprites.update(self, screen, dt)#Update the sprites
        Zoom(self) # Zoom sur la carte si la TAILLE du rectangle camera sur la carte a changé
        screenSize = (self.screenW, self.screenH) # Dimension de l'écran
        pygame.transform.scale(self.camera, screenSize, self.empty_background) # On agrandit ou rétréssie zoomImage pour qu'il corresponde aux longeurs de l'écran, on rempli le reste avec du vide (emptyBackground)
        self.camera.blit(self.worldMap, (0, 0), self.cameraRect) # On dessine la partie découpé par le rectangle FOV (cameraRect) de la carte sur zoomImage

class HUD:
    def DrawPoint(surface, x, y, r, color):
        BLACK = (0, 0, 0)  # Contours noirs
        dr = 3  # Coutours de 3 pixels
        pygame.draw.circle(surface, BLACK, (x, y), r+dr, 0)  # Contour noirs du point
        pygame.draw.circle(surface, color, (x, y), r, 0)  # Le point
    def DrawLine(surface, point1, point2, size, color):
        BLACK = (0, 0, 0)  # Contours de couleur noirs
        dsize = 8  # Coutours de 3 pixels
        # pygame.draw.line(surface, BLACK, point1, point2, size+dsize)  # Contour noirs du trait
        pygame.draw.line(surface, color, point1, point2, size)


class ScreenHud:
    def __init__(self, screen):
        self.generation_font = pygame.font.SysFont("Arial", 60)  # Initialisation des polices spéciales d'écritures
        self.alive_font = pygame.font.SysFont("Arial", 40)
        self.screen = screen


class PygameApplication:
    def __init__(self):
        return


import Packages.VehiculesComponents.VehicleCylindrique as vh
#CONSTANTS
DELTA_TIME_MAX_MILIS = 5 #the maximum of time between updates
import neat
class NeatApplication:
    def __init__(self, screenW, screenH):
        self.screenW = screenW
        self.screenH = screenH
        self.genomes = None #to be defined later
        self.config = None

    def InitPygame(self, mapName):
        #Init Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.screenW, self.screenH))
        self.clock = pygame.time.Clock()
        #Init
        self.paused = False
        self.map = WorldRenderer(self.screenW, self.screenH)
        self.map.InitWorldRenderer(mapName)

    def ProcessPygameEvents(self, events):
        # Exit On Quit Event
        for event in events:
            if event.type == pygame.QUIT: self.appRunning = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
        self.map.MapPygameCommands(events)

    def RunPygame(self, initialisationTIPE, Update):
        initialisationTIPE(self)
        self.appRunning = True
        #Texts to be rendered:
        SPauseText = pygame.font.SysFont('Consolas', 32).render('Pause', True, pygame.color.Color('Black'))
        while(self.appRunning):
            self.ProcessPygameEvents(pygame.event.get())
            Update(self)
            if self.paused:
                #Display Paused Text
                self.screen.blit(SPauseText, (100, 100))
            else:
                #Update everything
                dt = self.clock.get_time()
                if(dt >= DELTA_TIME_MAX_MILIS): #Lower than 1/DELTA_TIME_MAX fps
                    dt = DELTA_TIME_MAX_MILIS
                self.map.UpdateMap(self.screen, (float(dt)/1000)) #Conversion de dt en secondes
                #Update on the screen
                pygame.display.flip()
                #Render everything
                self.map.RenderMapOnScreen(self.screen) # Rendu de la carte sur l'écran
                self.clock.tick(60) # On garde le control du temps, 60 fps maximum

    def InNeat(self, ressourcesPath, neatPath):
        #Setup Neat
        self.cars = []
        self.nets = []
        self.still_alive = 0
        self.CurrentGeneration = 0
        #Load Config
        config_path = ressourcesPath+neatPath+"config.txt"
        config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction, neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                    config_path)
        # Create Population And Add Reporters
        self.population = neat.Population(config)
        self.population.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        self.population.add_reporter(stats)

    def InitSimulation(self, genomes, config, car):
        self.genomes = genomes
        self.config = config
        for i, g in self.genomes:
            net = neat.nn.FeedForwardNetwork.create(g, self.config)
            self.nets.append(net)
            g.fitness = 0
            self.map.Sprites.add(vh.Vehicle.Copy(car))

    def RunSimulation(self, MapName, InitSimulation, Update, car, neatPath):
        dlbda = 0.2 # Constante
        dc = 600 # Constante
        def UpdateCarsInformations():
            for i, car in enumerate(self.map.Sprites):
                #Making a decision with the given inputs
                output = self.nets[i].activate(car.GetCarSensorsState())
                choice = output.index(max(output))
                lbda = car.lbda
                coupleM = car.coupleM
                coupleF = car.coupleF
                if choice == 0: #The car chose to turn left
                    lbda += dlbda
                elif choice == 1: 
                    lbda -= dlbda # Right
                elif choice == 2: #Si la voiture ne choisit pas de freiner, on accélère
                    if (coupleM - dc > 0):
                        coupleM -= dc
                        coupleF= 0
                    else:
                        coupleF -= dc
                else:
                    coupleM += dc
                    coupleF = 0
                car.SetCarAcceleration(coupleM, coupleF, lbda)
        self.InNeat(car.ressourcePath, neatPath) # Attention à car.ressourcepath !!
        def StartEvolution(genomes, config):
            self.InitPygame(MapName)
            self.InitSimulation(genomes, config, car)
            def UpdateSimulation(app):
                Update(app)
                UpdateCarsInformations()
            self.RunPygame(InitSimulation, UpdateSimulation)
        self.population.run(StartEvolution, 1000)