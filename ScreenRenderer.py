import pygame
import numpy as np
import ctypes
import os
ctypes.windll.user32.SetProcessDPIAware()
import Packages.RessourcesManagers.FileManager as fm

# Les collisions entre voitures sont gérer par la class Sprite de pygame
# Les collisions avec les bords de la route sont gérer par des détections de couleurs des coins de la voiture
class WorldRenderer:
    def __init__ (self, screenW, screenH):
        self.path = os.path.realpath(fm.Config.Get("ressource path") + fm.Config.Get("maps path"))
        self.screenW = screenW # La longueur de l'écran
        self.screenH = screenH # La largeur de l'écran
        self.fov_width = screenW # Longueur du "rectangle caméra", longueur de la carte découpé et représentée sur l'écran
        self.fov_height = screenH # Largeur de la découpe de la carte.

    def LoadWorldFiles(self, mapName, jsonPath = ""):
        self.mapName = mapName # Le nom de la carte (inutile)
        path = os.path.realpath(fm.Config.Get("ressource path") + fm.Config.Get("maps path") + self.mapName)
        self.mapPicture = pygame.image.load(path).convert() # Convert() améliore les performances
        self.worldMap = pygame.image.load(path).convert()
        self.empty_background = pygame.Surface((self.screenW, self.screenH)) # Nécéssaire pour copier un morceau de la carte dessus
        self.cameraRect = pygame.Rect(0, 0, self.fov_width, self.fov_height) # Position de la camera SUR LA CARTE
        self.road_color = None

    def InitWorldRenderer(self, mapName):
        self.LoadWorldFiles(mapName) # Initialisation de la carte
        self.previous_cameraRect = pygame.Rect(0, 0, 1, 1) # Pour détecter si le zoom a changé
        self.camera = None # Initialisation

    def MapPygameCommands(self, events = None):  # Commandes de déplacement de la caméra sur la carte
        # Pan-box moves up/down/left/right, Zooms with p and m
        keys = pygame.key.get_pressed()
        speed =  fm.Config.Get("camera move speed") # Vitesse de la caméra (mouvement)
        zoomSpeed = fm.Config.Get("camera zoom speed")
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
        screen.blit(self.empty_background, (0, 0)) # Dans l'ordre mapPicture -> worldmap -> camera -> screen

    def UpdateMap(self, dt):
        def Zoom(self):
            if ( self.cameraRect != self.previous_cameraRect ): # Si le zoom a changé (taille du rectangle FOV)
                if ( self.cameraRect.width != self.previous_cameraRect.width or self.cameraRect.height != self.previous_cameraRect.height ):
                    self.camera = pygame.Surface( ( self.cameraRect.width, self.cameraRect.height ) ) # On change le zoom (la taille de zom)
                self.previous_cameraRect = self.cameraRect.copy() # on copie la taille du nouveau rectangle FOV (zoom)
        Zoom(self) # Zoom sur la carte si la TAILLE du rectangle camera sur la carte a changé
        screenSize = (self.screenW, self.screenH) # Dimension de l'écran
        pygame.transform.scale(self.camera, screenSize, self.empty_background) # On agrandit ou rétréssie zoomImage pour qu'il corresponde aux longeurs de l'écran, on rempli le reste avec du vide (emptyBackground)
        self.camera.blit(self.worldMap, (0, 0), self.cameraRect) # On dessine la partie découpé par le rectangle FOV (cameraRect) de la carte sur zoomImage

import Packages.WorldComponents.DriveMapGraph as graph
class HUD:
    def __init__(self, world, screen):
        self.worldMap = world
        self.screen = screen
        self.screenW = screen.get_width()
        self.screenH = screen.get_height()
        self.worldW = world.get_width()
        self.worldH = world.get_height()
        self.defaultFont = fm.Config.Get("default font")  # Initialisation de la police d'écriture par défaut
        self.mapPoints = []  # Points déssinés sur la carte
        self.mapSegments = [] # Lignes déssinées sur la carte
        self.mapSprites = pygame.sprite.Group([]) # Initialisation des Sprites

    def Update(self, dt):
        for sprite in self.mapSprites:
            sprite.Update(dt)

    def RenderScreenHud(self):
        return

    def RenderWorldHud(self):
        for sprite in self.srpites:
            sprite.Render(self.worldMap)
        for segment in self.mapSegments:
            HUD.RenderSegment(self.worldMap, segment.x1, segment.x2, segment.size, segment.color)
        for point in self.mapPoints:
            HUD.RenderPoint(self.worldMap, point.x1, point.x2, point.size, point.color)

    def AddWorldSprite(self, sprite):
        self.mapSprites.append(sprite)

    def AddWorldPoint(self, point, r, color):
        self.mapPoints.append((point, r, color))

    def AddWorldSegment(self, segment, r, color):
        self.mapPoints.append((segment.x1, segment.x2, r, color))

    def DefaultPointOnMap(self, point):
        r = fm.Config.Get("default point radius")
        color = fm.Config.Get("spline curve interpolation point color")
        self.AddWorldPoint(point, r, color)

    def DefaultSegmentOnMap(self, segment):
        r = fm.Config.Get("default line radius")
        color = fm.Config.Get("spline curve interpolation point color")
        self.AddWorldSegment(self.worldMap, segment.x1, segment.x2, r, color)
        self.AddWorldPoint(segment.x1, r, color)
        self.AddWorldPoint(segment.x2, r, color)

    def RenderPolygon(surface, polygon):
        prevP = polygon[0]
        HUD.RenderPoint(surface, prevP)
        for i in range(1, len(polygon)):
            HUD.RenderSegment(surface, graph.Segment(polygon[i], prevP))
            prevP = polygon[i]

    def RenderPoint(surface, point, r, color):
        (x, y) = point[0], point[1]
        BLACK = (0, 0, 0)  # Contours noirs
        dr = 3  # Coutours de 3 pixels
        pygame.draw.circle(surface, BLACK, (x, y), r+dr, 0)  # Contour noirs du point
        pygame.draw.circle(surface, color, (x, y), r, 0)  # Le point

    def RenderSegment(surface, point1, point2, size, color):
        pygame.draw.line(surface, color, point1, point2, size)

    def RenderText(surface, position, text, font):
        text = pygame.font.SysFont(font[0], font[1]).render('Pause', True, pygame.color.Color(font[2]))
        surface.blit(text, position)


def emptyFunction():  # Fonction vide, valeur par défaut
    return


class Application:
    def __init__(self, customInputs = emptyFunction, customRender = emptyFunction, customUpdate = emptyFunction, customInitialisation = emptyFunction):
        self.customInputs = customInputs
        self.customRender = customRender
        self.customUpdate = customUpdate
        self.customInitialisation = customInitialisation
        self.pauseFont = fm.Config.Get("pause font")  # Police d'écriture du mot "pause" à l'écran

    def Init(self, screenW, screenH, mapName):
        self.screenW = screenW
        self.screenH = screenH
        pygame.init()  # Intialisation pygame
        self.screen = pygame.display.set_mode((self.screenW, self.screenH))  # Initialisation de l'affichage
        self.clock = pygame.time.Clock()  # Initialisation de la clock
        self.paused = False
        self.map = WorldRenderer(self.screenW, self.screenH)  # Intialisation de la carte
        self.map.InitWorldRenderer(mapName)  # Initialisation du monde
        self.hud = HUD(self.map.worldMap, self.screen)  # Initialisation de l'HUD (interface)
        self.customInitialisation(self)

    def UpdateAll(self, dt):
        def ProcessPygameEvents(events):
            # Exit On Quit Event
            for event in events:
                if event.type == pygame.QUIT:  # arret de l'application
                    self.appRunning = False
                if event.type == pygame.KEYDOWN:  # Pause de l'application
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused
            self.map.MapPygameCommands(events)
            self.customInputs(events)
        events = pygame.event.get()  # Evenement d'inputs
        ProcessPygameEvents(events)  # Traitement des commandes
        self.map.UpdateMap(dt)  # Mise à jour du monde
        self.hud.Update(dt)
        self.customUpdate(self, dt)  # Traitements supplémentaires optionnels

    def RenderAll(self):
        self.map.RenderMapOnScreen(self.screen) # Rendu de la carte sur l'écran
        self.hud.RenderWorldHud()  # Rendu de "l'interface de la carte"
        self.customRender(self)  # rendus supplémentaires optionnels
        self.hud.RenderScreenHud()  # Rendu de l'interface écran
        pygame.display.flip()  # On rend tout sur l'écran

    def RenderPause(self):
        self.hud.renderText(self.screen, self.hud.center, "Pause", self.pauseFont)  # Rendu du mot "Pause" au centre de l'écran

    def Run(self):
        self.appRunning = True  # Booléen, application en cours d'éxécution
        MAX = fm.Config.Get("delta time max millis")  # Temps maximum entre les update()
        while(self.appRunning):
            if self.paused:  # Pause
                self.RenderPause()
            else:
                dt = self.clock.get_time()
                if(dt >= MAX): #Lower than 1/DELTA_TIME_MAX fps
                    dt = MAX
                self.UpdateAll(dt)
                self.RenderAll()
                self.clock.tick(60) # On garde le control du temps, 60 fps maximum


class SimpleApplication(Application):
    def __init__(self, mapName, customInputs = emptyFunction, customRender = emptyFunction, customUpdate = emptyFunction, customInitialisation = emptyFunction):
        Application.__init__(self, customInputs, customRender, customUpdate, customInitialisation)
        Application.Init(self, 1920, 1080, mapName)

    def Run(self):
        Application.Run(self)

import Packages.VehiculesComponents.VehicleCylindrique as vh
import neat
class NeatAplication(Application):
    def __init__(self):
        self.config = None
        self.genome = None
        Application.__init__(self)

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

    def Run(self, MapName, InitSimulation, Update, car, neatPath):
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