import pygame
import numpy as np
import ctypes
import sys
import os
ctypes.windll.user32.SetProcessDPIAware()

# Les collisions entre voitures sont gérer par la class Sprite de pygame
# Les collisions avec les bords de la route sont gérer par des détections de couleurs des coins de la voiture
class WorldRenderer:
    RESSOURCES_PATH = "./Ressources/"
    MAPS_PATH = "Maps/"
    NEAT_PATH = "Neat/"
    CAMERA_SPEED = 20

    def __init__ (self, screenW, screenH):
        self.path = os.path.realpath(WorldRenderer.RESSOURCES_PATH + WorldRenderer.MAPS_PATH)
        self.screenW = screenW # La longueur de l'écran
        self.screenH = screenH # La largeur de l'écran
        self.sprites = [] # Possibilité de faire le rendu d'éléments sur la carte
        self.fov_width = screenW # Longueur du "rectangle caméra", longueur de la carte découpé et représentée sur l'écran
        self.fov_height = screenH # Largeur de la découpe de la carte.

    def LoadWorldFiles(self, mapName, mapDataXML = ""):
        self.mapName = mapName # Le nom de la carte (inutile)
        path = os.path.realpath(WorldRenderer.RESSOURCES_PATH + WorldRenderer.MAPS_PATH + self.mapName)
        self.raw_map = pygame.image.load(path).convert() # Convert() améliore les performances
        self.actual_map = pygame.image.load(path).convert()
        self.empty_background = pygame.Surface((self.screenW, self.screenH)) # Nécéssaire pour copier un morceau de la carte dessus
        self.cameraRect = pygame.Rect(0, 0, self.fov_width, self.fov_height) # Position de la camera SUR LA CARTE
        self.road_color = None

    def InitWorldRenderer(self, mapName):
        self.LoadWorldFiles(mapName) # Initialisation de la carte
        self.generation_font = pygame.font.SysFont("Arial", 60) # Initialisation des polices spéciales d'écritures
        self.alive_font = pygame.font.SysFont("Arial", 40)
        self.Sprites = pygame.sprite.Group([]) # Initialisation des Sprites
        self.previous_cameraRect = pygame.Rect(0, 0, 1, 1) # Pour détecter si le zoom a changé
        self.zoom_image = None # Initialisation

    def MapPygameCommands(self, events): # Commandes de déplacement de la caméra sur la carte
        # Pan-box moves up/down/left/right, Zooms with p and m
        keys = pygame.key.get_pressed()
        speed = WorldRenderer.CAMERA_SPEED
        if ( keys[pygame.K_UP] ): # Déplacement de la caméra vers le haut
            y = self.cameraRect.y - speed
            self.cameraRect.y = np.max(0, y) # On ne doit pas dépasser y = 0
        if ( keys[pygame.K_DOWN] ): # Déplacement de la caméra verrs le bas
            y = self.cameraRect.y + speed
            self.cameraRect.y = np.min(self.actual_map.get_width() - self.cameraRect.width - 1, y) # Le bas du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        if ( keys[pygame.K_LEFT] ): # Déplacement de la caméra vers la gauche
            x = self.cameraRect.x - speed
            self.cameraRect.x = np.max(0, x) # On ne doit pas dépasser x = 0
        if ( keys[pygame.K_RIGHT] ): # Déplacement de la caméra vers la droite
            x = self.cameraRect.x + speed
            self.cameraRect.y = np.min(self.actual_map.get_width() - self.cameraRect.width - 1, x) # Le bord droit du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        if ( keys[pygame.K_p]): # Zoom de la caméra sur la carte
            w  = self.cameraRect.width + speed
            h = self.cameraRect.height + speed
            self.cameraRect.width = np.min(w, self.screenW) # On ne zoom pas plus que : pixel(carte) = pixel(écran)
            self.cameraRect.height = np.min(h, self.screenH)
        if ( keys[pygame.K_m]): # Dé-zoom de la caméra sur la carte
            w = self.cameraRect.width  - speed
            h = self.cameraRect.height - speed
            self.cameraRect.width = np.max(w, 0) # On ne dézoome pas plus que la carte ne le permet
            self.cameraRect.height = np.max(h, 0)
        # self.fov_width  = min(self.fov_width, self.actual_map.get_width())
        # self.fov_height = min(self.fov_height, self.actual_map.get_height())

    def RenderMapOnScreen(self, screen):
        self.actual_map.blit(self.raw_map, (0, 0))
        for car in self.Sprites:
            car.RenderCarOnScreen(self, self.actual_map)
        screen.blit(self.empty_background, (0, 0))

    def UpdateMap(self, screen, dt):
        def Zoom(self):
            if ( self.cameraRect != self.previous_cameraRect ): # Si le zoom a changé (taille du rectangle FOV)
                if ( self.cameraRect.width != self.previous_cameraRect.width or self.cameraRect.height != self.previous_cameraRect.height ):
                    self.zoom_image = pygame.Surface( ( self.cameraRect.width, self.cameraRect.height ) ) # On change le zoom
                self.previous_cameraRect = self.cameraRect.copy() # on copie la taille du nouveau rectangle FOV (zoom)
        self.Sprites.update(self, screen, dt)#Update the sprites
        Zoom(self) # Zoom sur la carte si la TAILLE du rectangle camera sur la carte a changé
        window_size = (self.screenW, self.screenH) 
        pygame.transform.scale(self.zoom_image, window_size, self.empty_background) # On agrandit ou rétréssie zoomImage pour qu'il corresponde aux longeurs de l'écran, on rempli le reste avec du vide (emptyBackground)
        self.zoom_image.blit(self.actual_map, (0, 0), self.cameraRect) # On dessine la partie découpé par le rectangle FOV (cameraRect) de la carte sur zoomImage

    def InitUI(self): # A supprimer
        ### Pan-position
        background = pygame.Surface( ( WINDOW_WIDTH, WINDOW_HEIGHT ) )   # zoomed section is copied here
        zoom_image = None
        cameraRect    = pygame.Rect( 0, 0, PAN_BOX_WIDTH, PAN_BOX_HEIGHT )  # curent pan "cursor position"
        last_box   = pygame.Rect( 0, 0, 1, 1 )

import Packages.VehiculesComponents.VehicleCylindrique as vh
#CONSTANTS
DELTA_TIME_MAX_MILIS = 5 #the maximum of time between updates
import neat
class Application:
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
                #Render everything
                self.map.RenderMapOnScreen(self.screen)
                #Update everything
                dt = self.clock.get_time()
                if(dt >= DELTA_TIME_MAX_MILIS): #Lower than 1/DELTA_TIME_MAX fps
                    dt = DELTA_TIME_MAX_MILIS
                self.map.UpdateMap(self.screen, (float(dt)/1000)) #Conversion de dt en secondes
                #Update on the screen
                pygame.display.flip()
                #Making the clock ticking
                self.clock.tick(60) # 60 FPS

    def InitNeat(self, ressourcesPath, neatPath):
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

    def InitSimulation(self, genomes, config, masse, x, y, J, R, JR, GA, E, roadColor, ressourcePath,):
        self.genomes = genomes
        self.config = config
        for i, g in self.genomes:
            net = neat.nn.FeedForwardNetwork.create(g, self.config)
            self.nets.append(net)
            g.fitness = 0
            self.map.Sprites.add(vh.Vehicle(masse, x, y, J, R, JR, GA, E, roadColor, ressourcePath))

    def RunSimulation(self, MapName, InitSimulation, Update, masse, x, y, J, R, JR, GA, E, roadColor, ressourcePath, neatPath):
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
        self.InitNeat(ressourcePath, neatPath)
        def StartEvolution(genomes, config):
            self.InitPygame(MapName)
            self.InitSimulation(genomes, config, masse, x, y, J, R, JR, GA, E, roadColor, ressourcePath)
            def UpdateSimulation(app):
                Update(app)
                UpdateCarsInformations()
            self.RunPygame(InitSimulation, UpdateSimulation)
        self.population.run(StartEvolution, 1000)

    


class GameKernel:
    # Window size
    WINDOW_WIDTH    = 600
    WINDOW_HEIGHT   = 600
    WINDOW_SURFACE  = pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE
    image_filename  = None

    PAN_BOX_WIDTH   = 64/2
    PAN_BOX_HEIGHT  = 64/2
    PAN_STEP        = 5

    def InitPygame(self):
        def PythonInitialisation(self):
            # The first argument is either the image filename
            # or a "--help" request for help
            # Did we get any arguments?
            if ( len( sys.argv ) == 1 ):
                self.errorExit( "Give an image Filename as an argument" )
            else:
                # Get image filename as first argument
                for arg in sys.argv[1:]:
                    if ( arg in [ '-h', '--help', '-?', '/?' ] ):
                        self.errorExit( "Zooms an image, using arrow keys to pan\nGive an image Filename as an argument" )
                # Use the first argument as the image source
                image_filename = sys.argv[1] 
                sys.stdout.write( "Using [%s] as the image\n" % ( image_filename ) )
                return image_filename
        def PygameInitialisation(self):
            ### PyGame initialisation
            pygame.init()
            self.window = pygame.display.set_mode( ( WINDOW_WIDTH, WINDOW_HEIGHT ), WINDOW_SURFACE )
            pygame.display.set_caption("Image Pan")
            image_filename = "./AVehiclesMap/Ressources/Maps/ParisTopView/ParisTopView.png"
        image_filename = PythonInitialisation(self)
        PygameInitialisation(self)
    
    def LoadImage(self, image_filename):
        ### Can we load the user's image OK?
        try:
            base_image = pygame.image.load( image_filename ).convert()
            base_image_copy = pygame.image.load( image_filename ).convert()
        except:
            self.errorExit( "Failed to open [%s]" % ( image_filename ) )

    def Run(self):
        ### Main Loop
        clock = pygame.time.Clock()
        done = False
        while not done:

            # Handle user-input
            for event in pygame.event.get():
                if ( event.type == pygame.QUIT ):
                    done = True

            # Movement keys
            # Pan-box moves up/down/left/right, Zooms with + and -
            keys = pygame.key.get_pressed()
            if ( keys[pygame.K_UP] ):
                cameraRect.y -= PAN_STEP
            if ( keys[pygame.K_DOWN] ):
                cameraRect.y += PAN_STEP
            if ( keys[pygame.K_LEFT] ):
                cameraRect.x -= PAN_STEP
            if ( keys[pygame.K_RIGHT] ):
                cameraRect.x += PAN_STEP
            if ( keys[pygame.K_PLUS] or keys[pygame.K_EQUALS] ):
                cameraRect.width  += PAN_STEP
                cameraRect.height += PAN_STEP
                if ( cameraRect.width > WINDOW_WIDTH ):  # Ensure size is sane
                    cameraRect.width = WINDOW_WIDTH
                if ( cameraRect.height > WINDOW_HEIGHT ):
                    cameraRect.height = WINDOW_HEIGHT
            if ( keys[pygame.K_MINUS] or keys[pygame.K_6]):
                cameraRect.width  -= PAN_STEP
                cameraRect.height -= PAN_STEP
                if ( cameraRect.width < PAN_STEP ):  # Ensure size is sane
                    cameraRect.width = PAN_STEP
                if ( cameraRect.height < PAN_STEP ):
                    cameraRect.height = PAN_STEP

            # Ensure the pan-box stays within image
            PAN_BOX_WIDTH  = min( PAN_BOX_WIDTH, base_image.get_width() )
            PAN_BOX_HEIGHT = min( PAN_BOX_HEIGHT, base_image.get_height() )
            if ( cameraRect.x < 0 ):
                cameraRect.x = 0 
            elif ( cameraRect.x + cameraRect.width >= base_image.get_width() ):
                cameraRect.x = base_image.get_width() - cameraRect.width - 1
            if ( cameraRect.y < 0 ):
                cameraRect.y = 0 
            elif ( cameraRect.y + cameraRect.height >= base_image.get_height() ):
                cameraRect.y = base_image.get_height() - cameraRect.height - 1

            # Re-do the zoom, but only if the pan box has changed since last time
            if ( cameraRect != last_box ):
                # Create a new sub-image but only if the size changed
                # otherwise we can just re-use it
                if ( cameraRect.width != last_box.width or cameraRect.height != last_box.height ):
                    zoom_image = pygame.Surface( ( cameraRect.width, cameraRect.height ) )  
                
                zoom_image.blit( base_image, ( 0, 0 ), cameraRect )                  # copy base image
                window_size = ( WINDOW_WIDTH, WINDOW_HEIGHT )
                #test
                x += 1; y += 1
                base_image.blit(base_image_copy, (0, 0))
                base_image.blit(test, (x, y))
                #endtest
                pygame.transform.scale( zoom_image, window_size, background )     # scale into thebackground
                last_box = cameraRect.copy()                                         # copy current position

            window.blit( background, ( 0, 0 ) )
            pygame.display.flip()

            # Clamp FPS
            clock.tick_busy_loop(60)
        self.Quit()

    def Quit(self):
        pygame.quit()

    def errorExit( message, code=1 ):
        """ Write an error message to the console, then exit with an error code """
        sys.stderr.write( message + "\n" )
        sys.exit( code )