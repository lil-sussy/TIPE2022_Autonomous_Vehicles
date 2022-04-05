import pygame
import Packages.App.Graphics.GraphicsEngine as ge
import Packages.App.PygameComponents.GameConsoleMaster.pygame_console.game_console as console
import Packages.RessourcesManagers.FileManager as fm

def emptyFunction(app):  # Fonction vide, valeur par défaut
    return

from threading import Thread
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
        self.world = ge.WorldRenderer(self.screenW, self.screenH)  # Intialisation de la carte
        self.world.InitWorldRenderer(mapName)  # Initialisation du monde
        self.hud = ge.HUD(self.world.worldMap, self.screen)  # Initialisation de l'HUD (interface)
        self.worldHud = ge.ClickableWorldHud(self.world)  # Initialisation des items clickables sur la carte
        self.keyboard = Keyboard(self)  # Initialisation des raccourcis clavier
        self.console = console.Console(self, screenW)
        self.customInitialisation(self)

    def ProcessNewEvents(self, dt):
        events = pygame.event.get()  # Evenement d'inputs
        keys = pygame.key.get_pressed()
        # Exit On Quit Event
        for event in events:
            if event.type == pygame.QUIT:  # arret de l'application
                self.exit = True
            if event.type == pygame.KEYDOWN:  # Pause de l'application
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
        self.world.MapPygameCommands(keys, dt)
        self.keyboard.Perform(keys)
        self.customInputs(keys)

    def UpdateAll(self, dt):
        self.ProcessNewEvents(dt)
        self.world.UpdateMap(dt)  # Mise à jour du monde
        self.hud.Update(dt)
        self.worldHud.Update(dt)
        self.customUpdate(self, dt)  # Traitements supplémentaires optionnels

    def RenderAll(self):
        pygame.display.flip()  # On rend tout sur l'écran
        self.world.ClearMap()  # On nettoie la carte des anciens éléments
        self.hud.RenderWorldHud()  # Rendu de "l'interface de la carte"
        self.worldHud.Render()  # Rendu des items clickables sur la carte
        self.customRender(self)  # rendus supplémentaires optionnels
        if self.console.enabled:
            self.console.Render()  # Rendu de la console à l'écran
        self.hud.RenderScreenHud()  # Rendu de l'interface écran
        self.world.RenderMapOnScreen(self.screen) # Rendu de la carte sur l'écran

    def RenderPause(self):
        self.hud.renderText(self.screen, self.hud.center, "Pause", self.pauseFont)  # Rendu du mot "Pause" au centre de l'écran

    def Run(self):
        self.exit = False  # Booléen, application en cours d'éxécution
        MAX = fm.Config.Get("delta time max millis")  # Temps maximum entre les update()
        def RenderApp():
            while(not self.exit):
                if self.paused:  # Pause
                    self.RenderPause()
                else:
                    self.RenderAll()
        def UpdateApp():
            while(not self.exit):
                if self.paused:  # Pause
                    self.paused = self.paused
                else:
                    dt = self.clock.get_time()
                    if(dt >= MAX): #Lower than 1/DELTA_TIME_MAX fps
                        dt = MAX
                    self.UpdateAll(dt)
                    self.clock.tick(60) # On garde le control du temps, 60 fps maximum
        self.renderThread = Thread(target=RenderApp)
        self.renderThread.start()
        UpdateApp()


class SimpleApplication(Application):
    def __init__(self, mapName, customInputs = emptyFunction, customRender = emptyFunction, customUpdate = emptyFunction, customInitialisation = emptyFunction):
        Application.__init__(self, customInputs, customRender, customUpdate, customInitialisation)
        Application.Init(self, 1920, 1080, mapName)

    def Run(self):
        Application.Run(self)

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
            self.world.Sprites.add(vh.Vehicle.Copy(car))

    def Run(self, MapName, InitSimulation, Update, car, neatPath):
        dlbda = 0.2 # Constante
        dc = 600 # Constante
        def UpdateCarsInformations():
            for i, car in enumerate(self.world.Sprites):
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





class Keyboard:
    def __init__(self, app):
        self.app = app

    def Perform(self, keys):
        if keys[pygame.K_LCTRL] and keys[pygame.K_p]:  # Ctrl + p, on cache la console
            self.app.console.enabled = True