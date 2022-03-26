import ScreenRenderer
import Packages.VehiculesComponents.VehicleCylindrique
import pygame
from NeatVisualizer import *

"""
Liens qui m'ont été utiles :
1) Blit Rotate :
https://newbedev.com/python-how-to-rotate-a-sprite-pygame-code-example

2) Image Viewer :
https://stackoverflow.com/questions/63795236/best-way-to-create-image-viewer-and-allowing-zoom-drag-and-line-drawing-by-pyth
"""

#Fonction vide
def FonctionVide():
    return

#Fonctions d'initialisations.
Initialisation = FonctionVide
Update = FonctionVide

#Initialisation de l'expérience une
MapName = "Races/map mirom.png"
car = ""
fm = 0; frottements = 0; masse = 1000#kg
pneus = 0.9; A = 2*pneus*9.81 #pneus = mud = 0.9 sur route sèche
def VehiculeEnCarthesien():
    def Init(App):
        AjouteLaVoiture()
    
    def AjouteLaVoiture():
        global car
        x = App.map.start['x']; y = App.map.start['y']
        car = VehicleCarthesian.Vehicle(x, y, masse, A)
        App.map.Sprites.add(car)

    def DirectionsDuVehicules(app):
        keys = pygame.key.get_pressed()
        global car; global fm; global lbda; global frottements;
        (x, y) = (car.rect.centerx, car.rect.centery)
        if keys[pygame.K_q]:
            lbda += dlambda
        if keys[pygame.K_d]:
            lbda += -dlambda
        if keys[pygame.K_0]:
            fm = -20*1000
        if keys[pygame.K_1]:
            fm = 20*1000
        if keys[pygame.K_2]:
            fm = 60*1000
        if keys[pygame.K_3]:
            fm = 100*1000
        if keys[pygame.K_4]:
            fm = 100*1000
        if keys[pygame.K_5]:
            fm = 200*1000
        if keys[pygame.K_6]:
            fm = 500*1000
        if keys[pygame.K_7]:
            fm = 1*1000*1000
        if keys[pygame.K_8]:
            fm = 2*1000*1000
        if keys[pygame.K_9]:
            fm = 10*1000*1000
        if keys[pygame.K_MINUS]:
            fm = -20*1000
        if keys[pygame.K_r]:
            AjouteLaVoiture()
        car.SetAcceleration(fm, frottements, lbda)
    global Initialisation
    global MapName
    global Update
    Initialisation = Init
    MapName = "ParisTopView/ParisTopView.png"
    Update = DirectionsDuVehicules
    App = Application(1920, 1080)
    App.RunPygame(MapName, Initialisation, Update)

#Initialisation de l'expérience 2
masse = 1000#kg
E = 2.61#m l'empattement (distance entre les roues)
J = masse*(E*1.5)*1.78#kgm² le moment d'inertie de la voiture
R = 43*10**-2#Le rayon des roues (pneus + jentes)
JR = 21*R**2#kgm² le moment d'inertie d'une roue
couplefrein = 2*1000
lbda = 0; dlambda = 0.02
lastlbda = 0; lastFM = 0; lastCF = 0
def VehiculeCylindriqueMapBlanche2():
    def Init(App):
        AjouteLaVoiture()
    
    def AjouteLaVoiture():
        global car; global masse; global J; global JR; global E
        x = App.map.start['x']; y = App.map.start['y']
        car = VehicleCylindrique.Vehicle(x, y, masse, J, R, JR, E/2, E)
        car.v = (0, 0)
        car.a = (0, 0)
        global fm; fm = 0; global lbda; lbda = 0;
        App.map.Sprites.empty()
        App.map.Sprites.add(car)

    def DirectionsDuVehicules(app):
        keys = pygame.key.get_pressed()
        global car; global fm; global lbda; global couplefrein;
        if keys[pygame.K_q]:
            lbda += -dlambda
        if keys[pygame.K_d]:
            lbda += dlambda
        if keys[pygame.K_0]:
            fm = -2*1000
        if keys[pygame.K_1]:
            fm = 2*1000
        if keys[pygame.K_2]:
            fm = 6*1000
        if keys[pygame.K_3]:
            fm = 8*1000
        if keys[pygame.K_4]:
            fm = 10*1000
        if keys[pygame.K_5]:
            fm = 20*1000
        if keys[pygame.K_6]:
            fm = 50*1000
        if keys[pygame.K_7]:
            fm = 100*1000
        if keys[pygame.K_8]:
            fm = 200*1000
        if keys[pygame.K_9]:
            fm = 9*10*1000*1000
        if keys[pygame.K_MINUS]:
            fm = -20*1000
        if keys[pygame.K_r]:
            AjouteLaVoiture()
        global lastlbda; global lastFM; global lastCF
        if(lastlbda != lbda or lastFM != fm or lastCF != couplefrein):
            car.SetAcceleration(fm, couplefrein, int(lbda*100)/100)
        lastlbda = lbda; lastFM = fm; lastCF = couplefrein
    global Initialisation
    global MapName
    global Update
    Initialisation = Init
    MapName = "TestsEnvironement/whitemap.png"
    MapName = "ParisTopView/CircuitUn.png"
    Update = DirectionsDuVehicules
    App = Application(1920, 1080)
    App.InitPygame(MapName)
    App.RunPygame(Initialisation, Update)

def Experience3():
    def InitSimulation(app):
        return
    def UpdateApp(app):
        still_alive = 0
        for i, car in enumerate(app.map.Sprites):
            if car.alive():
                still_alive += 1
                if (app.genomes[i][1].fitness == None):
                    app.genomes[i][1].fitness = 0
                app.genomes[i][1].fitness += int(car.GetSpeciesReward(1))
    def Commandes(app):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_m]:
            plot_stats(stats, true, true)
        #Voir le génome de la meilleure voiture
        #Restart la simulation
        #Voir les progrès, la run en cours.
    global MapName; global Initialisation; global Update
    Update = UpdateApp
    Initialisation = InitSimulation
    MapName = "ParisTopView/CircuitUn.png"
    App = ScreenRenderer.Application(1920, 1080)
    class Color:
        def __init__(self, r, g, b):
            self.r = r
            self.g = g
            self.b = b
    roadColor = Color(121, 123, 110)
    App.RunSimulation(MapName, Initialisation, Update, masse, 960, 980, J, R, JR, E/2, E, roadColor, ScreenRenderer.WorldRenderer.RESSOURCES_PATH, ScreenRenderer.WorldRenderer.NEAT_PATH)

#VehiculeCylindriqueMapBlanche2()
Experience3()