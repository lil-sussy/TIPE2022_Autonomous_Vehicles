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

#Initialisation de l'expérience 3
masse = 1000#kg
E = 2.61#m l'empattement (distance entre les roues)
J = masse*(E*1.5)*1.78#kgm² le moment d'inertie de la voiture
R = 43*10**-2#Le rayon des roues (pneus + jentes)
JR = 21*R**2#kgm² le moment d'inertie d'une roue
couplefrein = 2*1000
lbda = 0; dlambda = 0.02
lastlbda = 0; lastFM = 0; lastCF = 0

import Packages.MapComponents.ImageManager as imm
import Packages.VehiculesComponents.VehicleCylindrique as vh
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
    global Initialisation; global Update
    Update = UpdateApp # Appelé à chaque update de pygame
    Initialisation = InitSimulation # Appelé lors de l'initialisation
    MapName = "ParisTopView/CircuitUn.png"
    App = ScreenRenderer.NeatApplication(1920, 1080)
    class Color:
        def __init__(self, r, g, b):
            self.r = r
            self.g = g
            self.b = b
    roadColor = Color(121, 123, 110)
    debugLevel = 3 # 3 niveau de debugage, information supplémentaires du sprite
    basicCar = vh.Vehicle(930, 1000, masse, J, R, JR, E/2, E, roadColor, imm.Config.Get("ressource path"), debugLevel)
    App.RunSimulation(MapName, Initialisation, Update, basicCar, imm.Config.Get("neat path"))

#VehiculeCylindriqueMapBlanche2()
Experience3()