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

import Packages.RessourcesManagers.FileManager as fm
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
    basicCar = vh.Vehicle(930, 1000, masse, J, R, JR, E/2, E, roadColor, fm.Config.Get("ressource path"), debugLevel)
    App.RunSimulation(MapName, Initialisation, Update, basicCar, fm.Config.Get("neat path"))

#VehiculeCylindriqueMapBlanche2()


import Packages.RessourcesManagers.FileManager as fm

import numpy as np

import pygame
import Packages.RessourcesManagers.FileManager as fm

# path = "C:\\Users\\nayke\\OneDrive\\devlab\\TIPE 2022\\AVehiclesMap\\Ressources\\Maps\\ParisTopView\\CurvesTest1\\"
# curve1BP = np.asarray(Image.open(path + "Highways\\Curve1.png"))
# w = len(curve1BP); h = len(curve1BP[0])

import Packages.WorldComponents.DriveMapGraph as Graph
def GenerateCurve(bppath):
    progression = fm.ProgressBar("Génération de la courbe...") # Bar de progression
    blueprint = fm.OpenImage(bppath)[0]
    curve = Graph.BasicCurve.BluePrintToCurve(blueprint, progression)
    progression.Done()
    return curve

import ScreenRenderer
import Packages.WorldComponents.Cython.EuclidianCurves as Curves
def GenerateMap(pathCurve1):
    progression = fm.ProgressBar("Génération de la carte...") # Bar de progression
    curve1 = GenerateCurve(pathCurve1)
    ignored, w, h = fm.OpenImage(pathCurve1)
    dr = 30; R = 650
    background = fm.AsUint8(w*[h*[[0, 0, 0, 255]]]) #taille w*h*4
    curves = fm.AsUint16([curve1]) # Liste de courbes
    result = Curves.DrawRoadsOnMap(curves, background, dr/3, dr, R//2)
    progression.Done()
    fm.ShowImage(result)

def GenerateGraph(pathCurve1):
    progression = fm.ProgressBar("Génération du graphe...") # Bare de progression
    curve1 = GenerateCurve(pathCurve1)
    road, w, h = fm.OpenImage(pathCurve1)
    chunkNumber = 16
    graph = Graph.Graph.GenerateGraph([curve1], chunkNumber, w, h, progression)
    print(graph)

def BezierCurvef():
    def initialisation(app):
        return
    def update(app):
        return
    point1 = [250, 500]
    point2 = [750, 500]
    point3 = [400, 100]
    point4 = [600, 700]
    debugLevel = 3
    quality = 1 # 100%
    parameters = [0.7]
    curve1 = Graph.BezierCurve([point1, point3, point4, point2], quality, parameters, debugLevel)
    point = curve1.Interpolate(0.7)
    curve2 = Graph.BezierCurve([point1, point3, point4, point2, point], quality, [], debugLevel)
    MapName = "ParisTopView/CircuitUn.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.map.Sprites.add(curve2)
    app.RunPygame(initialisation, update)

def SplineCurvef():
    def initialisation(app):
        return
    def update(app):
        return
    center = np.asarray([500, 700])
    point1 = np.asarray([1, 1])*100 + center
    point2 = np.asarray([-1, -2])*100 + center
    point3 = np.asarray([1, -4])*100 + center
    point4 = np.asarray([4, -3])*100 + center
    point5 = np.asarray([7, -5])*100 + center
    debugLevel = 4
    quality = 1 # 100%
    curve1 = Graph.B_SplineCurve([point1, point2, point3, point4, point5], quality, [], debugLevel)
    MapName = "TestsEnvironement/whitemap.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.RunPygame(initialisation, update)

def ArrayConverter(nps):
    array = []
    for el in nps:
        array.append(el)
    return array

def DrawPoint(app, point):
    x, y = point[0], point[1]
    r = fm.Config.Get("default point radius")
    color = fm.Config.Get("spline curve interpolation point color")
    ScreenRenderer.HUD.RenderPoint(app.map.worldMap, x, y, r, color)

def DrawSegment(app, segment):
    r = fm.Config.Get("default line radius")
    color = fm.Config.Get("spline curve interpolation point color")
    DrawPoint(app, segment.x1)
    DrawPoint(app, segment.x2)
    ScreenRenderer.HUD.RenderSegment(app.map.worldMap, segment.x1, segment.x2, r, color)

def DrawPolygon(app, polygon):
    prevP = polygon[0]
    DrawPoint(app, prevP)
    for i in range(1, len(polygon)):
        DrawSegment(app, Graph.Segment(polygon[i], prevP))
        prevP = polygon[i]

def OnGrid(point, division, center):
    point = np.asarray(point)
    point = point*division + np.asarray(center)*division
    return point.tolist()

def OnGrids(points, division, center):
    newPoints = []
    for point in points:
        newPoints.append(OnGrid(point, division, center))
    return newPoints

def SplineCurveIntersection():
    def initialisation(app):
        return
    pointList = [[1, 1], [-1, -2], [1, -4], [4, -3], [7, -5]]
    pointList1 = OnGrids(pointList, 100, [5, 7])
    pointList2 = OnGrids(pointList, 100, [2.5, 10])
    debugLevel = 4
    quality = 1 # 100%
    curve1 = Graph.B_SplineCurve(pointList1, quality, [], debugLevel)
    curve2 = Graph.B_SplineCurve(pointList2, quality, [], debugLevel)
    offset =  fm.Config.Get("spline intersection offset")
    intersections = curve1.Intersection(curve2, offset)
    def update(app):
        for point in intersections:
            DrawPoint(app, point)
    MapName = "TestsEnvironement/whitemap.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.map.Sprites.add(curve2)
    app.RunPygame(initialisation, update)

def SplineCurveProjection():
    def initialisation(app):
        return
    center = ArrayConverter(np.asarray([500, 700]))
    point1 = ArrayConverter(np.asarray([1, 1])*100 + center)
    point2 = ArrayConverter(np.asarray([-1, -2])*100 + center)
    point3 = ArrayConverter(np.asarray([1, -4])*100 + center)
    point4 = ArrayConverter(np.asarray([4, -3])*100 + center)
    point5 = ArrayConverter(np.asarray([7, -5])*100 + center)
    debugLevel = 1
    quality = 1 # 100%
    curve1 = Graph.B_SplineCurve([point1, point2, point3, point4, point5], quality, [], debugLevel)
    offset = fm.Config.Get("spline projection offset")
    x0 = ArrayConverter(np.asarray([500, 500]))
    projection = curve1.Projection(x0, offset)
    def update(app):
        DrawPoint(app, projection)
        DrawPoint(app, x0)
    MapName = "TestsEnvironement/whitemap.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.RunPygame(initialisation, update)

def CurveWrapperTheorem():
    def initialisation(app):
        return
    center = ArrayConverter(np.asarray([500, 700]))
    point1 = ArrayConverter(np.asarray([1, 1])*100 + center)
    point2 = ArrayConverter(np.asarray([-1, -2])*100 + center)
    point3 = ArrayConverter(np.asarray([1, -4])*100 + center)
    point4 = ArrayConverter(np.asarray([4, -3])*100 + center)
    point5 = ArrayConverter(np.asarray([7, -5])*100 + center)
    debugLevel = 1
    quality = 1 # 100%
    curve1 = Graph.B_SplineCurve([point1, point2, point3, point4, point5], quality, [], debugLevel)
    polygon0 = curve1.cPoints
    polygon1 = curve1.bPoints
    polygons = [polygon0, polygon1]
    N = 10
    for i in range(N-1):
        lastPolygon = polygons[len(polygons) - 1]
        polygons.append(Graph.B_SplineCurve.CurveWrapperTest(lastPolygon, 1/2))
    def update(app):
        DrawPolygon(app, polygons[0])
        DrawPolygon(app, polygons[1])
        DrawPolygon(app, polygons[len(polygons) -1])
        return
    MapName = "TestsEnvironement/whitemap.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.RunPygame(initialisation, update)

SplineCurveIntersection()
pathCurve1 = "Ressources/Maps/ParisTopView/CurvesTest1/Highways/curve1.png"

# GenerateGraph(pathCurve1)
# GenerateMap(pathCurve1)

"""_references_

Diff between Bezier and spline curves :https://www.quora.com/What-is-the-difference-between-a-Bezier-curve-and-a-spline#:~:text=The%20main%20difference%20between%20Bezier,points%20are%20on%20the%20curve.

How to draw spline curves : https://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

Different approches to find the intersection of 2 spline curves :https://medium.com/@all2one/intersecting-two-splines-70a1d901c446

Coktail of 2 techniques, the fastest one, but the most complicated one :https://www.sciencedirect.com/science/article/abs/pii/S0010448598000529?via%3Dihub

One technique, efficient with high amount of beziers curve in one spline curve and simple : https://pomax.github.io/bezierinfo/#intersections

Line - curve intersection source code : https://pomax.github.io/bezierinfo/chapters/intersections/curve-line.js

Curve - Curve Intersection source code : https://pomax.github.io/bezierinfo/chapters/curveintersection/curve-curve.js

Bezier curve length :https://pomax.github.io/bezierinfo/#arclength
"""
