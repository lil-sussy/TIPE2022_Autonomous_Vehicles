import Packages.RessourcesManagers.FileManager as fm
import Packages.App.Graphics.GraphicsEngine as ge
import pygame
from Packages.WorldComponents.NeatVisualizer import *

"""
Liens qui m'ont été utiles :
1) Blit Rotate :
https://newbedev.com/python-how-to-rotate-a-sprite-pygame-code-example

2) Image Viewer :
https://stackoverflow.com/questions/63795236/best-way-to-create-image-viewer-and-allowing-zoom-drag-and-line-drawing-by-pyth
"""

#Fonction vide
def EmptyFunction(app):
    return

#Fonctions d'initialisations.
Initialisation = EmptyFunction
Update = EmptyFunction

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
import Packages.WorldComponents.VehicleCylindrique as vh
def Experiment3():
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
    App = ge.NeatApplication(1920, 1080)
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



import numpy as np

import pygame

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
    app = ge.NeatApplication(1920, 1080)
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
    app = ge.NeatApplication(1920, 1080)
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
    ge.HUD.RenderPoint(app.map.worldMap, x, y, r, color)

def DrawSegment(app, segment):
    r = fm.Config.Get("default line radius")
    color = fm.Config.Get("spline curve interpolation point color")
    DrawPoint(app, segment.x1)
    DrawPoint(app, segment.x2)
    ge.HUD.RenderSegment(app.map.worldMap, segment.x1, segment.x2, r, color)

def DrawPolygon(app, polygon):
    prevP = polygon[0]
    DrawPoint(app, prevP)
    for i in range(1, len(polygon)):
        DrawSegment(app, Graph.Segment(polygon[i], prevP))
        prevP = polygon[i]

def OnGrid(point, original_dimensions, new_dimensions, custom_ratio = 1):
    original_w, original_h = original_dimensions  # Dimensions de l'image de départ (la grille)
    original_center = [original_w//2, original_h//2]  # Centre de la grille
    new_w, new_h = new_dimensions  # Dimensions de l'image d'arrivé
    new_center = [new_w//2, new_h//2]  # Centre de la nouvelle image
    ratio = new_w/original_w*custom_ratio  # Ratio de taille
    point = np.asarray(point) - np.asarray(original_center)  # On centre le point
    point = point*ratio + np.asarray(new_center)  # Transformation linéaire dans le nouveau monde
    return point.tolist()

def OnGrids(points, original_dimensions, new_dimensions, custom_ratio = 1):
    newPoints = []
    for point in points:
        newPoints.append(OnGrid(point, original_dimensions, new_dimensions, custom_ratio))
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
    app = ge.NeatApplication(1920, 1080)
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
    app = ge.NeatApplication(1920, 1080)
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
    app = ge.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.RunPygame(initialisation, update)

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

import Packages.WorldComponents.WorldRoads as crv
import Packages.WorldComponents.DriveMapGraph as gr
import Packages.App.Application as ap
import Packages.App.Graphics.GraphicsEngine as ge


class IntersectionGrid:
    def __init__(self, grid_size, grid_origin, world):
        def AddAvalailbleIntersection(point):
            self.available_points.append(point)
        self.grid_size = grid_size
        self.grid_origin = grid_origin
        self.world_width = world.worldMap.get_width()
        self.world_height = world.worldMap.get_height()
        self.available_points = []  # Points disponibles
        self.used_points = []  # Points déjà utilisés (réutilisables)
        self.intersections = []
        self.roads = []
        self.choosed_points = []
        grid_lenght = self.world_width//grid_size
        for i in range(grid_size):
            x = grid_lenght *i
            y = grid_lenght *i
            point = [x, y]
            if world.IsInBounds(point):
                AddAvalailbleIntersection(point)
        self.available_color = fm.Config.Get("grid available point color")
        self.used_color = fm.Config.Get("grid used point color")
        self.intersection_color = fm.Config.Get("grid intersection color")
        self.point_size = fm.Config.Get("grid point size")

    def ChoosePoint(self, point):
        self.choosed_points.append(point)

    def CreateRoadFromChoosedPoints(self):
        road = crv.B_SplineCurve(self.choosed_points, 1, [], self.grid_size)
        intersections = self.choosed_points[0], self.choosed_points[len(self.choosed_points) -1]
        self.roads.append((intersections, road))
        self.choosed_points = []

    def ProcessEvents(self):
        def InBounds(point, size, pos):
            x, y = pos[0], pos[1]
            xcondition = (x >= point[0] - size) and (x <= point[0] + size)
            ycondition = (y >= point[1] - size) and (y <= point[1] + size)
            return xcondition and ycondition
        def ProcessAddPoint():
            pos = self.world.MousePosition()
            for point in self.available_points:
                if InBounds(point, self.point_size, pos):
                    self.ChoosePoint(point)
            for point in self.used_points:
                if InBounds(point, self.point_size, pos):
                    intersection = point
                    for begin, end, road in self.roads:
                        for i, sourcePoint in enumerate(road.sPoints):
                            if sourcePoint == intersection:  # Croisement de routes
                                sPoints = road.sPoints
                                road2 = crv.B_SplineCurve(sPoints[:i], road.quality, [], self.grid_size)
                                self.roads.append((begin, intersection, road2))
                                road3 = crv.B_SplineCurve(sPoints[:i], road.quality, [], self.grid_size)
                                self.roads.append((intersection, end, road3))
                                break
                    self.ChoosePoint(point)
                    self.CreateRoadFromChoosedPoints()
            for point in self.intersections:
                if InBounds(point, self.point_size, pos):
                    self.ChoosePoint(point)
        mouse = pygame.mouse
        if mouse.get_pressed()[0]:
            ProcessAddPoint()
    
    def Update(self, dt):
        for road in self.roads:
            road.Update(dt)
        return

    def Render(self):
        for point in self.available_points:
            ge.HUD.RenderPoint(self.world.worldMap, point, self.point_size, self.available_color)
        for point in self.used_points:
            ge.HUD.RenderPoint(self.world.worldMap, point, self.point_size, self.used_color)
        for intersection in self.intersections:
            ge.HUD.RenderPoint(self.world.worldMap, intersection, self.point_size, self.intersection_color)
        for road in self.roads:
            road.Render(self.world.worldMap)


def GenerateGraph():
    mapName = "/TestsEnvironement/large_map.png"
    chunk_number = 32
    def Init(app):
        dim_world = (app.world.width, app.world.height)
        dim_blueprint = 386*2, 201*2
        ratio = 1
        nodes = [[62, 27], [62, 94], [6, 94], [62, 168], [244, 192], [244, 6], [378, 94]]
        nodes = OnGrids(nodes, dim_blueprint, dim_world, ratio)
        w, h = dim_world
        app.graph = gr.Graph(chunk_number, w, h, nodes, [])
        # Les 2 routes courbées, les voies d'intersections
        sPoints = [[244, 6], [264, 50], [300, 74], [351, 80]]
        app.graph.AddArc(5, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[244, 192], [264, 144], [300, 116], [351, 110]]
        app.graph.AddArc(4, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        # Les 3 première venant du haut
        sPoints = [[48, 38], [48, 72]]
        app.graph.AddArc(0, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[62, 38], [62, 72]]
        app.graph.AddArc(0, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[74, 38], [74, 72]]
        app.graph.AddArc(0, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        # Les 3 routes venants du bas
        sPoints = [[48, 118], [48, 157]]
        app.graph.AddArc(3, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[62, 118], [62, 157]]
        app.graph.AddArc(3, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[74, 118], [74, 157]]
        app.graph.AddArc(3, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        # Les 4 routes de gauches
        sPoints = [[6, 80], [38, 80]]
        app.graph.AddArc(2, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[6, 90], [38, 90]]
        app.graph.AddArc(2, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[6, 100], [38, 100]]
        app.graph.AddArc(2, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[6, 110], [38, 110]]
        app.graph.AddArc(2, 1, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        # Les 4 routes de droites
        sPoints = [[87, 80], [351, 80]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[87, 90], [351, 90]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[87, 100], [351, 100]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[87, 110], [351, 110]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        # Les 4 routes de droites droites
        sPoints = [[351, 80], [401, 80]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[351, 90], [401, 90]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[351, 100], [401, 100]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
        sPoints = [[351, 110], [401, 110]]
        app.graph.AddArc(1, 6, OnGrids(sPoints, dim_blueprint, dim_world, ratio))
    def Render(app):
        app.graph.Render(app.world)
    def Update(app, dt):
        app.graph.update(app.world, dt)
    app = ap.SimpleApplication(mapName, EmptyFunction, Render, Update, Init)
    app.Run()