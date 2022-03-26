
import Packages.MapComponents.ImageManager as imm

import numpy as np

import pygame
import Packages.MapComponents.ImageManager as imm
class BezierCurve(pygame.sprite.Sprite):
    def __init__(self, points, quality, parameters, debugLevel):
        pygame.sprite.Sprite.__init__(self)
        self.quality = quality # Quality est un pourcentage
        self.parameters = parameters # Parameters est une liste de paramètres t de points à surligner sur la courbe
        self.points = imm.AsUint16(points)
        self.sprite_image = None # On ne veut rien dessiner, sprite_image de pygame
        self.debugLevel = debugLevel
        self.curve = []
        self.GenerateCurveMemory(quality)

    def Interpolate(self, t): # Retourne le point de cette courbe de Bézier paramamétré par t
        def SegmentInterpolator(p1, p2, t): # Retourne le point paramétré par t sur le segment[p1, P2]
            return (1-t)*p1 + t*p2
        def BezierProcess(points, t): 
            if len(points) == 1:
                return points[0]
            newPoints = [] # Les points du processus de Bézier
            for i in range(len(points)-1): 
                p1 = points[i] # 2 points consécutifs dans la liste
                p2 = points[i+1]
                newPoints.append(SegmentInterpolator(p1, p2, t)) # Point situé sur le segment [p1, p2]
            return BezierProcess(imm.AsUint16(newPoints), t) # La liste de point est diminuée de 1 à chaque réccursion
        return BezierProcess(self.points, t)

    def GenerateCurveMemory(self, quality):
        MAX_QUALITY = imm.Config.Get("bezier curve quality")
        quality = MAX_QUALITY*self.quality
        for t in np.linspace(0, 1, quality): # Dessin de la courbe
            point = self.Interpolate(t)  # On dessin un trait entre l'ancien et ce nouveau point, pour tous les points de la courbe de Bézier
            self.curve.append(point)

    def update(self, map, screen, dt):
        pygame.sprite.Sprite.update(self)

    def Render(self, world): # overrided
        level = self.debugLevel # Niveau de débuggage
        zoomRatio = world.cameraRect.width/world.worldMap.get_width()
        if level >= 1: # Rendu de la courbe de Bézier
            CURVE_COLOR = imm.Config.Get("bezier curve color")
            SIZE = imm.Config.Get("bezier curve radius")  
            SIZE = int(SIZE *zoomRatio)# la taille dépend du zoom
            point1 = self.curve[0]
            for i in range(len(self.curve) -1):
                point2 = self.curve[i+1]
                ScreenRenderer.HUD.DrawLine(world.worldMap, point1, point2, SIZE, CURVE_COLOR)  # On dessin le petit trait sur la carte
                point1 = point2
        RADIUS = imm.Config.Get("bezier point radius")
        RADIUS = int(RADIUS *zoomRatio)
        if level >= 2: # Rendu du/des points spéciales
            SPECIAL_POINT_COLOR = imm.Config.Get("bezier special point color")
            for t in self.parameters:
                x, y = self.Interpolate(t)  # Coordonées du point spécial
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, SPECIAL_POINT_COLOR)
        if level >= 3:
            POINT_COLOR = imm.Config.Get("bezier point color")
            for point in self.points:
                x, y = point  # Coordonées du point définissant la courbe de Bézier
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, POINT_COLOR)

# path = "C:\\Users\\nayke\\OneDrive\\devlab\\TIPE 2022\\AVehiclesMap\\Ressources\\Maps\\ParisTopView\\CurvesTest1\\"
# curve1BP = np.asarray(Image.open(path + "Highways\\Curve1.png"))
# w = len(curve1BP); h = len(curve1BP[0])

import Packages.MapComponents.DriveMapGraph as Graph
def GenerateCurve(bppath):
    progression = imm.ProgressBar("Génération de la courbe...") # Bar de progression
    blueprint = imm.Open(bppath)[0]
    curve = Graph.Curve.BluePrintToCurve(blueprint, progression)
    progression.Done()
    return curve

import ScreenRenderer
import Packages.MapComponents.Cython.EuclidianCurves as Curves
def GenerateMap(pathCurve1):
    progression = imm.ProgressBar("Génération de la carte...") # Bar de progression
    curve1 = GenerateCurve(pathCurve1)
    ignored, w, h = imm.Open(pathCurve1)
    dr = 30; R = 650
    background = imm.AsUint8(w*[h*[[0, 0, 0, 255]]]) #taille w*h*4
    curves = imm.AsUint16([curve1]) # Liste de courbes
    result = Curves.DrawRoadsOnMap(curves, background, dr/3, dr, R//2)
    progression.Done()
    imm.Show(result)

def GenerateGraph(pathCurve1):
    progression = imm.ProgressBar("Génération du graphe...") # Bare de progression
    curve1 = GenerateCurve(pathCurve1)
    road, w, h = imm.Open(pathCurve1)
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
    curve1 = BezierCurve([point1, point3, point4, point2], quality, parameters, debugLevel)
    point = curve1.Interpolate(0.7)
    curve2 = BezierCurve([point1, point3, point4, point2, point], quality, [], debugLevel)
    MapName = "ParisTopView/CircuitUn.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    app.map.Sprites.add(curve2)
    app.RunPygame(initialisation, update)

pathCurve1 = "Ressources/Maps/ParisTopView/CurvesTest1/Highways/curve1.png"

# GenerateGraph(pathCurve1)
# GenerateMap(pathCurve1)

BezierCurvef()