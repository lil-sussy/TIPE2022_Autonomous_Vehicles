
import Packages.MapComponents.ImageManager as imm

import numpy as np

import pygame
import Packages.MapComponents.ImageManager as imm

# path = "C:\\Users\\nayke\\OneDrive\\devlab\\TIPE 2022\\AVehiclesMap\\Ressources\\Maps\\ParisTopView\\CurvesTest1\\"
# curve1BP = np.asarray(Image.open(path + "Highways\\Curve1.png"))
# w = len(curve1BP); h = len(curve1BP[0])

import Packages.MapComponents.DriveMapGraph as Graph
def GenerateCurve(bppath):
    progression = imm.ProgressBar("Génération de la courbe...") # Bar de progression
    blueprint = imm.Open(bppath)[0]
    curve = Graph.BasicCurve.BluePrintToCurve(blueprint, progression)
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
    parameters = [0.7]
    curve1 = Graph.SplineCurve([point1, point2, point3, point4, point5], quality, [], debugLevel)
    point = curve1.Interpolate(0.7)
    # curve2 = Graph.SplineCurve([point1, point3, point4, point2, point], quality, [], debugLevel)
    MapName = "TestsEnvironement/whitemap.png"
    app = ScreenRenderer.NeatApplication(1920, 1080)
    app.InitPygame(MapName)
    app.map.Sprites.add(curve1)
    # app.map.Sprites.add(curve2)
    app.RunPygame(initialisation, update)

pathCurve1 = "Ressources/Maps/ParisTopView/CurvesTest1/Highways/curve1.png"

# GenerateGraph(pathCurve1)
# GenerateMap(pathCurve1)

# https://www.quora.com/What-is-the-difference-between-a-Bezier-curve-and-a-spline#:~:text=The%20main%20difference%20between%20Bezier,points%20are%20on%20the%20curve.

# https://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf


SplineCurvef()