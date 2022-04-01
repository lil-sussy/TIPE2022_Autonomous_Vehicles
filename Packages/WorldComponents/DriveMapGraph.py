from re import X
import numpy as np
import Packages.WorldComponents.WorldRoads as WorldStuff
import pygame
import Packages.App.Graphics.GraphicsEngine as ge

class Graph(pygame.sprite.Sprite):
    def __init__(self, chunkNumber, worldWidth, worldHeight, nodes = [], arcs = [], debugLevel = 4):
        self.chunkNumber = chunkNumber  # Le nombre de chunk en longueur pour partionner la carte entière
        self.chunks = ([[] for column in range(chunkNumber)] for line in range(chunkNumber))  # Matrice des chunks (séparation de la carte), on ne souhaite pas utiliser le module numpy et les ndarray
        self.width = worldWidth  # The width of the worldmap
        self.height = worldHeight  # The height of the worldmap
        self.nodes = []  # The nodes (intersections) of the graph
        self.arcs = []  # the arcs (the roads) of the graph
        for node in nodes:
            self.AddNode(node)  # adding the starting intersections to the graph
        for arc in arcs:
            self.AddArc(arc)  # Adding the starting roads to the graph
        self.debugLevel = debugLevel  # Niveau d'informations à afficher à l'écran

    def AddNode(self, localisation):  # Ajoute un nouveau noeud au graphe
        if (self.GetNode(localisation) == -1):  # On ne l'ajoute que si ses coordonées ne correspondent à aucun noeud du graphe
            self.nodes.append(localisation)
            return len(self.nodes) -1  # On retourne l'indice du nouveau noeud
        else: return self.GetNode(localisation)  # On retourne l'indice du noeud

    def AddArc(self, i, j, sourcePoints):  # Ajoute un arc (i, j) à la liste des arcs du graphe
        curve = WorldStuff.B_SplineCurve(sourcePoints, 1, [], self.chunkNumber, self.debugLevel-2)  # On créer l'interpolation des points sources tracant la route
        for chunk in curve.chunksCrossed:  # La courbe traverse des chunks de la carte
            x = chunk % 1000
            y = chunk//1000
            arc_indice = len(self.arcs)  # L'indice du nouvelle arc
            self.chunks[x][y].append(arc_indice)  # On ajoute l'arc au chunk, indiquant que l'arc traverse ce chunk
        self.arcs.append(((i, j), curve, curve.chunksCrossed))  # Un arc est un couple de sommet et un trajet (une courbe B-Spline) ainsi que les chunks traversés  par cette même coubre

    def GetArcs(self, chunkID):  # Retourne l'ensemble des Arcs traversant le chunk
        x = chunkID % 1000
        y = chunkID // 1000
        a = []
        for i in self.chunks[x][y]:
            a.append(self.arcs[i])
        return a

    def RemoveArc(self, i, j):  # Retire l'arc (i, j) du graphe
        for n, ((i1, j1), curve) in enumerate(self.arcs):
            if (i, j) == (i1, j1):
                del self.arcs[n]
                return curve

    def GetNode(self, M):  # Renvoie s'il existe l'indice de l'intersection se trouvant à (x, y) ( = le noeud)
        (x, y) = M
        for i, (x1, y1) in enumerate(self.nodes):
            if (x, y) == (x1, y1):
                return i
        return -1

    def EvaluateChunk(self, M):  # Retourne le chunk dans lequel se trouve M
        x = M[0]
        y = M[1]
        chunkColumn = x // (self.width//self.chunkNumber) # On divise la carte en {self.chunkNumber}² chunks
        chunkLine = y // (self.height//self.chunkNumber) # On regarde à quelle case de la matrice le point (x, y) appartient
        chunkID = chunkColumn*1000 + chunkLine # première case de la matrice : 1001
        return chunkID

    def NearestCurveNaif(Arcs, M):  # Renvoie la courbe de la liste Arcs la plus proche du point M
        # Algorithme Naif
        (x, y) = M
        minDist = -1
        minM = (x, y)
        for n, ((i, j), curve) in enumerate(Arcs):  # Parcours des arcs
            for k, M1 in enumerate(curve):  # Parcours de la courbe de l'arc
                if M1 == M:  # Le point est sur la courbe
                    return M, (i, j), k
                elif minDist == -1 or WorldStuff.Segment(M1, M).Length() < minDist:
                    minDist = WorldStuff.Segment(M1, M).Length() # Recherche du point le plus proche
                    minM = M1, (i, j), k
        return minM

    def NearestCurveOptimized(self, M):  # Renvoie la courbe du graphe la plus proche de M
        chunkID = self.EvaluateChunk(M)
        m0 = chunkID % 10000
        n0 = chunkID // 1000
        arcs = []  # La recherche par chunks est d'autant plus rapide que le nombre de chunks est élevé
        r = 1  # Le rayon de chunk autour du chunk 0 (le chunk de M)
        while len(arcs) == 0 and r < self.chunkNumber:  # On fait grandir r pour toucher une courbe
            begin = max(0, m0 - r*1000)  # On ne dépasse pas les limites de la matrice de chunk
            end = min(self.chunkNumber*1000, m0 + r*1000)
            for m in range(begin, end, 1000):
                begin = max(0, n0 - r)
                end = min(self.chunkNumber, n0 + r)
                for n in range(begin, end, 1):
                    for arc in self.GetArcs(1000*m + n):
                        arcs.append(arc)
            r += 1
        return Graph.NearestCurveNaif(arcs, M)  # Plus proche point, complexité n

    def update(self, world, dt):  # Overrided
        pygame.sprite.Sprite.update(self)
        for (i, j), curve, chunksCrossed in self.arcs:
            curve.update(world, dt)  # Toute les courbes sont des sprites, on les updates (inutiles pour le moment)

    def Render(self, world):  # Invoquée dans la class ScreenRenderer.WorldRenderer
        level = self.debugLevel
        NODE_COLOR = fm.Config.Get("graph node color")
        ARC_COLOR = fm.Config.Get("graph arc color")
        RADIUS = world.ScreenSizeToWorldSize(fm.Config.Get("graph node radius"))
        if level >= 1:
            for point in self.nodes:
                ge.HUD.RenderPoint(world.worldMap, point, RADIUS, NODE_COLOR)   # On dessinne le point du noeud sur la carte
        if level >= 2:
            for (i, j), curve, chunksCrossed in self.arcs:  # On rend le tracé des courbes des arcs sur l'écran
                curve.Render(world)
        if level >= 3:
            FONT = world.ScreenFontToWorldFont(fm.Config.Get("default font"))
            for i in range(len(self.nodes)):  # On ajoute sur l'écran l'indice des noeuds
                point = self.nodes[i]
                x = point[0] - 2*RADIUS
                y = point[1]  # On écrit au milieu du point du noeud
                nodeName = "intersection " + str(i)
                ge.HUD.RenderText(world.worldMap, [x, y], nodeName, FONT)   # On dessinne le point du noeud sur la carte

    def GenerateGraph(roads, numberOfChunks, worldW, worldH, progression = None):  # Génère un graphe à partir d'une liste de listes de points sources, véritable constructeur de cette classe
        graph = Graph(numberOfChunks, worldW, worldH)  # Initalisation
        for road in roads:
            start, end, s_points = road  # La route est une courbe S-Spline passant par les points s_points et liant les 2 intersections de coordonées start et end
            i = graph.AddNode(start)
            j = graph.AddNode(end)
            graph.AddArc(i, j, s_points)
        return graph

import pyximport
pyximport.install(language_level=3, setup_args={'include_dirs': np.get_include()})  # On introduit la dépendance numpy dans le module pyximport
import Packages.RessourcesManagers.FileManager as fm
# Dessine les routes sur la carte à partir de courbes
def DrawRoadsOnMap(L, map, dr, b, R, pr = None):
    def Perpendicular(v):  # Retourne 2 vecteurs normés perpendiculaires à v
        (x, y) = (v[0], v[1])
        if x == 0:
            return [-1, 0], [1, 0]
        if y == 0:
            return [0, 1], [0, -1]
        a = y*y/x  # Produit scalaire
        n = np.sqrt(a*a + y*y)  # Norme
        return [a/n, -y/n], [-a/n, y/n]  # Normalisation
    def Differential(L):  # Retourne la liste des vecteurs différentiels de la courbe
        V = []
        last = L[0]  #Dernier point, différentiel
        L = L[1:]
        for M in L:
            x = M[0]; y = M[1]
            V.append([x - last[0], y - last[1]])  #Vecteur dx, dy
            last = [x, y]
        return V
    def Brush(map, x0, y0, b, color):  # Paint une région sphérique de la carte
        for x in range(int(np.abs(x0-b)), min(len(map)-1, int(x0+b))):
            for y in range(int(np.abs(y0-b)), min(len(map) -1, int(y0+b))):
                if WorldStuff.Segment([x, y], [x0, y0]).Length() <= b:
                    map[x][y] = color
    pcurve = 0  # Progression des courbes
    for curve in L:
        #Prendre la dérivé : Liste de vecteurs
        speedList = Differential(curve)
        pvector = 0  # Progression des vecteurs de la courbe
        for i, v in enumerate(speedList):
            r = 0
            v1, v2 = Perpendicular(v)  #2 vecteurs normés perpendiculaires à v
            x = curve[i][0]; y = curve[i][1]
            x1, y1 = (x, y)
            x2, y2 = (x, y)  #x,y = center, x1 = gauche, x2 = droite
            while r <= R - b/2:
                x1 += v1[0]*dr
                y1 += v1[1]*dr
                x2 += v2[0]*dr
                y2 += v2[1]*dr  #Coloriage à gauche et à droite
                GRAY = [121, 123, 110, 255]
                Brush(map, x1, y1, b, GRAY)
                Brush(map, x2, y2, b, GRAY)
                r += dr
            if pr != None:
                pvector += 1
                p = pvector/len(speedList)
                p = pcurve/len(L) + p/len(L)
                pr.SetProgress(p/len(L))  # Progression
        pcurve += 1
    return map