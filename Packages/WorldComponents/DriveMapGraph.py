from re import X
import numpy as np
import Packages.WorldComponents.WorldRoads as WorldStuff
import pygame

class Graph(pygame.sprite.Sprite):
    def __init__(self, nodes, arcs, chunkNumber, w, h, debugLevel = 4):
        self.chunk = chunkNumber*[chunkNumber*[[]]]
        self.chunkNumber = chunkNumber
        self.w = w
        self.h = h
        self.nodes = []
        self.arcs = []
        for node in nodes:
            self.AddNode(node)
        for arc in arcs:
            self.AddArc(arc)
        self.debugLevel = debugLevel  # Niveau d'informations à afficher à l'écran

    def AddNode(self, localisation):  # Ajoute un noeud au graphe
        if (self.GetNode(localisation) == -1):  # Si les coordonées ne correspondent à aucun noeud du graphe
            self.nodes.append(localisation)
            return len(self.nodes) -1
        else: return self.GetNode(localisation)  # On retourne l'indice du noeud

    def AddArc(self, i, j, curve, chunksCrossed):  # Ajoute un arc au graphe
        for chunk in chunksCrossed:
            x = chunk % 1000
            y = chunk//1000
            self.chunk[x][y].append(len(self.Arcs))  # L'indice de l'arc
        self.Arcs.append(((i, j), curve, chunksCrossed))  # Un arc est un couple de sommet et un trajet (une courbe)

    def GetArcs(self, chunkID):  # Retourne l'ensemble des Arcs traversant le chunk
        x = chunkID % 1000
        y = chunkID // 1000
        a = []
        for i in self.chunk[x][y]:
            a.append(self.Arcs[i])
        return a

    def RemoveArc(self, i, j):  # Retire l'arc (i, j) du graphe
        for n, ((i1, j1), curve) in enumerate(self.Arcs):
            if (i, j) == (i1, j1):
                del self.Arcs[n]
                return curve

    def GetNode(self, M):  # Renvoie s'il existe l'indice de l'intersection se trouvant à (x, y) ( = le noeud)
        (x, y) = M
        for i, (x1, y1) in enumerate(self.nodes):
            if (x, y) == (x1, y1):
                return i
        return -1

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
        chunkID = CyCurves.EuclidianMap.ChunkOfM(M)
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

    def update(self, dt):  # Overrided
        pygame.sprite.Sprite.update(self)

    def Render(self, world):  # Invoquée dans la class ScreenRenderer.WorldRenderer
        level = self.debugLevel
        zoomRatio = world.cameraRect.width/world.worldMap.get_width()
        if level >= 1:
            for point in self.nodes:
                x, y = point
                NODE_COLOR = fm.Config.Get("graph node color")
                ARC_COLOR = fm.Config.Get("graph arc color")
                RADIUS = fm.Config.Get("bezier node radius")
                RADIUS = int(RADIUS *zoomRatio)
                WorldStuff.ScreenRenderer.HUD.RenderPoint(world.worldMap, x, y, RADIUS, NODE_COLOR)   # On dessinne le point du noeud sur la carte
        if level >= 2:
            for (i, j), curve, chunksCrossed in self.arcs:
                curve = curve.curve
                lastPoint = curve[0]
                for i in range(len(1, curve)):
                    WorldStuff.ScreenRenderer.HUD.RenderSegment(world.worldMap, point, lastPoint, RADIUS, ARC_COLOR)   # On dessine l'arc sur la carte
        return

    def GenerateGraph(curves, chunkNumber, w, h, pr):  # Génère un graphe à partir des courbes
        def appendOnce(list, n):
            for k in list:
                if k == n:
                    return list
            return list.append(n)
        graph = Graph([], [], chunkNumber, w, h)  # Initalisation
        graph.AddNode(curves[0][0])  # Le premier point de chaque courbe devient un noeud
        graph.AddNode(curves[0][len(curves[0])-1])  # Le dernier point de chaque courbe est un noeud
        for l in curves :
            i = graph.AddNode(l[0])
            prevk = 0  # Dernier indice
            chunksCrossed = []
            for k in range(len(l)):  # Parcours du tracé
                # Recherche des intersections entre la courbe et le graphe
                appendOnce(chunksCrossed, CyCurves.EuclidianMap.ChunkOfM(l[k], chunkNumber, w, h))  # On ajoute que la courbe appartient à ce chunk (une seule fois)
                M, (i2, j2), k2 = graph.NearestCurveOptimized(l[k])
                if (M == l[k]):  # Intersection !
                    j = graph.GetNode(M)
                    if j != -1:  # Si la courbe l n'intersecte pas un noeud du graphe en (l[k] == l2[k2])
                        j = graph.AddNode(M)  # Il faut donc créer un nouveau noeud
                        l2 = graph.RemoveArc(i2, j2)  # On coupe l'autre courbe en 2
                        graph.AddArc(i2, j, l2[0:k2], chunksCrossed)
                        graph.AddArc(j, j2, l2[k2:0], chunksCrossed)
                    graph.AddArc(i, j, l[prevk : k])  # On coupe le tracé ---[lastk : k]---
                    chunksCrossed = []  # Le nouvel arc traversera des chunks différents
                    prevk = k  # On recommence avec le reste de la courbe
            end = len(l) -1
            j = graph.AddNode(l[end])
            graph.AddArc(i, j, l[prevk : end], chunksCrossed)  # On coupe le tracé ---[lastk : k]---
        return graph

import pyximport
pyximport.install(language_level=3, setup_args={'include_dirs': np.get_include()})  # On introduit la dépendance numpy dans le module pyximport
import Packages.RessourcesManagers.FileManager as fm
class BasicCurve:  # Génère une courbe (liste de pixels) à partir d'une image appelé "plan"
    def BluePrintToCurve(blueprint, pr = None):
        def CompareColors(c1, c2):  # Compare 2 couleurs strictement
            for i, x in enumerate(c1):
                if x != c2[i]:
                    return False
            return True
        w = len(blueprint)  # Longueur
        h = len(blueprint[0])  # Largeur
        curve = []  # Une courbe est une liste contenant des coordonées
        k = 0
        for x in range(w):  # On cherche tous les pixels noirs, parcours complet
            for y in range(h):
                if CompareColors(blueprint[x][y], [0, 0, 0, 255]):
                    curve.append([x, y])  # On ajoute les coordonées des pixels noirs
            if pr != None:  # Bar de Progression
                k += 1
                pr.SetProgress(k/len(blueprint))  # Progression
        curve = fm.AsUint16(curve)  # Conversion en ndarray, tableau d'entier de 16 bits
        return curve

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