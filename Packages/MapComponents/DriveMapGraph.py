import numpy as np
import Packages.MapComponents.Cython.EuclidianCurves as CyCurves

class Graph:
    def __init__(self, nodes, arcs, chunkNumber, w, h):
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

    def AddNode(self, localisation):  # Ajoute un noeud au graphe
        if (self.GetNode(localisation) == -1):
            self.nodes.append(localisation)
            return len(self.nodes) -1
        else: return self.GetNode(localisation)

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
                elif minDist == -1 or EuclidianMap.Distance(M1, M) < minDist:
                    minDist = EuclidianMap.Distance(M1, M) # Recherche du point le plus proche
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

import pygame
import ScreenRenderer
import Packages.MapComponents.ImageManager as imm
class BezierCurve(pygame.sprite.Sprite):
    def __init__(self, points, quality, parameters, debugLevel):
        def GenerateCurveMemory(quality):
            MAX_QUALITY = imm.Config.Get("bezier curve quality")
            quality = MAX_QUALITY*quality
            for t in np.linspace(0, 1, quality):  # Dessin de la courbe
                point = self.Interpolate(t)   # On dessin un trait entre l'ancien et ce nouveau point, pour tous les points de la courbe de Bézier
                self.curve.append(point)
        pygame.sprite.Sprite.__init__(self)
        self.quality = quality  # Quality est un pourcentage
        self.parameters = parameters  # Parameters est une liste de paramètres t de points à surligner sur la courbe
        self.points = imm.AsUint16(points)
        self.sprite_image = None  # On ne veut rien dessiner, sprite_image de pygame
        self.debugLevel = debugLevel
        self.curve = []
        GenerateCurveMemory(quality)

    def Interpolate(self, t):  # Retourne le point de cette courbe de Bézier paramamétré par t
        def SegmentInterpolator(p1, p2, t):  # Retourne le point paramétré par t sur le segment[p1, P2]
            return (1-t)*p1 + t*p2
        def BezierProcess(points, t): 
            if len(points) == 1:
                return points[0]
            newPoints = []  # Les points du processus de Bézier
            for i in range(len(points)-1): 
                p1 = points[i]  # 2 points consécutifs dans la liste
                p2 = points[i+1]
                newPoints.append(SegmentInterpolator(p1, p2, t))  # Point situé sur le segment [p1, p2]
            return BezierProcess(imm.AsUint16(newPoints), t)  # La liste de point est diminuée de 1 à chaque réccursion
        return BezierProcess(self.points, t)

    def update(self, map, screen, dt):
        pygame.sprite.Sprite.update(self)

    def Render(self, world):  # Invoquée dans la class ScreenRenderer.MapRenderer
        level = self.debugLevel  # Niveau de débuggage
        zoomRatio = world.cameraRect.width/world.worldMap.get_width()
        if level >= 1:  # Rendu de la courbe de Bézier
            CURVE_COLOR = imm.Config.Get("bezier curve color")
            SIZE = imm.Config.Get("bezier curve radius")  
            SIZE = int(SIZE *zoomRatio) # la taille dépend du zoom
            point1 = self.curve[0]
            for i in range(len(self.curve) -1):
                point2 = self.curve[i+1]
                ScreenRenderer.HUD.DrawLine(world.worldMap, point1, point2, SIZE, CURVE_COLOR)   # On dessin le petit trait sur la carte
                point1 = point2
        RADIUS = imm.Config.Get("bezier point radius")
        RADIUS = int(RADIUS *zoomRatio)
        if level >= 2:  # Rendu du/des points spéciales
            SPECIAL_POINT_COLOR = imm.Config.Get("bezier special point color")
            for t in self.parameters:
                x, y = self.Interpolate(t)   # Coordonées du point spécial
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, SPECIAL_POINT_COLOR)
        if level >= 3:
            POINT_COLOR = imm.Config.Get("bezier point color")
            for point in self.points:
                x, y = point   # Coordonées du point définissant la courbe de Bézier
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, POINT_COLOR)
        if level >= 4:
            LINE_COLOR = imm.Config.Get("bezier point color")
            SIZE = imm.Config.Get("bezier curve radius")/3
            SIZE = int(SIZE *zoomRatio) # la taille dépend du zoom
            lastPoint = self.points[0]
            for point in self.points:
                x, y = point   # Coordonées du point définissant la courbe de Bézier
                ScreenRenderer.HUD.DrawLine(world.worldMap, lastPoint, point, SIZE, LINE_COLOR)
                lastPoint = point


class SplineCurve(pygame.sprite.Sprite):
    def __init__(self, points, quality, parameters, debugLevel):
        pygame.sprite.Sprite.__init__(self)  # Initialisation pygame
        self.pointsA = []  # Points de contrôles det points de controles des courbes de Béziers
        self.pointsB = []  # Points de controles des courbes de Béziers
        self.pointsS = points  # Points d'interpolation
        self.quality = quality
        self.debugLevel = debugLevel
        self.parameters = parameters
        self.pointsB, self.pointsA, self.bezierCurves = self.GeneratesBezierControlPoints()

    def Interpolate(self, t):
        k = int(t)
        return self.bezierCurves[k].Interpolate(t-k)

    def update(self, map, screen, dt):  # Overrided
        pygame.sprite.Sprite.update(self)
        for bezierCurve in self.bezierCurves:
            bezierCurve.update(map, screen, dt)

    def Render(self, world):  # Invoquée dans la class ScreenRenderer.MapRenderer
        for bezierCurve in self.bezierCurves:
            bezierCurve.Render(world)
        zoomRatio = world.cameraRect.width/world.worldMap.get_width()
        RADIUS = imm.Config.Get("bezier point radius")
        RADIUS = int(RADIUS *zoomRatio)
        POINT_B_COLOR = imm.Config.Get("spline curve interpolation point color")
        POINT_A_COLOR = imm.Config.Get("spline curve bezier point color")
        if self.debugLevel >= 2:  # Rendu des points d'interpolations des courbes de Béziers
            for point in self.pointsS:
                x, y = point   # Coordonées du point définissant la courbe de Bézier
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, POINT_B_COLOR)
        if self.debugLevel >= 3:
            for point in self.pointsA:  # Rendu des points de contrôle de la courbe de Spline
                x, y = point
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, POINT_A_COLOR)

    def GeneratesBezierControlPoints(self):  # Génère les points de controls de la courbe de Bézier passant par les n points de {points} appélés point d'interpolations S0 :: Sn
        def LinearCombination(a1, p1, a2, p2):
            x = a1 * p1[0] + a2 * p2[0]
            y = a1 * p1[1] + a2 * p2[1]
            return [x, y]
        def InverseOfM141(N):  # Retourne l'inverse de la matrice141 de taille N = n-2
            matrix141 = [[4, 1] + (N-2)*[0]]  # Matrice141 = Diag((1 4 1)), la matrice 
            for i in range(1, N-1):  # création des lignes intérieures de la matrice
                line = (i-1)*[0] + [1, 4, 1] + (N-i-2)*[0]  # Ligne de taille n 0 ... 0 1 4 1 0 ... 0
                matrix141.append(line)
            lastLine = (N-2)*[0] + [1, 4]
            matrix141.append(lastLine)
            return np.linalg.inv(matrix141)  # Matrice Inverse
        def GenerateCMatrix(matS):  # Génération de la matrice constante C à partir des points d'interpolations S0 :: Sn. Etape 1 du processus
            N = len(matS)
            C1 = LinearCombination(6, matS[1], -1, matS[0])  # 6*S1 - S0
            matC = [C1]
            for i in range(2, N-2):
                Ci = [matS[i][0]*6, matS[i][1]*6]  # 6*Si
                matC.append(Ci)
            Cn = LinearCombination(6, matS[N-2], -1, matS[N-1])  # 6*Sn-2 - Sn-1
            matC.append(Cn)  # De taille N-2
            return matC
        def TransfromIntoSquareMatrix(matrix):  # On transforme la matrice n, 2 non carrée en carrée n, n
            lines = len(matrix)
            columns = len(matrix[0])
            for i in range(lines):  # On ajoute la différences de lignes et de colones en colones
                matrix[i] = matrix[i] + max(lines-columns, 0)*[0]  # Il manque (lignes - colones) colones
            return matrix
        def MatrixMultiply(m1, m2):
            return [
                        [
                            sum([
                                m1[i][k] * m2[k][j]
                                for k in range(0, len(m2))
                            ])
                            for j in range(0, len(m2[0]))
                        ]
                        for i in range(0, len(m1))
                    ]
        def GenerateBMatrix(matA, matS):  # Etape 3 du processus
            matB = []  # List des points de controles des courbes de Béziers définissant la courbe de Spline
            bezierCurves = []
            lastSPoint = lastAPoint = matA[0]  # S0 = A0 = B0 (origine de la courbe)
            for i, Ai in enumerate(matA):
                Si = matS[i]
                Bi = LinearCombination(1/3, Ai, 2/3, lastAPoint)  # B(i//3) = 2/3 *Ai+1 + 1/3 *Ai
                Biplus1 = LinearCombination(2/3, Ai, 1/3, lastAPoint)  # B(i//3+1) = 2/3 *Ai + 1/3 *Ai+1
                Biplus2 = Si  # B(i//3 + 2) = Si
                matB.append(Bi)
                matB.append(Biplus1)  # Les 2 points du segments [Ai, Ai+1]
                matB.append(Biplus2)  # Et le point Si
                parameters = []  # Pas encore d'implémentations des points spéciaux
                biCurve = BezierCurve([lastSPoint, Bi, Biplus1, Si], self.quality, parameters, self.debugLevel)  # Courbe de Bézier (B(i//3), B(i//3+1), B(i//3+2), B(i+1//3))
                bezierCurves.append(biCurve)
                lastAPoint = Ai
                lastSPoint = Si
            return matB, bezierCurves
        points = self.pointsS
        n = len(points)  # n points
        invM141 = InverseOfM141(n-2)  # Seulement les n-2 points du milieux iront dans cette matrice -> on a besoin d'une matrtice de taille n-2
        matC = GenerateCMatrix(points)  # Etape 1 du processus
        matA = MatrixMultiply(invM141, matC)  # On calcluls les points A1 :: An-1. Etape 2 du processus
        matA.insert(0, points[0])  # A0 = S0
        matA.append(points[n-1])  # An = Sn
        matB, bezierCurves = GenerateBMatrix(matA, points)  # On calcul les points de controle de la courbe de Bézier. Etape 3 du processus
        return matB, matA, bezierCurves  # On retourne les points de controles de la courbe de Bézier parssant par les points B0 :: Bn

import pyximport
pyximport.install(language_level=3, setup_args={'include_dirs': np.get_include()})  # On introduit la dépendance numpy dans le module pyximport
import Packages.MapComponents.ImageManager as imm
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
        curve = imm.AsUint16(curve)  # Conversion en ndarray, tableau d'entier de 16 bits
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
                    if CyCurves.EuclidianMap.Distance((x, y), (x0, y0)) <= b:
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