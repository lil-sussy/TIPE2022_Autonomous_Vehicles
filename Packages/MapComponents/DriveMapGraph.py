from re import X
import numpy as np
import Packages.MapComponents.Cython.EuclidianCurves as CyCurves
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
                elif minDist == -1 or Segment(M1, M).Length() < minDist:
                    minDist = Segment(M1, M).Length() # Recherche du point le plus proche
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

    def update(self, map, screen, dt):  # Overrided
        pygame.sprite.Sprite.update(self)

    def Render(self, world):  # Invoquée dans la class ScreenRenderer.MapRenderer
        level = self.debugLevel
        zoomRatio = world.cameraRect.width/world.worldMap.get_width()
        if level >= 1:
            for point in self.nodes:
                x, y = point
                NODE_COLOR = imm.Config.Get("graph node color")
                ARC_COLOR = imm.Config.Get("graph arc color")
                RADIUS = imm.Config.Get("bezier node radius")
                RADIUS = int(RADIUS *zoomRatio)
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, NODE_COLOR)   # On dessinne le point du noeud sur la carte
        if level >= 2:
            for (i, j), curve, chunksCrossed in self.arcs:
                curve = curve.curve
                lastPoint = curve[0]
                for i in range(len(1, curve)):
                    ScreenRenderer.HUD.DrawLine(world.worldMap, point, lastPoint, RADIUS, ARC_COLOR)   # On dessine l'arc sur la carte
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

import pygame
import ScreenRenderer
import Packages.MapComponents.ImageManager as imm
""" _references_
Bezier Curve Point projection source code : https://pomax.github.io/bezierinfo/chapters/projections/project.js

Bezier curve length : https://pomax.github.io/bezierinfo/#arclength

Line - curve intersection source code : https://pomax.github.io/bezierinfo/chapters/intersections/curve-line.js

Curve - Curve Intersection source code : https://pomax.github.io/bezierinfo/chapters/curveintersection/curve-curve.js
"""
class BezierCurve(pygame.sprite.Sprite):
    def __init__(self, points, quality, parameters, debugLevel):
        def GenerateCurveMemory(quality):
            MAX_QUALITY = imm.Config.Get("bezier curve quality")
            quality = MAX_QUALITY*quality
            lastPoint = self.Interpolate(0)
            for t in np.linspace(0, 1, quality):  # Dessin de la courbe
                point = self.Interpolate(t)   # On dessin un trait entre l'ancien et ce nouveau point, pour tous les points de la courbe de Bézier
                self.curve.append(point)
                self.length += Segment(point, lastPoint).Length()
                lastPoint = point
        pygame.sprite.Sprite.__init__(self)
        self.quality = quality  # Quality est un pourcentage
        self.parameters = parameters  # Parameters est une liste de paramètres t de points à surligner sur la courbe
        self.points = points
        self.sprite_image = None  # On ne veut rien dessiner, sprite_image de pygame
        self.debugLevel = debugLevel
        self.curve = []
        self.length = 0
        GenerateCurveMemory(quality)

    def Interpolate(self, t):  # Retourne le point de cette courbe de Bézier paramamétré par t
        def BezierProcess(points, t): 
            if len(points) == 1:
                return points[0]
            newPoints = []  # Les points du processus de Bézier
            for i in range(len(points)-1): 
                p1 = points[i]  # 2 points consécutifs dans la liste
                p2 = points[i+1]
                newPoints.append(Segment(p1, p2).Interpolate(t))  # Point situé sur le Segment(x1, x2) [p1, p2]
            return BezierProcess(newPoints, t)  # La liste de point est diminuée de 1 à chaque réccursion
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


""" _references_
Diff between Bezier and spline curves : https://www.quora.com/What-is-the-difference-between-a-Bezier-curve-and-a-spline#:~:text=The%20main%20difference%20between%20Bezier,points%20are%20on%20the%20curve.

How to draw spline curves : https://www.math.ucla.edu/~baker/149.1.02w/handouts/dd_splines.pdf

Different approches to find the intersection of 2 spline curves : https://medium.com/@all2one/intersecting-two-splines-70a1d901c446

Coktail of 2 techniques, the fastest one, but the most complicated one : https://www.sciencedirect.com/science/article/abs/pii/S0010448598000529?via%3Dihub

One technique, efficient with high amount of beziers curve in one spline curve and simple : https://pomax.github.io/bezierinfo/#intersections
"""
class B_SplineCurve(pygame.sprite.Sprite):
    def __init__(self, points, quality, parameters, debugLevel):
        pygame.sprite.Sprite.__init__(self)  # Initialisation pygame
        # cPoints = points de contrôle de la courbe B Spline (polygone de control)
        # bPoints = points de contrôles des courbes de Béziers
        # sPoints = points sources (points d'interpolation) de la courbe B Spline
        self.cPoints = []  # Points de contrôles det points de controles des courbes de Béziers
        self.bPoints = []  # Points de controles des courbes de Béziers
        self.sPoints = points  # Points d'interpolation
        self.quality = quality
        self.debugLevel = debugLevel
        self.parameters = parameters
        self.bPoints, self.cPoints, self.bezierCurves, self.bezierNumber, self.length = self.GeneratesBezierControlPoints()

    def Interpolate(self, t):
        k = int(t)  # Partie entière
        deci = t - k  # Décimales
        return self.bezierCurves[k].Interpolate(deci)

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
        LINE_RADIUS = imm.Config.Get("spline curve line radius")
        LINE_RADIUS = int(LINE_RADIUS *zoomRatio)
        S_POINT_COLOR = imm.Config.Get("spline curve interpolation point color")
        B_POINT_COLOR = imm.Config.Get("spline curve bezier point color")
        if self.debugLevel >= 2:  # Rendu des points d'interpolations des courbes de Béziers
            for point in self.sPoints:
                x, y = point   # Coordonées du point définissant la courbe de Bézier
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, S_POINT_COLOR)
        if self.debugLevel >= 3:
            for point in self.cPoints:  # Rendu des points de contrôle de la courbe de Spline
                x, y = point
                ScreenRenderer.HUD.DrawPoint(world.worldMap, x, y, RADIUS, B_POINT_COLOR)
        if self.debugLevel >= 4:
            prevC = self.cPoints[0]
            for i in range(len(self.cPoints)): # Rendu du polygone de control de la courbe B Spline
                Ci = self.cPoints[i]
                ScreenRenderer.HUD.DrawLine(world.worldMap, prevC, Ci, LINE_RADIUS, B_POINT_COLOR)
                prevC = Ci

    def GeneratesBezierControlPoints(self):  # Génère les points de controls de la courbe de Bézier passant par les n points de {points} appélés point d'interpolations S0 :: Sn
        def InverseOfM141(N):  # Retourne l'inverse de la matrice141 de taille N = n-2
            matrix141 = [[4, 1] + (N-2)*[0]]  # Matrice141 = Diag((1 4 1)), première ligne : 4 1 0 ... 0
            for i in range(1, N-1):  # création des lignes intérieures de la matrice
                line = (i-1)*[0] + [1, 4, 1] + (N-i-2)*[0]  # Ligne de taille n 0 ... 0 1 4 1 0 ... 0
                matrix141.append(line)
            lastLine = (N-2)*[0] + [1, 4]  # Dernière ligne 0 ... 0 1 4
            matrix141.append(lastLine)
            return np.linalg.inv(matrix141)  # Matrice Inverse
        def EvaluateExtraSourcePoints(sPoints):  # Génération de la matrice constante C à partir des points d'interpolations S0 :: Sn. Etape 1 du processus
            N = len(sPoints)
            C1 = Segment(sPoints[1], sPoints[0]).LinearTransformation(6, -1)  # 6*S1 - S0
            extraSPoints = [C1]
            for i in range(2, N-2):  # Toutes les lignes intérieures de la matrice
                Ci = [sPoints[i][0]*6, sPoints[i][1]*6]  # 6*Si
                extraSPoints.append(Ci)
            Cn = Segment(sPoints[N-2], sPoints[N-1]).LinearTransformation(6, -1)  # 6*Sn-2 - Sn-1
            extraSPoints.append(Cn)  # De taille N-2
            return extraSPoints
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
        def FindBezierControlPoints(cPoints, sPoints):  # Etape 3 du processus, Génération des points de controles des courbes de Bézier
            def AppendAll(list, toAppend):
                for el in toAppend:
                    list.append(el)
                return list
            def AppendBezierCurve(bCurves, points):
                parameters = []  # Pas encore d'implémentations des points spéciaux
                curve = BezierCurve(points, self.quality, parameters, self.debugLevel)
                bCurves.append(curve)  # On ajoute la courbe de Bézier permettant de tracder la courbe de spline
                return bCurves, curve.length
            # cPoints = points de contrôle de la courbe B Spline (polygone de control)
            # bPoints = points de contrôles des courbes de Béziers
            # sPoints = points sources (points d'interpolation) de la courbe B Spline
            bPoints = []  # List des points de controles des courbes de Béziers définissant la courbe de Spline
            bCurves = []  # Liste des courbes de Bézier dessinant la courbe B Spline
            prevS = prevC = cPoints[0]  # S0 = A0 = B0 (origine de la courbe)
            length = 0   # Taille de la courbe B-Spline
            for i in range(1, len(cPoints)):  # Enumération des points de controles
                Ci = cPoints[i]
                Si = sPoints[i]
                points = Segment(prevC, Ci).Cut(3)
                Bk = points[1]  # B(k//3) = 2/3 *Ai+1 + 1/3 *Ai
                Bk1 = points[2]  # B(k//3+1) = 2/3 *Ai + 1/3 *Ai+1
                AppendAll(bPoints, [Bk, Bk1])  # Les 2 points du Segment(x1, x2)s [Ai, Ai+1] et le point Si -> courbe de Bézier (Si-1, Ai, Ai+1, Si)
                bCurves, curveLength = AppendBezierCurve(bCurves, [prevS, Bk, Bk1, Si])  # Courbe de Bézier (Si-1, B(k//3+1), B(k//3+2), Si)
                length += curveLength
                prevC = Ci  # Point de control précédent
                prevS = Si  # Point source précédent
            return bPoints, bCurves, length
        sPoints = self.sPoints
        n = len(sPoints)  # n points
        invM141 = InverseOfM141(n-2)  # Seulement les n-2 points du milieux iront dans cette matrice -> on a besoin d'une matrtice de taille n-2
        extraSourcePoints = EvaluateExtraSourcePoints(sPoints)  # Etape 1 du processus
        cPoints = MatrixMultiply(invM141, extraSourcePoints)  # On calcluls les points A1 :: An-1. Etape 2 du processus
        cPoints.insert(0, sPoints[0])  # A0 = S0
        cPoints.append(sPoints[n-1])  # An = Sn
        bPoints, bezierCurves, length = FindBezierControlPoints(cPoints, sPoints)  # On calcul les points de controle de la courbe de Bézier. Etape 3 du processus
        return bPoints, cPoints, bezierCurves, len(bezierCurves), length  # On retourne les points de controles de la courbe de Bézier parssant par les points B0 :: Bn

    def CurveWrapperTestOld(polygon, t0):  # Affine les Segment(x1, x2)s du polygone de contrôle de la courbe de Spline
            newPolygon = [polygon[0]]
            # L'on prend le paramètre t correspondant au plus grand segment entre [x1, t0/2] et [t0/2, x2]
            if t0 >= 1/2:  # t0 est le paramètre du point d'intesection sur l'un des segments du polygone : [x1, t0] [t0, x2] = [x1, x2]
                t = t0/2  # Si le point d'intersection est situé sur la 2e moitié du segment alors on prend la moitié de [x1, t0]
            else:
                t = 1 - t0/2  # S'il est sur l'autre moitié alors on prend l'autre moitié : [t0, x2]/2
            for i in range(1, len(polygon)):
                cut = Segment(polygon[i-1], polygon[i]).Interpolate(t) 
                newPolygon.append(cut)
            newPolygon.append(polygon[len(polygon) - 1])  # Le nouveau polygone est de taille n+1
            return newPolygon

    def CurveWrapperTest(polygon, t0):  # Affine les Segment(x1, x2)s du polygone de contrôle de la courbe de Spline
            newPolygon = [polygon[0]]
            t1 = 1/3
            t2 = 2/3
            for i in range(1, len(polygon)):
                cut1 = Segment(polygon[i-1], polygon[i]).Interpolate(t1) 
                cut2 = Segment(polygon[i-1], polygon[i]).Interpolate(t2) 
                newPolygon.append(cut1)
                newPolygon.append(cut2)
            newPolygon.append(polygon[len(polygon) - 1])  # Le nouveau polygone est de taille n+1
            return newPolygon

    def Projection(self, x, offset):  # Retourne
        fragment = offset/self.length  # On divise la courbe B-Spline en segments de longueurs offset
        bestProjection = Segment(self.Interpolate(0), self.Interpolate(fragment)).Projection(x)
        distMin = Segment(bestProjection, x).Length()
        for t in np.linspace(fragment, self.bezierNumber -1, int(1/fragment)):
            segment = Segment(self.Interpolate(t-fragment), self.Interpolate(t))
            if segment.Length() > 0:
                projection = segment.Projection(x)  # Projection sur le segemnt de taille offset
                dist = Segment(projection, x).Length()
                if dist < distMin:
                    bestProjection = projection
                    distMin = dist
        return bestProjection

    def ProjectionOld(self, x, offset):  # Retourne la projection de x sur la courbe B-Spline
        def PolygonProjection(polygon, x0):  # Retourne la projection du point x0 sur le polygone
            distMin = 10**9  # Très très grand nombre, initialisation
            prevC = polygon[0]
            for i in range(1, len(polygon)):  # Parcours des segments du polygone
                Ci = polygon[i]
                projection = Segment(prevC, Ci).Projection(x0)
                dist = Segment(projection, x0).Length()  # Distance entre le point et la projection
                if dist <= distMin:
                    distMin = dist  # Meilleure projection
                    projectionMin = projection
                    i0 = i
                    t0 = Segment(prevC, Ci).GetInterpolation(projection)
            return projectionMin, i0, t0  # On retourne les coordonées de la projection sur le polygone, mais aussi l'indice du segment projeteur, et le paramètre de la projection
        polygon = self.cPoints  # Polygone de contrôle de la courbe B-Spline
        lastProjection, i0, t0 = PolygonProjection(polygon, x)
        dist = offset+1  # Condition d'arrêt
        while(dist >= offset):
            polygon = B_SplineCurve.CurveWrapperTest(polygon, t0)  # Itération de la segmentation du polygone de contrôle
            projection, ignored, ignored = PolygonProjection(polygon, x)  # Projection du point x sur le polygone de contrôle
            dist = Segment(projection, lastProjection).Length()  # Distance entre les 2 projections
            lastProjection = projection
        polygon = B_SplineCurve.CurveWrapperTest(polygon, t0)  # Un dernier pour la route !
        projection, ignored, ignored= PolygonProjection(polygon, x)
        return projection

    def Intersection(self, bspline2, offset):  # Retourne
        def ConvexHullIntersection(bspline1, bspline2):  # Intersection des enveloppe convexe (rectangulaire) des 2 courbes B Spline
            return True
        def Subdivision(x1, x2, dx):  # Retourne une subdivsion de taille régulière dx de l'intervale x1 x2
            x = x1
            subdivision = [x1]
            while (x < x2):
                x += dx
                subdivision.append(x)
            del subdivision[len(subdivision)-1]  # On supprime le dernier élément sûrement plus grand que x2
            subdivision.append(x2)
            return subdivision
        if ConvexHullIntersection(self, bspline2): # Intersection des enveloppe convexe (rectangulaire)
            fragment1 = (offset/self.length)  # On divise la courbe B-Spline en segments de longueurs offset
            fragment2 = (offset/bspline2.length)  # On divise la courbe B-Spline en segments de longueurs offset
            intersections = []
            lastT1 = 0
            for t1 in Subdivision(fragment1, self.bezierNumber -1, fragment1):  # Intervale, [0, n] n = nombre de courbes de Bézier
                segment1 = Segment(self.Interpolate(lastT1), self.Interpolate(t1))
                lastT2 = 0
                for t2 in Subdivision(fragment2, bspline2.bezierNumber-1, fragment2):
                    segment2 = Segment(bspline2.Interpolate(lastT2), bspline2.Interpolate(t2))  # 2 points de la courbe paramétrés
                    intersection = segment1.Intersection(segment2)  # On cherche l'intersection des polygones courbes
                    if intersection:
                        intersections.append(intersection)
                    lastT2 = t2
                lastT1 = t1
            return intersections
        else:
            return False

    def IntersectionOld(self, bspline2, offset):  # Retourne, s'il existe, le point d'intersection avec la courbe B Spline
        def ConvexHullIntersection(bspline1, bspline2):  # Intersection des enveloppe convexe (rectangulaire) des 2 courbes B Spline
            return True
        def CurveWrapperHalf(polygon):  # Affine les Segment(x1, x2)s du polygone de contrôle de la courbe de Spline
            newPolygon = [polygon[0]]
            for i in range(1, len(polygon)):
                middle = Segment(polygon[i-1], polygon[i]).Cut(2)[1]
                newPolygon.append(middle)
            newPolygon.append(polygon[len(polygon) - 1])  # Le nouveau polygone est de taille n+1
            return newPolygon
        def PolygonsIntersection(polygon1, polygon2):
            prevCi = self.cPoints[0]
            prevCk = bspline2.cPoints[0]
            intersections = []  # Liste de toutes les intersections trouvées
            for i in range(1, len(polygon1)):  # Parcours des points (et des Segment(x1, x2)s) des polygones de controle des 2 courbes
                for k in range(1, len(polygon2)):
                    Ci = polygon1[i]
                    Ck = polygon2[k]
                    intersection = Segment(prevCi, Ci).Intersection(Segment(prevCk, Ck))  # Intersection des Segment(x1, x2)s des 2 polygones de controles
                    projection = self.Projection(intersection, offset)
                    if intersection != -1:  # Les 2 Segment(x1, x2)s (et donc les 2 polygones, et donc les 2 courbes s'intersectent)
                        if Segment(intersection, projection).Length() <= offset:  # Si l'intersection trouvé est suffisament proche de cette courbe
                            intersections.append(intersection)  # On ajoute l'intersection à la liste des intersections
                            return intersection  # Alors on a trouvé l'intersection
                        else:  # On coupe le polygone de contrôle des 2 courbes et on recommence
                            t1 = Segment(prevCi, Ci).GetInterpolation(intersection)  # Le paramètre du premier Segment(x1, x2) de l'intersection
                            t2 = Segment(prevCk, Ck).GetInterpolation(intersection)
                            polygon1 = B_SplineCurve.CurveWrapperTest(polygon1, t1)
                            polygon2 = B_SplineCurve.CurveWrapperTest(polygon2, t2)
                            return PolygonsIntersection(polygon1, polygon2)
                    prevCi = Ci
                    prevCk = Ck
            if len(intersections) == 0:
                return False   # Les 2 courbes ne s'intersectent pas
            else:
                return intersections
        if not ConvexHullIntersection(self, bspline2):  # Si les enveloppe convexe ne s'intersectent pas alors les courbes ne s'intersectent pas
            return False
        else:
            return PolygonsIntersection(self.cPoints, bspline2.cPoints)

"""_summary_
https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
"""
class Segment:
    def __init__(self, x1, x2):  # Créer le segment [x1, x2]
        self.x1 = x1
        self.x2 = x2

    def __str__(self):
        return "Segment({self.x1}, {self.x2})"

    def LinearTransformation(self, a1, a2):  # a1 *x1 + a2 *x2, combinaison linéaire de x1, x2, retourne un unique point
        x = a1 *int(self.x1[0]) + a2 *(self.x2[0])
        y = a1 *int(self.x1[1]) + a2 *(self.x2[1])
        return [x, y]

    def Interpolate(self, t):  # Retourne le point paramétré par t sur le segment[x1, x2]
        return self.LinearTransformation((1-t), t)  # t = 0 -> x1, t = 1 -> x2

    def Length(self):  # Distance entre 2 points
        dx = (self.x1[0]) - (self.x2[0])
        dy = (self.x1[1]) - (self.x2[1])
        return np.sqrt(np.power(dx, 2) + np.power(dy, 2))  # Distance Euclidienne

    def Cut(self, amount): # Coupe le segment [x1, x2] en {amount} segments
        ratio = 1/amount
        points = []
        for k in range(amount+1):  # Le premier point est x1 et le dernier est x2
            kratio = k*ratio
            point = self.Interpolate(kratio)  # Segment [a*x1 + (1-a)x2]
            points.append(point)
        return points

    def LineProjection(p1, p2, x0):  # Retourne la projection Orthogonale de x0 sur la droite prolongée (p1, p2)
        p1p2 = np.asarray([p1[0] - p2[0], p1[1] - p2[1]])  # vecteur directeur de la droite
        v0Norm = np.linalg.norm(p1p2)  # Norme du vecteur directeur
        p1x0 = np.asarray([p1[0] - x0[0], p1[1] - x0[1]])  # Vecteur p1 x0
        dotProduct = np.dot(p1x0, p1p2)  # Produit scalaire de la droite et du vecteur p1 x0
        gramSchmidt = p1x0 - (dotProduct/v0Norm) *p1p2  # Procédé de GraSchmidt
        return [p1[0] + gramSchmidt[0], p1[1] + gramSchmidt[1]]  # Projection orthogonale de x0 sur la droite

    def Projection(self, x0):
        projection = Segment.LineProjection(self.x1, self.x2, x0)
        if self.Fits(projection):
            return projection
        elif Segment(self.x1, projection).Length() < Segment(self.x2, projection).Length():  # Il faut retourner x1 ou x2
            return self.x1
        else:
            return self.x2

    def LineLineInterection(p1, p2, p3, p4):  # Intersection des droites (p1, p2) (p3, p4)
        x1, y1 = (p1[0], p1[1])  # Point -> 2 dimensions, donc système de Kramer 8x8, ou alors system de Kramer par bloc 4x4
        x2, y2 = (p2[0], p2[1])
        x3, y3 = (p3[0], p3[1])
        x4, y4 = (p4[0], p4[1])
        detx = (x1*y2 - x2*y1)*(x3 - x4) - (x3*y4 - x4*y3)*(x1 - x2)  # Determinant de l'absice
        dety = (x1*y2 - x2*y1)*(y3 - y4) - (x3*y4 - x4*y3)*(y1 - y2)  # Determinant de l'ordonnée
        kramerCondition = (x1 - x2)*(y3 - y4) - (x3 - x4)*(y1 - y2)  # Déterminant de Kramer, intersection unique des droites ssi ce déterminant est non nulle
        if kramerCondition == 0:   # Les lignes sont parallèles ou confondu
            return -1
        else:
            return [(detx/kramerCondition), (dety/kramerCondition)]

    def GetInterpolation(self, x):  # Retourne le paramètre t du Segment(x1, x2) correspondant au point x, si x appartient au Segment(x1, x2)
        if not self.Fits(x):
            return False
        else:
            return Segment(self.x1, x).Length()/self.Length()  # Distance entre x1 et x sur la distance totale

    def Fits(self, x):  # Retourne si oui ou non x appartient au Segment [p1 P2]
        d1 = Segment(self.x1, x).Length()  # Distance p1, x
        d2 = Segment(self.x2, x).Length()
        length = int((d1 + d2)*100)  # On regarde jusqu'à la 2e décimale uniquement
        return length == int(self.Length()*100)  # Si la somme des distances n'est pas la longueur du Segment(x1, x2) alors le point n'est pas sur le Segment(x1, x2)

    def Intersection(self, segment2):  # Retourne, s'il existe, le point d'intersection avec le 2e Segment
        intersection = Segment.LineLineInterection(self.x1, self.x2, segment2.x1, segment2.x2)  # Intersection des droites des segments
        if intersection == -1:
            return False
        if segment2.Fits(intersection) and self.Fits(intersection):  # Si le point n'appartient pas aux 2 Segments alors ce n'est pas une intersection
            return intersection
        return False

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