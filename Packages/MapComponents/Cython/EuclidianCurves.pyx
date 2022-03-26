# cython: language_level=3
cimport cython
cimport numpy as cnp
import numpy as np
import Packages.MapComponents.ImageManager as imm

class EuclidianMap:
    def Distance(cnp.ndarray[cnp.uint16_t, ndim=1] M1, cnp.ndarray[cnp.uint16_t, ndim=1] M2):
        cdef int x1 = M1[0]
        cdef int y1 = M1[1]
        cdef int x2 = M2[0]
        cdef int y2 = M2[1]
        return np.sqrt(np.power(x1 - x2, 2) + np.power(y1 - y2, 2)) # Distance Euclidienne

    def ChunkOfM(cnp.ndarray[cnp.uint16_t, ndim=1] M, int chunkNumber, int w, int h):
        x = M[0]
        y = M[1]
        chunkColumn = x // (w//chunkNumber) # On divise la carte en {chunkNumber}² chunks
        chunkLine = y // (h//chunkNumber) # On regarde à quelle case de la matrice le point (x, y) appartient
        chunkID = chunkColumn*1000 + chunkLine # première case de la matrice : 1001
        return chunkID

def BluePrintToCurve(cnp.ndarray [cnp.uint16_t, ndim=3] blueprint, pr = None):
        def CompareColors(cnp.ndarray [cnp.uint8_t, ndim=1] c1, cnp.ndarray[cnp.uint8_t, ndim=1] c2): # Compare 2 couleurs strictement
            for i, x in enumerate(c1):
                if x != c2[i]:
                    return False
            return True
        cdef int w = len(blueprint) # Longueur
        cdef int h = len(blueprint[0]) # Largeur
        curve = [] # Une courbe est une liste contenant des coordonées
        k = 0
        for x in range(w): # On cherche tous les pixels noirs, parcours complet
            for y in range(h):
                if CompareColors(blueprint[x][y], np.asarray([0, 0, 0, 255])):
                    curve.append([x, y]) # On ajoute les coordonées des pixels noirs
            if pr != None: # Bar de Progression
                k += 1
                pr.SetProgress(k/len(blueprint)) # Progression
        curve = np.asarray(curve) # Conversion en ndarray, tableau d'entier de 16 bits
        curve = imm.AsUint16(curve)
        return curve

# Liste de courbes, courbe = liste de points, point = liste de 2 coordonées
def DrawRoadsOnMap(cnp.ndarray[cnp.uint16_t, ndim=3] curvesList, cnp.ndarray[cnp.uint8_t, ndim=3] map, float dr, int brushSize, int roadRadius, pr = None):
    def Perpendicular(cnp.ndarray[cnp.uint16_t, ndim=1] v): # Retourne 2 vecteurs normés perpendiculaires à v
        cdef float x = v[0]
        cdef float y = v[1]
        if x == 0:
            return [-1, 0], [1, 0]
        if y == 0:
            return [0, 1], [0, -1]
        cdef float a = y*y/x # Produit scalaire
        cdef float n = np.sqrt(a*a + y*y) # Norme
        cdef cnp.ndarray [cnp.uint16_t, ndim=1] v1 = imm.AsUint16([a/n, -y/n]) # Normalisation
        cdef cnp.ndarray [cnp.uint16_t, ndim=1] v2 = imm.AsUint16([-a/n, y/n])
        return v1, v2
    def Differential(cnp.ndarray[cnp.uint16_t, ndim=2] curve): # Retourne la liste des vecteurs différentiels de la courbe
        cdef int length = curve.shape[0]
        cdef cnp.ndarray[cnp.uint16_t, ndim=2] vectors = np.zeros([length, 2], dtype = np.uint16) # liste de vecteurs vitesses
        last = curve[0] #Dernier point, différentiel
        curve = curve[1:]
        cdef float dx, dy
        for i in range(length-1):
            M = curve[i]
            x = M[0]; y = M[1]
            dx = x - last[0]
            dy = y - last[1]
            vectors[i] = imm.AsUint16([dx, dy]) # Vecteurs (dx, dy)
            last = [x, y]
        return vectors
    def Brush(cnp.ndarray[cnp.uint8_t, ndim=3] map, int x0, int y0, int brushSize, cnp.ndarray[cnp.uint8_t, ndim=1] color): # Paint une région sphérique de la carte
        for x in range(int(np.abs(x0-brushSize)), min(len(map)-1, int(x0+brushSize))):
            for y in range(int(np.abs(y0-brushSize)), min(len(map) -1, int(y0+brushSize))):
                if EuclidianMap.Distance((x, y), (x0, y0)) <= brushSize:
                    map[x][y] = color
    cdef int pcurve = 0, lenCurves = len(curvesList) # Progression des courbes
    cdef int pvector # Progression des vecteurs de la courbe
    cdef float radius
    cdef cnp.ndarray [cnp.uint16_t, ndim=1] v, v1, v2
    cdef int x, y, x1, y1, x2, y2 # (x, y) le centre; (x1, y1) coloriage à gauche, (x2, y2) coloriage à droite
    GRAY = [121, 123, 110, 255] # Couleur grise de la route
    for curve in curvesList:
        vectors = Differential(curve) # Prendre la dérivé : Liste de vecteurs
        pvcetor = 0
        for i in range(len(vectors)):
            v = vectors[i]
            radius = 0
            v1, v2 = Perpendicular(v) # 2 vecteurs normés perpendiculaires à v
            x = curve[i][0]
            y = curve[i][1]
            x1, y1 = (x, y) # On commence par le centre
            x2, y2 = (x, y) # x,y = center, x1 = gauche, x2 = droite
            while radius <= roadRadius - brushSize/2:
                x1 += int(v1[0]*dr)
                y1 += int(v1[1]*dr) # Prolongement de dr dans la direction v1
                x2 += int(v2[0]*dr)
                y2 += int(v2[1]*dr) # Prolongement de dr dans la direction v2
                Brush(map, x1, y1, brushSize, GRAY)
                Brush(map, x2, y2, brushSize, GRAY)
                radius += dr
            if pr != None:
                pvector += 1
                p = pvector/len(vectors)
                p = pcurve/lenCurves + p/lenCurves
                pr.SetProgress(p/lenCurves) # Progression
        pcurve += 1
    return map