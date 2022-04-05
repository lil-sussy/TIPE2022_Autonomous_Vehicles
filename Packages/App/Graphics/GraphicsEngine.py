import pygame
import numpy as np
import ctypes
import os
ctypes.windll.user32.SetProcessDPIAware()
import Packages.RessourcesManagers.FileManager as fm

# Les collisions entre voitures sont gérer par la class Sprite de pygame
# Les collisions avec les bords de la route sont gérer par des détections de couleurs des coins de la voiture
class WorldRenderer:
    def __init__ (self, screenW, screenH):
        self.path = os.path.realpath(fm.Config.Get("ressource path") + fm.Config.Get("maps path"))
        self.screenW = screenW # La longueur de l'écran
        self.screenH = screenH # La largeur de l'écran
        self.zoomRatio = 1  # Ratio de zoom (taille de la caméra sur la carte/sur taille de l'écran)

    def InitWorldRenderer(self, mapName, jsonPath=""):
        def LoadWorldFiles(mapName, jsonPath):
            self.mapName = mapName # Le nom de la carte (inutile)
            path = os.path.realpath(fm.Config.Get("ressource path") + fm.Config.Get("maps path") + self.mapName)
            self.mapPicture = pygame.image.load(path).convert() # Convert() améliore les performances
            self.worldMap = pygame.image.load(path).convert()
            self.width = self.worldMap.get_width() # Longueur de la carte
            self.height = self.worldMap.get_height() # Largeur de la carte
            self.center = [self.width//2, self.height//2] # Centre de la carte
            self.fov_width = self.width # Longueur du "rectangle caméra", longueur de la carte découpé et représentée sur l'écran
            self.fov_height = self.height # Largeur de la découpe de la carte.
            self.empty_background = pygame.Surface((self.screenW, self.screenH)) # Nécéssaire pour copier un morceau de la carte dessus
            rectW, rectH = self.fov_width, self.fov_height
            self.cameraRect = pygame.Rect(self.width//2-rectW//2, self.height//2-rectH//2, rectW, rectH) # Position de la camera SUR LA CARTE
            self.road_color = None
        LoadWorldFiles(mapName, jsonPath) # Initialisation de la carte
        self.previous_cameraRect = pygame.Rect(0, 0, 1, 1) # Pour détecter si le zoom a changé
        self.camera = None # Initialisation

    def MapPygameCommands(self, keys, dt):  # Commandes de déplacement de la caméra sur la carte
        # Pan-box moves up/down/left/right, Zooms with p and m
        SPEED =  fm.Config.Get("camera move speed") # Vitesse de la caméra (mouvement) en pourcentage de la longeur de l'écran par secondes
        SPEED = self.cameraRect.width*SPEED*(dt/1000)  # dt est en milisecondes
        SPEED = int(SPEED/self.zoomRatio)
        ZOOM_SPEED = fm.Config.Get("camera zoom speed")
        cameraX = self.cameraRect.x
        cameraY = self.cameraRect.y
        cameraW = self.cameraRect.w
        cameraH = self.cameraRect.h
        if ( keys[pygame.K_UP] ): # Déplacement de la caméra vers le haut
            cameraY = max(0, cameraY - SPEED) # On ne doit pas dépasser y = 0
        if ( keys[pygame.K_DOWN] ): # Déplacement de la caméra verrs le bas
            bottomBorder = self.worldMap.get_height() - self.cameraRect.height - 1
            cameraY = min(bottomBorder, cameraY + SPEED) # Le bas du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        if ( keys[pygame.K_LEFT] ): # Déplacement de la caméra vers la gauche
            cameraX = max(0, cameraX - SPEED) # On ne doit pas dépasser x = 0
        if ( keys[pygame.K_RIGHT] ): # Déplacement de la caméra vers la droite
            rightBorder = self.worldMap.get_width() - self.cameraRect.width - 1
            cameraX = min(rightBorder, cameraX + SPEED) # Le bord droit du rectangle FOV (rectangle camera) ne doit pas dépasser l'écran
        if ( keys[pygame.K_p]): # Zoom de la caméra sur la carte
            diffW = cameraW - max(cameraW *(1 - ZOOM_SPEED), self.screenW//2)  # On ne zoom pas plus que : pixel(carte) = 2pixel(écran)
            diffH = cameraH - max(cameraH *(1 - ZOOM_SPEED), self.screenH//2)  # Zoom => Augmentation des dimensions du point de vue
            cameraW = cameraW - diffW
            cameraH = cameraH - diffH
            cameraX += diffW/2  # Recentrage
            cameraY += diffH/2
        if ( keys[pygame.K_m]): # Dé-zoom de la caméra sur la carte
            diffW = cameraW - min(cameraW *(1 + ZOOM_SPEED), self.width)  # On ne dézoome pas plus que la carte ne le permet
            diffH = cameraH - min(cameraH *(1 + ZOOM_SPEED), self.height)  # Dé-zoom => Augmentation des dimensions du point de vue
            cameraW = cameraW - diffW
            cameraH = cameraH - diffH
            cameraX += diffW/2  # Diff est déjà négatif
            cameraY += diffH/2
        self.cameraRect = pygame.Rect(cameraX, cameraY, cameraW, cameraH)

    def IsInBounds(self, point):
        x, y = point[0], point[1]
        return x >= 0 and y >= 0 and x < self.worldMap.get_width() and y < self.worldMap.get_height()

    def Bounds(self, point):
        x, y = point[0], point[1]
        x = max(0, point[0])
        y = max(0, point[0])
        x = min(x, self.worldMap.get_width() -1)
        y = min(x, self.worldMap.get_height() -1)
        return [x, y]

    def ScreenSizeToWorldSize(self, size):  # Convertit une taille (en pixels) sur l'écran en taille sur le monde sur la caméra 
        size = size *(self.width/self.screenW)  # Conversion en taille sur la carte
        size = int(size *self.zoomRatio)
        return size

    def ScreenFontToWorldFont(self, font):
        size = font[1]
        return [font[0], self.ScreenSizeToWorldSize(size), font[2]]

    def GetMousePosition(self):
        x, y = pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]
        worldX = self.cameraRect.left + x*self.zoomRatio
        worldY = self.cameraRect.top + y*self.zoomRatio
        return [worldX, worldY]

    def ClearMap(self):
        self.worldMap.blit(self.mapPicture, (0, 0)) # On efface les anciens dessin sur la carte

    def RenderMapOnScreen(self, screen):  # Rendu des informations additionelles des sprites
        def Zoom(self):
            if self.cameraRect != self.previous_cameraRect: # Si la caméra a changé de place ou de taille (taille du rectangle FOV)
                width_change = self.cameraRect.width != self.previous_cameraRect.width  # Changement de la taille de la caméra
                height_change = self.cameraRect.height != self.previous_cameraRect.height
                if width_change or height_change:  # Si le zoom a changé
                    self.zoomRatio = self.cameraRect.width/self.worldMap.get_width() # On calcule le ratio de zoom
                    self.camera = pygame.Surface( ( self.cameraRect.width, self.cameraRect.height ) ) # On change le zoom (la taille de zom)
                self.previous_cameraRect = self.cameraRect.copy() # on copie la taille du nouveau rectangle FOV (zoom)
        Zoom(self) # Zoom sur la carte si la TAILLE du rectangle camera sur la carte a changé
        screenSize = (self.screenW, self.screenH) # Dimension de l'écran
        pygame.transform.scale(self.camera, screenSize, self.empty_background) # On agrandit ou rétréssie zoomImage pour qu'il corresponde aux longeurs de l'écran, on rempli le reste avec du vide (emptyBackground)
        self.camera.blit(self.worldMap, (0, 0), self.cameraRect) # On dessine la partie découpé par le rectangle FOV (cameraRect) de la carte sur zoomImage
        # POURQUOI EMPTY_BACKGROUND ??????????????
        screen.blit(self.empty_background, (0, 0)) # Dans l'ordre mapPicture -> worldmap -> camera -> screen
        try:
            return
        except pygame.error:  # Apparement screen serait locked lors de la première éxécution, mais c'est insolvable
            return

    def UpdateMap(self, dt):
        return

import Packages.WorldComponents.DriveMapGraph as graph
class HUD:
    def __init__(self, world, screen):
        self.worldMap = world
        self.screen = screen
        self.screenW = screen.get_width()
        self.screenH = screen.get_height()
        self.worldW = world.get_width()
        self.worldH = world.get_height()
        self.sprites = []
        self.defaultFont = fm.Config.Get("default font")  # Initialisation de la police d'écriture par défaut
        self.mapPoints = []  # Points déssinés sur la carte
        self.mapSegments = [] # Lignes déssinées sur la carte
        self.mapSprites = pygame.sprite.Group([]) # Initialisation des Sprites (images sur la carte), on traite les sprites comme des composant objet à part entière

    def Update(self, dt):
        for sprite in self.mapSprites:
            sprite.Update(dt)

    def RenderScreenHud(self):
        return

    def RenderWorldHud(self):
        for sprite in self.sprites:
            sprite.Render(self.worldMap)
        for segment in self.mapSegments:
            HUD.RenderSegment(self.worldMap, segment.x1, segment.x2, segment.size, segment.color)
        for point in self.mapPoints:
            HUD.RenderPoint(self.worldMap, point.x1, point.x2, point.size, point.color)

    def AddWorldSprite(self, sprite):
        self.mapSprites.append(sprite)

    def AddWorldPoint(self, point, r, color):
        self.mapPoints.append((point, r, color))

    def AddWorldSegment(self, segment, r, color):
        self.mapPoints.append((segment.x1, segment.x2, r, color))

    def AddWorldSimplePoint(self, point):
        r = fm.Config.Get("default point radius")
        color = fm.Config.Get("spline curve interpolation point color")
        self.AddWorldPoint(point, r, color)

    def AddWorldDefaultSegment(self, segment):
        r = fm.Config.Get("default line radius")
        color = fm.Config.Get("spline curve interpolation point color")
        self.AddWorldSegment(self.worldMap, segment.x1, segment.x2, r, color)
        self.AddWorldPoint(segment.x1, r, color)
        self.AddWorldPoint(segment.x2, r, color)

    def RenderPolygon(surface, polygon):
        prevP = polygon[0]
        HUD.RenderPoint(surface, prevP)
        for i in range(1, len(polygon)):
            HUD.RenderSegment(surface, graph.Segment(polygon[i], prevP))
            prevP = polygon[i]

    def RenderPoint(surface, point, r, color):
        (x, y) = point[0], point[1]
        BLACK = (0, 0, 0)  # Contours noirs
        dr = 3  # Coutours de 3 pixels
        # pygame.draw.circle(surface, BLACK, (x, y), r+dr, 0)  # Contour noirs du point
        pygame.draw.circle(surface, color, (x, y), r, 0)  # Le point

    def RenderSegment(surface, point1, point2, size, color):
        pygame.draw.line(surface, color, point1, point2, size)

    def RenderText(surface, position, text, font):
        text = pygame.font.SysFont(font[0], font[1]).render(text, True, pygame.color.Color(font[2]))
        surface.blit(text, position)

class ClickableWorldHud():
    def __init__(self, world):
        self.world = world
        self.map = world.worldMap
        self.mouseListeners = []
        self.keyboardListeners = []
        self.buttons = []

    def AddSimpleButton(self, point, size, actionPerform):
        color = fm.Config.Get("spline curve interpolation point color")
        self.buttons.append(([point, size, color], actionPerform))

    def Update(self, dt):
        for button, action in self.buttons:
            x, y = button[0][0], button[0][1]  # Coordonées du button

    def Render(world):
        return