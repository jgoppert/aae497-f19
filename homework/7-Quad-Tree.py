# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 14:51:58 2019

@author: cohen
"""

import matplotlib.pyplot as plt
import numpy as np
from random import random
from random import seed

seed(4)

class Quad(object):
    def __init__(self,left,right,down,up):
        self.leftb = left
        self.rightb = right
        self.downb = down
        self.upb = up
        self.childQuads = [None,None,None,None]
        self.particles = None
   
#Use this method to see which particles from any list of particles are enclosed by the quad.     
    def getPartsInQ(self,parts):
        partsInQ = []
        externParts = []
        for aPart in parts:
            if((aPart.xpos > self.leftb) and (aPart.xpos < self.rightb) and (aPart.ypos > self.downb) and (aPart.ypos < self.upb)):
                partsInQ.append(aPart)
            else:
                externParts.append(aPart)
            
        return externParts, partsInQ
    
    def doesRectOverlap(self,rectLb,rectRb,rectDb,rectUb):
        doesOverlap = not((self.rightb<rectLb)or(self.leftb>rectRb)or(self.upb<rectDb)or(self.downb>rectUb))
        return doesOverlap
    
    def isQuadEnclosed(self,rectLb,rectRb,rectDb,rectUb):
        isEnclosed = (self.rightb<rectRb)and(self.leftb>rectLb)and(self.upb<rectUb)and(self.downb>rectDb)
        return isEnclosed
        
class particle(object):
    def __init__(self,x,y):
        self.xpos = x
        self.ypos = y
        
def buildQuadTree(quadBase,recurseCount):
    
    leftOvParts = quadBase.particles
    leftRightCent = quadBase.leftb + (quadBase.rightb - quadBase.leftb)/2
    upDownCent = quadBase.downb + (quadBase.upb - quadBase.downb)/2
    
    subQuad1 = Quad(quadBase.leftb, leftRightCent, upDownCent, quadBase.upb)
    subQuad2 = Quad(leftRightCent, quadBase.rightb, upDownCent, quadBase.upb)
    subQuad3 = Quad(quadBase.leftb, leftRightCent, quadBase.downb, upDownCent)
    subQuad4 = Quad(leftRightCent, quadBase.rightb, quadBase.downb, upDownCent)
                
    if leftOvParts != None:    
        leftOvParts, subQuad1.particles = subQuad1.getPartsInQ(leftOvParts)  
        if (subQuad1.particles != []) and (len(subQuad1.particles)>1) and (recurseCount < 500):
            buildQuadTree(subQuad1,recurseCount+1)
        quadBase.childQuads[0] = subQuad1
    
        if leftOvParts != None:
            leftOvParts, subQuad2.particles = subQuad2.getPartsInQ(leftOvParts)  
            if (subQuad2.particles != []) and (len(subQuad2.particles)>1) and (recurseCount < 500):
                buildQuadTree(subQuad2,recurseCount+1)
            quadBase.childQuads[1] = subQuad2
    
            if leftOvParts != None:
                leftOvParts, subQuad3.particles = subQuad3.getPartsInQ(leftOvParts)  
                if (subQuad3.particles != []) and (len(subQuad3.particles)>1) and (recurseCount < 500):
                    buildQuadTree(subQuad3,recurseCount+1)
                quadBase.childQuads[2] = subQuad3
                if leftOvParts != None:
                    leftOvParts, subQuad4.particles = subQuad4.getPartsInQ(leftOvParts)  
                    if (subQuad4.particles != []) and (len(subQuad4.particles)>1) and (recurseCount < 500):
                        buildQuadTree(subQuad4,recurseCount+1)
                    quadBase.childQuads[3] = subQuad4
    
    return
    
def plotQuadTree(quadBase):
    if quadBase.childQuads[0] != None:
        leftRightCent = quadBase.leftb + (quadBase.rightb - quadBase.leftb)/2
        upDownCent = quadBase.downb + (quadBase.upb - quadBase.downb)/2
        
        plt.plot([quadBase.leftb,quadBase.rightb],[upDownCent,upDownCent],'k')
        plt.plot([leftRightCent,leftRightCent],[quadBase.downb,quadBase.upb],'k')
        
        plotQuadTree(quadBase.childQuads[0])
        plotQuadTree(quadBase.childQuads[1])
        plotQuadTree(quadBase.childQuads[2])
        plotQuadTree(quadBase.childQuads[3])
    

numParticles = 1000
particles = []
xlims = [0,60]
ylims = [0,30]

for i in range(0,numParticles-1):
    particles.append(particle(random()*(xlims[1]-xlims[0])+xlims[0],random()*(ylims[1]-ylims[0])+ylims[0]))

plt.plot([particleX.xpos for particleX in particles], [particleY.ypos for particleY in particles],'x')

initialQuad = Quad(xlims[0],xlims[1],ylims[0],ylims[1])
initialQuad.particles = particles

buildQuadTree(initialQuad,0)
plotQuadTree(initialQuad)
plt.gca().set_aspect('equal', 'box')
