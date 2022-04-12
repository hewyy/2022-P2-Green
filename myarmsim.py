#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
import sys
import os
from numpy import linspace,dot,zeros,pi,asarray,meshgrid,ones,c_,save,load,array, rad2deg
from p2sim import ArmAnimatorApp
from arm import Arm
from joy.decl import KEYDOWN,K_k,K_o, K_DOWN, K_UP
from joy import progress, JoyApp
from move import Move


class MyArm(ArmAnimatorApp):
    def __init__(self,Tp2ws,x,y,s,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      
      #first three columns represent axis. Last column represents
      #translation. Adjust last column to adjust workspace placement relative to arm.
      #Base of arm is always at world origin 
      Tws2w = asarray([
           [1,0,0, -5], 
           [0,1,0, -12],
           [0,0,1,0],
           [0,0,0,  1]
      ])
    
      self.square_pos_x = x
      self.square_pos_y = y
      self.square_scale = s
      ###
      ### Arm specification
      ###

      armSpec = asarray([
        [0,0.02,1,0,-1.57], # base rotation around the z-axis
        [0,1,0,10,-1.57], # arm rotation around the y-axis #1.57
        [0,1,0,10,0] #the arm extending/unextending 
      ]).T
      self.armSpec = armSpec
                      
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws,simTimeStep=0.25,**kw)
      
      self.idealArm = Arm(armSpec)  #create instance of arm class without real-life properties
      self.move = Move(self)     #move plan
     
    def save_cal(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      
      #plot square corners on paper in red
      for point in self.square_w:
          fvp.plot3D(*point[:-1],marker='o',color='r')
      
      #plot steps robot arm will take from current position to next corner/calibration point in green
      for i,point in enumerate(self.move.steps):
          if self.currStep and self.currStep == i:
            fvp.plot3D(*point,marker='o',color='b')     #dot that robot is currently trying to move to is blue
          else:
            fvp.plot3D(*point,marker='o',color='g')
      #plot calibration grid points in magenta
      for point in self.calib_grid:
          fvp.plot3D(*point[:-1],marker='o',color='m')
      return ArmAnimatorApp.show(self,fvp)


    # Define 4 corners of square in paper coordinates
    # columns(left to right): x,y,z coordinates, 1's
    # scale s represents full length of square side
    def createSquare(self, x,y,s):
      square_p = asarray([
        [x-0.5*s, y+0.5*s, 0, 1],     #upper left
        [x+0.5*s, y+0.5*s, 0, 1],     #uper right
        [x+0.5*s, y-0.5*s, 0, 1],     #bottom right
        [x-0.5*s, y-0.5*s, 0, 1]      #bottom left
        ])

      #Convert all coordinates for square to world coordinates
      return dot(square_p, self.Tp2w.T)

    #Create calibration grid on paper. These are points to move to during calibration.
    def createGrid(self, x_spacing, y_spacing):
      # create the array for the x points
      x_lin = [0]
      while x_lin[-1]+x_spacing*0.394 < 8:
        x_lin.append(x_lin[-1]+x_spacing*0.394)
      
      # create the array for the y points
      y_lin = [0]
      while y_lin[-1]+y_spacing*0.394 < 11:
        y_lin.append(y_lin[-1]+y_spacing*0.394)      

      nx, ny = (len(x_lin),len(y_lin))     #can be adjusted to add more calibration points
      xv,yv = meshgrid(x_lin,y_lin,indexing='xy')
      grid = c_[xv.reshape(nx*ny,1), yv.reshape(nx*ny,1), zeros((nx*ny,1)), ones((nx*ny,1))]
      grid_idx = list(range(nx*ny))
      for i in range(nx-(nx % 2)):
          idx = nx*(2*i+1)
          grid_idx[idx:idx+nx] = grid_idx[idx:idx+nx][::-1]
      return dot(grid,self.Tp2w.T), nx, ny # test
      
    def onStart(self):
      ArmAnimatorApp.onStart(self)
      self.calib_grid, self.nx, self.ny = self.createGrid(4,4)
      self.calib_idx = 0
      self.square_w = self.createSquare(self.square_pos_x, self.square_pos_y,self.square_scale)
      #if calibration file exists, load calibration array in here, and skip over next part
      #also set calibrated == true so that it calculates offset
      #manually delete existing calibration array file before moving on to new arena
      if(os.path.exists("calib_array.npy")):
          self.calib_ang = load("calib_array.npy")
          print(type(self.calib_ang))
          self.move.calibrated = True
          print(self.calib_ang[:,:-1]) 
      else:
          #This is the matrix you save your angles in and use to calculate angle offset
          self.calib_ang = zeros((self.nx*self.ny,len(self.arm)))    

    def onEvent(self,evt):
      if evt.type == KEYDOWN:
          progress('in keydown')
          if evt.key == K_DOWN:
            self.draw = True
            progress('pen enabled')
            return
          if evt.key == K_UP:
            self.draw = False
            progress('pen disabled')
            return
          p = "wert".find(evt.unicode)
          if p>=0:
              if self.move.isRunning():
                  progress('Move running!')
                  return
              self.move.moveArm.angles = self.calib_ang[p]
              self.move.start()
              self.move.square = True
              #after each move, set the previous goal position as your new starting position
              self.move.currentPos = self.move.pos
              progress('Move plan started!')
              return
          #Press 'k' to move to grid point for calibration
          if evt.key == K_k:
              self.move.pos = self.calib_grid[self.calib_idx]    #set next grid point as goal position
              self.move.start()
              progress('Moving to calibration point')
              for i,motor in enumerate(self.arm):
                  self.calib_ang[self.calib_idx,i] = motor.get_goal()*(pi/18000) #convert angles from centidegrees to radians
              progress("here")
              return
          #Manual adjustment before this step  
          #Press 'o' to calculate new angle
          if evt.key == K_o:
              #Calculate error
              progress('Calculate error')
              real_ang = zeros(len(self.arm))
              for i,motor in enumerate(self.arm):
                real_ang[i] = motor.get_pos()*(pi/18000)   
              self.calib_ang[self.calib_idx] = real_ang
              self.calib_idx += 1
              if self.calib_idx == len(self.calib_ang):
                  progress('Calibration_complete!')
                  self.move.calibrated = True
                  save("calib_array.npy",self.calib_ang)    #save calibration array
              return
          #manual movements
          # row of 'a' on QWERTY keyboard increments motors
          p = "asdf".find(evt.unicode)
          if p>=0:
            progress('manual move')
            self.arm[p].set_pos(self.arm[p].get_goal() + 100)
            return
          # row of 'z' in QWERTY keyboard decrements motors
          p = "zxcv".find(evt.unicode)
          if p>=0:
            progress('manual move')
            self.arm[p].set_pos(self.arm[p].get_goal() - 100)
            return
      return ArmAnimatorApp.onEvent(self,evt)


if __name__=="__main__": 
  from sys import argv, stdout, exit

# Transform of paper coordinates to workspace
  Tp2ws=asarray([[  1.  ,   0.  ,   0.  ,   0.16],
       [  0.  ,   0.71,   0.71,   1.92],
       [  0.  ,  -0.71,   0.71,  10.63],
       [  0.  ,   0.  ,   0.  ,   1.  ]])

  x,y,s = 4,8,2
    #Initial test
    # 
  app = MyArm(Tp2ws,x,y,s
                    ## Uncomment the next line (cfg=...) to save video frames;
                    ## you can use the frameViewer.py program to view those
                    ## frames in real time (they will not display locally)
                    #      cfg=dict(logVideo="f%04d.png")
                    )
  app.run()