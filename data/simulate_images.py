import numpy as np
import pybullet as pb
import pybullet_data
import time
import pybullet_data
import os, glob, random

def start_simulation():
    # Start physics environment
    physicsClient = pb.connect(pb.GUI)
    # Create floor
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = pb.loadURDF('plane.urdf')
    # Set gravity and simulation time params
    pb.setGravity(0, 0, -9.8)
    pb.setRealTimeSimulation(1)


def add_chessboard():
    # Create visual shape for chessboard
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,fileName='solid_board.obj',meshScale=[1,1,1])
    # Create collision mesh for chessboard
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName='solid_board.obj',meshScale=[1,1,1])
    # Create actual pybullet object from visual and collion shapes
    multiBodyId = pb.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=collisionShapeId, 
        baseVisualShapeIndex=visualShapeId,
        basePosition=[0,0,1],
        baseOrientation=pb.getQuaternionFromEuler([np.pi/2,0,0]))
    # Add chessboard texture to object
    textureId = pb.loadTexture('chessboard_demo.png')
    pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)
    # Set friction for the chess board to a reasonable level so it does not go spinning away in the simulator
    pb.changeDynamics(multiBodyId,-1,linearDamping=0, angularDamping=0, rollingFriction=1, spinningFriction=1)

def add_pieces(piece, square):
    return

if __name__ == "__main__":
    start_simulation()
    add_chessboard()
    while pb.isConnected():
        pb.stepSimulation()
        # pb.getCameraImage(320,200, renderer=pb.ER_BULLET_HARDWARE_OPENGL)