import numpy as np
import pybullet as pb
import pybullet_data
import time
import pybullet_data

def start_simulation():
    # Start physics environment
    physicsClient = pb.connect(pb.GUI)
    # Create floor
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = pb.loadURDF('plane.urdf')
    # Set gravity and simulation time params
    pb.setGravity(0, 0, -9.8)
    # pb.setRealTimeSimulation(1)


def add_chessboard():
    # Create visual shape for chessboard
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,fileName='Board/solid_board.obj')
    # Create collision mesh for chessboard
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName='Board/solid_board.obj')
    # Create actual pybullet object from visual and collion shapes
    multiBodyId = pb.createMultiBody(
        baseMass=100.0,
        baseCollisionShapeIndex=collisionShapeId, 
        baseVisualShapeIndex=visualShapeId,
        basePosition=[0.0,0.0,-2.7],
        baseOrientation=pb.getQuaternionFromEuler([np.pi/2,0,np.pi])
        )
    # Add chessboard texture to object
    textureId = pb.loadTexture('Board/chessboard_demo.png')
    pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)
    # Set friction for the chess board to a reasonable level so it does not go spinning away in the simulator
    pb.changeDynamics(multiBodyId,-1,linearDamping=100, angularDamping=100, rollingFriction=200, spinningFriction=200)





def add_pieces(color, piece, square):
    # position conversion
    position=[(square[1]*.05)-.175,(square[0]*.0514)-.18,0.05]
    color_dict={'B':[.3,.3,.3,1],'W':[.9,.9,.9,1]}
    C=color_dict[color]
    # Create visual shape for piece
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,fileName='Pieces/'+piece+'.obj',rgbaColor=C,specularColor=[.8,.8,.8],meshScale=[.001,.001,.001])
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName='Pieces/'+piece+'.obj',meshScale=[.001,.001,.001])
    # Create actual pybullet object from visual and collion shapes
    multiBodyId = pb.createMultiBody(
        baseMass=10.0,
        baseCollisionShapeIndex=collisionShapeId, 
        baseVisualShapeIndex=visualShapeId,
        basePosition=position,
        baseOrientation=pb.getQuaternionFromEuler([np.pi/2,0,0]))
    pb.changeDynamics(multiBodyId,-1, rollingFriction=200, spinningFriction=200)
    return





if __name__ == "__main__":
    start_simulation()
    add_chessboard()
    count=0
    while pb.isConnected():
        # pb.stepSimulation()
        count+=1
        if count == 10000:
            add_pieces('W','P',[1,0])
            add_pieces('W','P',[1,1])
            add_pieces('W','P',[1,2])
            add_pieces('W','P',[1,3])
            add_pieces('W','P',[1,4])
            add_pieces('W','P',[1,5])
            add_pieces('W','P',[1,6])
            add_pieces('W','P',[1,7])
            add_pieces('W','R',[0,0])
            add_pieces('W','N',[0,1])
            add_pieces('W','B',[0,2])
            add_pieces('W','Q',[0,3])
            add_pieces('W','K',[0,4])
            add_pieces('W','B',[0,5])
            add_pieces('W','N',[0,6])
            add_pieces('W','R',[0,7])

            add_pieces('B','P',[6,0])
            add_pieces('B','P',[6,1])
            add_pieces('B','P',[6,2])
            add_pieces('B','P',[6,3])
            add_pieces('B','P',[6,4])
            add_pieces('B','P',[6,5])
            add_pieces('B','P',[6,6])
            add_pieces('B','P',[6,7])
            add_pieces('B','R',[7,0])
            add_pieces('B','N',[7,1])
            add_pieces('B','B',[7,2])
            add_pieces('B','Q',[7,3])
            add_pieces('B','K',[7,4])
            add_pieces('B','B',[7,5])
            add_pieces('B','N',[7,6])
            add_pieces('B','R',[7,7])





        # pb.getCameraImage(320,200, renderer=pb.ER_BULLET_HARDWARE_OPENGL)