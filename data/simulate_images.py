import numpy as np
import pybullet as pb
import pybullet_data
import time
import pybullet_data

def startSimulation():
    # Start physics environment
    physicsClient = pb.connect(pb.GUI)
    # Create floor
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = pb.loadURDF('plane.urdf')
    # Set gravity and simulation time params
    pb.setGravity(0, 0, -9.8)
    # pb.setRealTimeSimulation(1)


def addChessboard():
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


def addPieces(color, piece, square):
    # position conversion
    position=[(square[1]*.05)-.175,(square[0]*.0514)-.18,0.05]
    color_dict={'B':[.3,.3,.3,1],'W':[.9,.9,.9,1]}
    C=color_dict[color]
    # Create visual shape for piece
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,fileName='Pieces/'+piece+'.obj',rgbaColor=C,specularColor=[.8,.8,.8],meshScale=[.001,.001,.001])
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName='Pieces/'+piece+'.obj',meshScale=[.001,.001,.001])
    # Create actual pybullet object from visual and collion shapes
    rotation = np.random.uniform(0, 2*np.pi)
    multiBodyId = pb.createMultiBody(
        baseMass=10.0,
        baseCollisionShapeIndex=collisionShapeId, 
        baseVisualShapeIndex=visualShapeId,
        basePosition=position,
        baseOrientation=pb.getQuaternionFromEuler([np.pi/2,0,rotation]))
    pb.changeDynamics(multiBodyId,-1, rollingFriction=200, spinningFriction=200)
    return multiBodyId


class BoardState:
    def __init__(self):
        self.state, self.state_ids = self.initializeState()
        self.stateToPieces()

    def initializeState(self):
        state = np.chararray([8,8], itemsize=2)
        state_ids = np.zeros([8,8])
        state[:] = ''
        state[0,0:8] = np.array(['WR','WN','WB','WQ','WK','WB','WN','WR'])
        state[1 ,0:8] = np.array(['WP','WP','WP','WP','WP','WP','WP','WP'])
        state[7,0:8] = np.array(['BR','BN','BB','BQ','BK','BB','BN','BR'])
        state[6 ,0:8] = np.array(['BP','BP','BP','BP','BP','BP','BP','BP'])
        return state, state_ids
        
    def stateToPieces(self):
        for j in range(8):
            for i, piece in enumerate(self.state[j,:]):
                if piece != '':
                    p = piece.decode('ascii')
                    self.state_ids[j,i] = addPieces(p[0],p[1],[j,i])
                    print(self.state_ids[j,i])
                    print(self.state_ids)
        # [[addPieces(piece[0],piece[1],[j,i]) for i, piece in enumerate(self.state[j,:])] for j in range(8)]

    def randomMove(self):
        pass




if __name__ == "__main__":
    startSimulation()
    addChessboard()
    board = BoardState()
    count=0
    while pb.isConnected():
        # pb.stepSimulation()
        count+=1



        pb.getCameraImage(320,200, renderer=pb.ER_BULLET_HARDWARE_OPENGL)