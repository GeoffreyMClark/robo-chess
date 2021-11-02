# To do:
#  3. add randomization of lighting conditions function
#  4. check intrinsic camera setting to make sure they match the pycamera V2


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

def positionConversion(square):
    return [(square[1]*.05)-.175,(square[0]*.0514)-.18,0.05]

def addPieces(color, piece, square):
    # position conversion
    position=positionConversion(square)
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

def movePiece(piece_id, new_square):
    # position conversion
    position=positionConversion(new_square)
    rotation = np.random.uniform(0, 2*np.pi)
    orientation = pb.getQuaternionFromEuler([np.pi/2,0,rotation])
    multiBodyId = pb.resetBasePositionAndOrientation(int(piece_id), position, orientation)

def moveCameraRandom():
    theta = np.random.uniform(0, 2*np.pi) # camera angular position around the board
    r = np.random.uniform(0.1, 1.0) # camera distance from center of board
    phi = np.random.uniform(.1*np.pi, .4*np.pi) # camera angular position above the board
    # theta = 3*np.pi/2
    # r=0.5
    # phi=.2*np.pi
    
    
    camera_coordinates = [np.cos(phi)*np.sin(theta), np.cos(phi)*np.cos(theta), np.sin(phi)]
    # camera_target = [np.random.uniform(-.1, .1), np.random.uniform(-.1, .1), np.random.uniform(-.1, .1)]
    yaw=-(theta+np.pi)*180/np.pi
    pitch=-phi*180/np.pi#-phi*180/np.pi
    roll=0#180 if phi > np.pi/2 else 0
    viewMatrix = pb.computeViewMatrixFromYawPitchRoll(
        camera_coordinates, r, yaw, pitch, roll, 2)
    projectionMatrix = pb.computeProjectionMatrixFOV(
        fov=62.2, aspect=4/3, nearVal=0.01, farVal=20.0)
    return viewMatrix, projectionMatrix

def changeLightingParams():
    direction=[0,0,0]
    color=[np.random.uniform(0,1), np.random.uniform(0,1), np.random.uniform(0,1)]
    ambient=0.0
    diffuse=0.0
    specular=0.0
    return direction, color, ambient, diffuse, specular


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

    def randomMove(self):
        # select random piece, camera_coordinates, yaw, pitch, roll
        all_pieces = self.state_ids[self.state_ids!=0]
        piece_id = np.random.choice(all_pieces)
        old_location = np.where(self.state_ids==piece_id)
        # select empty space
        empty_locations = np.asarray(np.where(self.state_ids == 0))
        location_select = np.random.randint(0,empty_locations[0].shape[0])
        move_location = empty_locations[:, location_select]
        # move piece to new position
        movePiece(piece_id, move_location)
        self.state_ids[old_location[0],old_location[1]] = 0
        self.state_ids[move_location[0], move_location[1]] = piece_id



if __name__ == "__main__":
    startSimulation()
    addChessboard()
    board = BoardState()
    count=0
    while pb.isConnected():
        # pb.stepSimulation()
        count+=1
        board.randomMove()
        viewMatrix, projectionMatrix = moveCameraRandom()
        dir, color, amb, dif, spec = changeLightingParams()

        width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
            width=3280, height=2464,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix,
            lightDirection=dir,
            lightColor=color,
            shadow=0,
            lightAmbientCoeff=amb,
            lightDiffuseCoeff=dif,
            lightSpecularCoeff=-1000.,
            renderer=pb.ER_BULLET_HARDWARE_OPENGL)

        pass