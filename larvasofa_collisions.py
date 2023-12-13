from stlib3.scene import ContactHeader
from stlib3.physics.rigid import RigidObject

# Units : mm and kg

def createScene(rootNode):
    
    rootNode.findData('dt').value = 0.01
    rootNode.findData('gravity').value = [0, 0, -9180] # gravity is disabled
    rootNode.addObject('VisualStyle',
                       displayFlags='showCollision showVisualModels showForceFields showInteractionForceFields'
                                     +' hideCollisionModels hideBoundingCollisionModels hideWireframe')
    
    rootNode.bbox = "-50 -50 -50 50 50 50" # field of view of the viewer

    # Required plugins
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Constraint.Lagrangian.Correction", # Needed to use components LinearSolverConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Constraint.Projective",  # Needed to use components FixedConstraint
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components ShewchukPCGLinearSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.Component.Mapping.Linear", # Needed to use components BarycentricMapping  
                            "Sofa.Component.StateContainer", # Needed to use components MechanicalObject
                            "Sofa.Component.Setting", # Needed to use components BackgroundSetting  
                            "Sofa.Component.Topology.Container.Dynamic", # Needed to use components TetrahedronSetGeometryAlgorithms,TetrahedronSetTopologyContainer  
                            "Sofa.GL.Component.Rendering3D", # Needed to use components OglModel,OglSceneFrame
                            "CGALPlugin", # Needed to use MeshGenerationFromPolyhedron
                            "SoftRobots",
                            "SofaPython3"
                        ])
    
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1]) # set background color
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight") # add a triedron to help with orientation

    # setup the animation loop
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-8)

    # add a collision pipeline
    ContactHeader(rootNode, alarmDistance=1, contactDistance=.01)

    # Add a floor
    floor = RigidObject(parent=rootNode, surfaceMeshFileName="floor.obj", rotation=[90, 0, 0], translation=[0,0,-10], isAStaticObject=True)

    # create the larva object
    larvaBody = rootNode.addChild('larvaBody')

    # load the detailed mesh
    larvaSTL = 'mesh_cuticle_simplified.stl'
    larvaBody.addObject('MeshSTLLoader', name='larvaMesh', filename=larvaSTL)
    
    # create the overall body mechanics
    bodyMechanics = larvaBody.addChild('bodyMechanics')
    
    # use CGAL to create a simplified tetrahedral mesh of the larva
    bodyMechanics.addObject('MeshGenerationFromPolyhedron', name='gen', template='Vec3d', inputPoints='@../larvaMesh.position', inputTriangles='@../larvaMesh.triangles', drawTetras='0',
                       cellSize="10", facetAngle="30", facetSize="5", cellRatio="2",   #Convergence problem if lower than 2
                       facetApproximation="1")
    
    # Create the state from the coarse mesh
    bodyMechanics.addObject('MechanicalObject', name="dofs", position="@gen.outputPoints")
    bodyMechanics.addObject('TetrahedronSetTopologyContainer', name='topo', tetrahedra='@gen.outputTetras')
    bodyMechanics.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo", drawTetrahedra="0", drawScaleTetrahedra="0.8")
    
    # Set the mechanical parameters and force fields
    bodyMechanics.addObject('UniformMass', totalMass=0.5) # add mass
    bodyMechanics.addObject('TetrahedronFEMForceField', youngModulus=180, poissonRatio=0.45) # add elasticity

    # Define resolution method
    bodyMechanics.addObject('EulerImplicitSolver') # define numerical scheme
    bodyMechanics.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d") # define solve method
    


    # Fix the tail of the larva, so that it is anchored and we can deform the larva by pulling on it
    # bodyMechanics.addObject('BoxROI', name="boxROI", box=[55, -25, -10, 80, 15, 10], drawBoxes=True) # create an anchor or selection zone (red skeleton)
    # bodyMechanics.addObject('FixedConstraint', indices="@boxROI.indices") # add the constraint
    bodyMechanics.addObject('LinearSolverConstraintCorrection') # add the constraint solver

    # Set a visual model
    visualModel = larvaBody.addChild('visualModel')
    visualModel.addObject('OglModel', name='visualMesh', src='@../larvaMesh')
    # Align the visual model on the coarse mechanical mesh
    visualModel.addObject('BarycentricMapping', input='@../bodyMechanics/dofs', output='@visualMesh')

    # Collision model
    collision = bodyMechanics.addChild('larvaCollisionModel') # collision model
    collision.addObject('TriangleSetTopologyContainer', name="Container")#, position="@../topo.position") # uncomment to get a segfault
    collision.addObject('TriangleSetTopologyModifier')
    collision.addObject('Tetra2TriangleTopologicalMapping', input="@../topo", output="@Container")

    collision.addObject('TriangleCollisionModel', selfCollision=True) # 3 types of collision
    collision.addObject('LineCollisionModel', selfCollision=True)
    collision.addObject('PointCollisionModel', selfCollision=True)

    
    
    return rootNode
