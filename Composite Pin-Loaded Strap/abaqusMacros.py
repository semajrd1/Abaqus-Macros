# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__

def STRAP():
    # Import additional Abaqus modules used in this script
    import section
    import regionToolset
    import displayGroupMdbToolset as dgm
    import part
    import material
    import assembly
    import step
    import interaction
    import load
    import mesh
    import optimization
    import job
    import sketch
    import visualization
    import xyPlot
    import displayGroupOdbToolset as dgo
    import connectorBehavior

    # ======================================================EDIT THE PARAMETERS BELOW AS REQUIRED===========================================================
    # Define input parameters for the strap and pin geometry, material properties, and meshing

    # Pin radius (for the circular pin)
    PIN_RADIUS = 10.0
    # Distance between the two pins
    PIN_TO_PIN_LENGTH = 250.0
    # Thickness of each ply in the strap
    PLY_THICKNESS = 0.167
    # Vertical distance from the centre of the upper pin to the start of the strap (ply)
    PLY_STARTING_LOCATION = 50.0
    # Number of plies which form the strap
    NUMBER_OF_PLIES = 6

    # Ply material properties (engineering constants for an orthotropic material)
    E1 = 112479.9 
    E2 = 7230.6 
    E3 = 7230.6 
    v12 = 0.33 
    v13 = 0.33 
    v23 = 0.33 
    G12 = 2580.9 
    G13 = 2580.9
    G23 = 1701.3

    # Pin material properties (assumed isotropic)
    E_pin = 200000.0
    v_pin = 0.33

    # Applied displacement magnitude (for loading the pin)
    applied_disp = 3.0

    # (Please edit meshing parameters with caution)
    # Number of elements to seed around the curved region of the strap
    NUMBER_OF_ELEMENTS_CURVED_REG = 50
    # Global mesh density (approximate element size) for the strap (ply)
    GLOBAL_MESH_DENSITY_PLY = 0.8
    # Global mesh density for the pin
    GLOBAL_MESH_DENSITY_PIN = 1.5

    # ======================================================================================================================================================
    # Dictionary to store cohesive zone (CZ) surface pairs for later contact property assignments
    CZ_SURFACES_DICT = {}

    # Loop over each ply to create the strap geometry parts
    for i in range(NUMBER_OF_PLIES):
        # Temporary ply index and name assignment
        PLY_INDEX_TEMP = i
        NAME = 'Ply-' + str(PLY_INDEX_TEMP+1)

        # Calculate geometric parameters based on ply index and user-defined parameters
        A = PIN_RADIUS + PLY_INDEX_TEMP * PLY_THICKNESS
        B = PIN_TO_PIN_LENGTH / 2
        C = A + PLY_THICKNESS
        D = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP+0) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)
        E = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP+1) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)
        F = PLY_STARTING_LOCATION
        G = PIN_TO_PIN_LENGTH
        H = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP-1) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)

        # --------------------- Create the upper half of the ply (Part A) ---------------------
        # Create a constrained sketch for the upper half cross-sectional profile
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.setPrimaryObject(option=STANDALONE)
        
        # Draw vertical and horizontal lines to define the profile
        s.Line(point1=(-A, 0.0), point2=(-A, -B))
        s.VerticalConstraint(entity=g[2], addUndoState=False)
        s.Line(point1=(-A, -B), point2=(-C, -B))
        s.HorizontalConstraint(entity=g[3], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[2], entity2=g[3], addUndoState=False)
        s.Line(point1=(-C, -B), point2=(-C, 0.0))
        s.VerticalConstraint(entity=g[4], addUndoState=False)
        s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
        
        # Draw arcs representing the curved boundaries of the pin/strap interface
        s.ArcByCenterEnds(center=(0.0, 0.0), point1=(-A, 0.0), point2=(A, 0.0), direction=CLOCKWISE)
        s.ArcByCenterEnds(center=(0.0, 0.0), point1=(-C, 0.0), point2=(C, 0.0), direction=CLOCKWISE)

        # Draw lines to complete the upper half outline
        s.Line(point1=(A, 0.0), point2=(D, -F))
        s.Line(point1=(D, -F), point2=(E, -F))
        s.HorizontalConstraint(entity=g[8], addUndoState=False)
        s.Line(point1=(E, -F), point2=(C, 0.0))

        # Create a 3D deformable part by extruding the sketch for the upper half (Part A)
        p = mdb.models['Model-1'].Part(name=NAME+'A', dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = mdb.models['Model-1'].parts[NAME+'A']
        p.BaseSolidExtrude(sketch=s, depth=6.0)
        s.unsetPrimaryObject()
        p = mdb.models['Model-1'].parts[NAME+'A']

        # Clean up the sketch after extrusion
        del mdb.models['Model-1'].sketches['__profile__']

        # --------------------- Create the lower half of the ply (Part B) ---------------------
        # Create a new sketch for the lower half profile
        s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        s1.setPrimaryObject(option=STANDALONE)

        # Draw lines and constraints for the lower half (mirroring the upper half)
        s1.Line(point1=(-A, -G), point2=(-A, -B))
        s1.VerticalConstraint(entity=g[2], addUndoState=False)
        s1.Line(point1=(-A, -B), point2=(-C, -B))
        s1.HorizontalConstraint(entity=g[3], addUndoState=False)
        s1.PerpendicularConstraint(entity1=g[2], entity2=g[3], addUndoState=False)
        s1.Line(point1=(-C, -B), point2=(-C, -G))
        s1.VerticalConstraint(entity=g[4], addUndoState=False)
        s1.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    
        # Create arcs for the lower half profile
        s1.ArcByCenterEnds(center=(0.0, -G), point1=(-A, -G), point2=(A, -G), direction=COUNTERCLOCKWISE)
        s1.ArcByCenterEnds(center=(0.0, -G), point1=(-C, -G), point2=(C, -G), direction=COUNTERCLOCKWISE)
        
        # Draw lines to complete the lower half outline
        s1.Line(point1=(A, -G), point2=(H, -F))
        s1.Line(point1=(H, -F), point2=(D, -F))
        s1.HorizontalConstraint(entity=g[8], addUndoState=False)
        s1.Line(point1=(D, -F), point2=(C, -G))
        
        # Create a 3D deformable part by extruding the lower half sketch (Part B)
        p = mdb.models['Model-1'].Part(name=NAME+'B', dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = mdb.models['Model-1'].parts[NAME+'B']
        p.BaseSolidExtrude(sketch=s1, depth=6.0)
        s1.unsetPrimaryObject()
        p = mdb.models['Model-1'].parts[NAME+'B']
        session.viewports['Viewport: 1'].setValues(displayedObject=p)
        del mdb.models['Model-1'].sketches['__profile__']
    
    # --------------------- Define Material and Section for the Ply ---------------------
    mdb.models['Model-1'].Material(name='Material-1')
    mdb.models['Model-1'].materials['Material-1'].Elastic(
        type=ENGINEERING_CONSTANTS, table=((E1, E2, E3, v12, v13, v23, G12, G13, G23), ))

    mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1', material='Material-1', thickness=None)

    # --------------------- Assembly, Orientation, and Meshing for Each Ply ---------------------
    for i in range(NUMBER_OF_PLIES):
        PLY_INDEX_TEMP = i
        NAME = 'Ply-' + str(PLY_INDEX_TEMP+1)

        # Recalculate geometry variables (same as before) for current ply
        A = PIN_RADIUS + PLY_INDEX_TEMP * PLY_THICKNESS
        B = PIN_TO_PIN_LENGTH / 2
        C = A + PLY_THICKNESS
        D = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP+0) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)
        E = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP+1) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)
        F = PLY_STARTING_LOCATION
        G = PIN_TO_PIN_LENGTH
        H = (PLY_STARTING_LOCATION + (PIN_TO_PIN_LENGTH / PLY_THICKNESS) * (PIN_RADIUS + (PLY_INDEX_TEMP-1) * PLY_THICKNESS)) / (PIN_TO_PIN_LENGTH / PLY_THICKNESS)

        # Create instances for both upper (A) and lower (B) halves of the ply in the assembly
        a = mdb.models['Model-1'].rootAssembly
        a1 = mdb.models['Model-1'].rootAssembly
        a1.DatumCsysByDefault(CARTESIAN)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        a1.Instance(name='Ply-' + str(i+1) + 'A-1', part=p, dependent=ON)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        a1.Instance(name='Ply-' + str(i+1) + 'B-1', part=p, dependent=ON)

        # --------------------- Create Tie Constraint for the Middle Surfaces ---------------------
        # Tie the mating surfaces of the two halves of the ply to ensure continuity
        a = mdb.models['Model-1'].rootAssembly
        s1 = a.instances['Ply-' + str(i+1) + 'B-1'].faces
        side1Faces1 = s1.findAt(((-(PIN_RADIUS + PLY_THICKNESS * PLY_INDEX_TEMP + PLY_THICKNESS/2), -PIN_TO_PIN_LENGTH/2, 3.0), ))
        region1 = a.Surface(side1Faces=side1Faces1, name=NAME + '_M_SURF_1')
        a = mdb.models['Model-1'].rootAssembly
        s1 = a.instances['Ply-' + str(i+1) + 'A-1'].faces
        side1Faces1 = s1.findAt(((-(PIN_RADIUS + PLY_THICKNESS * PLY_INDEX_TEMP + PLY_THICKNESS/2), -PIN_TO_PIN_LENGTH/2, 3.0), ))
        region2 = a.Surface(side1Faces=side1Faces1, name=NAME + '_S_SURF_1')
        mdb.models['Model-1'].Tie(name=NAME + 'Middle_Constraint', master=region1, slave=region2, 
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)

        # --------------------- Define Material Orientation for Part A ---------------------
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        c = p.cells
        cells = c.findAt(((-A, 0.0, 3.0), ))
        region = regionToolset.Region(cells=cells)

        # Create a surface to define the normal axis
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        s = p.faces
        side1Faces = s.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -PIN_TO_PIN_LENGTH/4, 3.0), ),
                               ((0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 3.0), ),
                               (((A + D)/2, -F/2, 3.0), ))
        normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-1')

        # Create an edge set for the primary axis
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        e = p.edges
        edges = e.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -PIN_TO_PIN_LENGTH/4, 0.0), ),
                          ((0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 0.0), ),
                          (((A + D)/2, -F/2, 0.0), ))
        primaryAxisRegion = p.Set(edges=edges, name='Set-1')

        # Apply the material orientation using the defined normal and primary axis regions
        mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A'].MaterialOrientation(region=region, 
            orientationType=DISCRETE, axis=AXIS_1, normalAxisDefinition=SURFACE, 
            normalAxisRegion=normalAxisRegion, flipNormalDirection=False, 
            normalAxisDirection=AXIS_3, primaryAxisDefinition=EDGE, 
            primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_1, 
            flipPrimaryDirection=True, additionalRotationType=ROTATION_NONE, 
            angle=0.0, additionalRotationField='', stackDirection=STACK_3)
        
        # --------------------- Define Material Orientation for Part B ---------------------
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        c = p.cells
        cells = c.findAt(((-A, -PIN_TO_PIN_LENGTH, 3.0), ))
        region = regionToolset.Region(cells=cells)

        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        s = p.faces
        side1Faces = s.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -(3 * PIN_TO_PIN_LENGTH)/4, 3.0), ),
                               ((0, -(PIN_TO_PIN_LENGTH + PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 3.0), ),
                               (((A + H)/2, -(G + F)/2, 3.0), ))
        normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-1')

        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        e = p.edges
        edges = e.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -(3 * PIN_TO_PIN_LENGTH)/4, 0.0), ),
                          ((0, -(PIN_TO_PIN_LENGTH + PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 0.0), ),
                          (((A + H)/2, -(G + F)/2, 0.0), ))
        primaryAxisRegion = p.Set(edges=edges, name='Set-1')
        mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B'].MaterialOrientation(region=region, 
            orientationType=DISCRETE, axis=AXIS_1, normalAxisDefinition=SURFACE, 
            normalAxisRegion=normalAxisRegion, flipNormalDirection=False, 
            normalAxisDirection=AXIS_3, primaryAxisDefinition=EDGE, 
            primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_1, 
            flipPrimaryDirection=True, additionalRotationType=ROTATION_NONE, 
            angle=0.0, additionalRotationField='', stackDirection=STACK_3)

        # --------------------- Assign Section and Mesh Part A ---------------------
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        c = p.cells
        cells = c.findAt(((-A, 0.0, 3.0), ))
        region = p.Set(cells=cells, name='Set-3')
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
        
        # Seed and generate mesh for Part A with a global seed size
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        p.seedPart(size=GLOBAL_MESH_DENSITY_PLY, deviationFactor=0.1, minSizeFactor=0.1)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        p.generateMesh()

        # Set element types for Part A cells
        elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, secondOrderAccuracy=OFF, distortionControl=DEFAULT)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
        elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
        pickedRegions = (cells, )
        p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))

        # Delete the mesh in the curved region so it can be re-meshed with refined seeding
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        c = p.cells
        pickedRegions = c.findAt(((0.0, (PIN_RADIUS + PLY_THICKNESS * PLY_INDEX_TEMP + PLY_THICKNESS/2), 6.0), ))
        p.deleteMesh(regions=pickedRegions)

        # Seed the curved edges with a finer number of elements
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        e = p.edges
        pickedEdges = e.findAt(((0.0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 6.0), ),
                               ((0.0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), 6.0), ))
        p.seedEdgeByNumber(edges=pickedEdges, number=NUMBER_OF_ELEMENTS_CURVED_REG, constraint=FINER)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'A']
        p.generateMesh()

        # --------------------- Assign Section and Mesh Part B ---------------------
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        c = p.cells
        pickedRegions = c.findAt(((0.0, -PIN_TO_PIN_LENGTH - (PIN_RADIUS + PLY_THICKNESS * PLY_INDEX_TEMP + PLY_THICKNESS/2), 6.0), ))
        p.deleteMesh(regions=pickedRegions)

        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        e = p.edges
        pickedEdges = e.findAt(((0.0, -PIN_TO_PIN_LENGTH - (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 6.0), ),
                               ((0.0, -PIN_TO_PIN_LENGTH - (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), 6.0), ))
        p.seedEdgeByNumber(edges=pickedEdges, number=NUMBER_OF_ELEMENTS_CURVED_REG, constraint=FINER)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        p.generateMesh()

        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        c = p.cells
        cells = c.findAt(((-A, -PIN_TO_PIN_LENGTH, 3.0), ))
        region = p.Set(cells=cells, name='Set-3')
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
        
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        p.seedPart(size=GLOBAL_MESH_DENSITY_PLY, deviationFactor=0.1, minSizeFactor=0.1)
        p = mdb.models['Model-1'].parts['Ply-' + str(i+1) + 'B']
        p.generateMesh()

        elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, secondOrderAccuracy=OFF, distortionControl=DEFAULT)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
        elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
        pickedRegions = (cells, )
        p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))

        # --------------------- Create Tie Constraint for Adjacent Plies (if applicable) ---------------------
        # For plies after the first, tie the starting position between the previous upper (A) and current lower (B) halves
        if PLY_INDEX_TEMP >= 1:
            a = mdb.models['Model-1'].rootAssembly
            s1 = a.instances['Ply-' + str(i) + 'A-1'].faces
            side1Faces1 = s1.findAt((((H + D)/2, -F, 3.0), ))
            region1 = a.Surface(side1Faces=side1Faces1, name=NAME + '_M_SURF_2')
            a = mdb.models['Model-1'].rootAssembly
            s1 = a.instances['Ply-' + str(i+1) + 'B-1'].faces
            side1Faces1 = s1.findAt((((H + D)/2, -F, 3.0), ))
            region2 = a.Surface(side1Faces=side1Faces1, name=NAME + '_S_SURF_2')
            mdb.models['Model-1'].Tie(name=NAME + 'Starting_Position_Constraint', master=region1, slave=region2, 
                positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
      
        # --------------------- Create Interaction Surfaces for Contact ---------------------
        # Define contact surfaces between the two halves (upper and lower) of each ply
        a = mdb.models['Model-1'].rootAssembly
        s1 = a.instances['Ply-' + str(i+1) + 'A-1'].faces
        side1Faces1 = s1.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -PIN_TO_PIN_LENGTH/4, 3.0), ),
                                ((0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 3.0), ),
                                (((A + D)/2, -F/2, 3.0), ))
        s2 = a.instances['Ply-' + str(i+1) + 'B-1'].faces
        side1Faces2 = s2.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), -(3 * PIN_TO_PIN_LENGTH)/4, 3.0), ),
                                ((0, -(PIN_TO_PIN_LENGTH + PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)), 3.0), ),
                                (((A + H)/2, -(G + F)/2, 3.0), ))
        a.Surface(side1Faces=side1Faces1 + side1Faces2, name='PLY' + str(i+1) + '_INT')

        a = mdb.models['Model-1'].rootAssembly
        s1 = a.instances['Ply-' + str(i+1) + 'A-1'].faces
        side1Faces1 = s1.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), -PIN_TO_PIN_LENGTH/4, 3.0), ),
                                ((0, (PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), 3.0), ),
                                ((PLY_THICKNESS + (A + D)/2, -F/2, 3.0), ))
        s2 = a.instances['Ply-' + str(i+1) + 'B-1'].faces
        side1Faces2 = s2.findAt(((-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), -(3 * PIN_TO_PIN_LENGTH)/4, 3.0), ),
                                ((0, -(PIN_TO_PIN_LENGTH + PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP+1)), 3.0), ),
                                ((PLY_THICKNESS + (A + H)/2, -(G + F)/2, 3.0), ))
        a.Surface(side1Faces=side1Faces1 + side1Faces2, name='PLY' + str(i+1) + '_EXT')

        # --------------------- Apply Symmetry Boundary Conditions to Each Ply ---------------------
        a = mdb.models['Model-1'].rootAssembly
        f1 = a.instances['Ply-' + str(i+1) + 'A-1'].faces
        faces1 = f1.findAt(( (-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)) - (PLY_THICKNESS/2), 0.0, 0.0), ))
        region = a.Set(faces=faces1, name='Set-SYM-Ply-' + str(i+1) + 'A-1')
        mdb.models['Model-1'].ZsymmBC(name='SYM-Ply-' + str(i+1) + 'A-1', createStepName='Initial', 
            region=region, localCsys=None)
        
        a = mdb.models['Model-1'].rootAssembly
        f1 = a.instances['Ply-' + str(i+1) + 'B-1'].faces
        faces1 = f1.findAt(( (-(PIN_RADIUS + PLY_THICKNESS * (PLY_INDEX_TEMP)) - (PLY_THICKNESS/2), -PIN_TO_PIN_LENGTH, 0.0), ))
        region = a.Set(faces=faces1, name='Set-SYM-Ply-' + str(i+1) + 'B-1')
        mdb.models['Model-1'].ZsymmBC(name='SYM-Ply-' + str(i+1) + 'B-1', createStepName='Initial', 
            region=region, localCsys=None)

        # --------------------- Save Cohesive Zone Surface Data for Adjacent Plies ---------------------
        if i > 0:
            # Dynamically create keys for the CZ surface dictionary
            key1 = 'PLY' + str(i) + 'EXT'
            key2 = 'PLY' + str(i+1) + 'INT'
            # Get the surface objects from the assembly
            value1 = mdb.models['Model-1'].rootAssembly.surfaces['PLY' + str(i) + '_EXT']
            value2 = mdb.models['Model-1'].rootAssembly.surfaces['PLY' + str(i+1) + '_INT']
            CZ_SURFACES_DICT[key1] = value1
            CZ_SURFACES_DICT[key2] = value2

    # --------------------- Define Contact Properties ---------------------
    # Create a general contact property (for normal and tangential behavior)
    mdb.models['Model-1'].ContactProperty('General Contact Property')
    mdb.models['Model-1'].interactionProperties['General Contact Property'].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON, constraintEnforcementMethod=DEFAULT)
    mdb.models['Model-1'].interactionProperties['General Contact Property'].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
        pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, 
        table=((0.33, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
        fraction=0.005, elasticSlipStiffness=None)
    # Create a cohesive contact property for the interface between plies
    mdb.models['Model-1'].ContactProperty('Cohesive Contact Property')
    mdb.models['Model-1'].interactionProperties['Cohesive Contact Property'].CohesiveBehavior()
    mdb.models['Model-1'].interactionProperties['Cohesive Contact Property'].Damage(
        initTable=((30.0, 30.0, 45.0), ), useEvolution=OFF, useStabilization=OFF)
    mdb.models['Model-1'].interactionProperties['Cohesive Contact Property'].cohesiveBehavior.setValues(
        eligibility=INITIAL_NODES)

    # Create a contact interaction for all the defined contact pairs in the assembly
    mdb.models['Model-1'].ContactStd(name='Int-1', createStepName='Initial')
    mdb.models['Model-1'].interactions['Int-1'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=ON)
    
    # --------------------- Assign Contact Properties to Surfaces ---------------------
    print(CZ_SURFACES_DICT)
    assignments_x = []
    # General contact property is assigned globally
    assignments_x.append((GLOBAL, SELF, 'General Contact Property'))
    # For each adjacent pair of plies, assign the cohesive contact property between the surfaces
    for i in range(NUMBER_OF_PLIES-1):
        assignments_x.append((CZ_SURFACES_DICT.get('PLY' + str(i+1) + 'EXT'), CZ_SURFACES_DICT.get('PLY' + str(i+2) + 'INT'), 'Cohesive Contact Property'))
    assignments_x = tuple(assignments_x)
    print(assignments_x)

    mdb.models['Model-1'].interactions['Int-1'].contactPropertyAssignments.appendInStep(
        stepName='Initial', assignments=assignments_x)

    # --------------------- Create the Pin Geometry ---------------------
    # Create a new sketch for the pin cross-section (a circle)
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(PIN_RADIUS, 0.0))
    p = mdb.models['Model-1'].Part(name='Part-7', dimensionality=THREE_D, type=DEFORMABLE_BODY)

    p = mdb.models['Model-1'].parts['Part-7']
    p.BaseSolidExtrude(sketch=s, depth=10.0)
    s.unsetPrimaryObject()

    p = mdb.models['Model-1'].parts['Part-7']
    del mdb.models['Model-1'].sketches['__profile__']

    # Create two instances of the pin (top and bottom)
    a = mdb.models['Model-1'].rootAssembly
    p = mdb.models['Model-1'].parts['Part-7']
    a.Instance(name='Part-7-1', part=p, dependent=ON)

    a = mdb.models['Model-1'].rootAssembly
    p = mdb.models['Model-1'].parts['Part-7']
    a.Instance(name='Part-7-2', part=p, dependent=ON)

    # Translate the second instance downward by the pin-to-pin length
    a = mdb.models['Model-1'].rootAssembly
    a.translate(instanceList=('Part-7-2', ), vector=(0.0, -PIN_TO_PIN_LENGTH, 0.0))

    # --------------------- Mesh the Pin ---------------------
    p = mdb.models['Model-1'].parts['Part-7']
    p.seedPart(size=GLOBAL_MESH_DENSITY_PIN, deviationFactor=0.1, minSizeFactor=0.1)

    p = mdb.models['Model-1'].parts['Part-7']
    p.generateMesh()

    # --------------------- Define Material and Section for the Pin ---------------------
    mdb.models['Model-1'].Material(name='Material-2')
    mdb.models['Model-1'].materials['Material-2'].Elastic(table=((E_pin, v_pin), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name='Section-2', material='Material-2', thickness=None)

    p = mdb.models['Model-1'].parts['Part-7']
    c = p.cells

    cells = c.findAt(((0.0, 0.0, 0.0), ))
    region = p.Set(cells=cells, name='Set-1')
    p = mdb.models['Model-1'].parts['Part-7']
    p.SectionAssignment(region=region, sectionName='Section-2', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

    # --------------------- Apply Symmetry Boundary Conditions to the Pin ---------------------
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Part-7-1'].faces
    faces1 = f1.findAt(((0.0, 0.0, 0.0), ))
    region = a.Set(faces=faces1, name='Set-SYM-TOP-PIN')
    mdb.models['Model-1'].ZsymmBC(name='SYM-TOP-PIN', createStepName='Initial', 
        region=region, localCsys=None)
    
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Part-7-2'].faces
    faces1 = f1.findAt(((0.0, -PIN_TO_PIN_LENGTH, 0.0), ))
    region = a.Set(faces=faces1, name='Set-SYM-BOTTOM-PIN')
    mdb.models['Model-1'].ZsymmBC(name='SYM-BOTTOM-PIN', createStepName='Initial', 
        region=region, localCsys=None)

    # --------------------- Create the Analysis Step ---------------------
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', 
        stabilizationMagnitude=0.01, 
        stabilizationMethod=DISSIPATED_ENERGY_FRACTION, 
        continueDampingFactors=False, adaptiveDampingRatio=0.05, 
        initialInc=0.01, minInc=1e-12, nlgeom=ON)

    # --------------------- Create and Configure the Job ---------------------
    mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=6, 
        numDomains=6, numGPUs=4)

    # --------------------- Apply Boundary Conditions on the Pin ---------------------
    # Get the root assembly
    a = mdb.models['Model-1'].rootAssembly

    # For the bottom pin instance (Part-7-2), find the edge at the specified location
    e1 = a.instances['Part-7-2'].edges
    # Locate the face at (0.0, -PIN_TO_PIN_LENGTH, 10.0) on the bottom pin (using f1 from a previous context)
    faces1 = f1.findAt(((0.0, -PIN_TO_PIN_LENGTH, 10.0), ))
    # Create a set for the bottom pin face to enforce encastre (fully fixed) BC
    region = a.Set(faces=faces1, name='Set-BOTTOM_PIN_ENCASTRE')
    mdb.models['Model-1'].EncastreBC(name='BOTTOM_PIN_ENCASTRE', createStepName='Initial', 
        region=region, localCsys=None)

    # For the top pin instance (Part-7-1), apply a displacement boundary condition
    a = mdb.models['Model-1'].rootAssembly
    # Find the face at (0.0, 0.0, 10.0) on the top pin
    f1 = a.instances['Part-7-1'].faces
    faces1 = f1.findAt(((0.0, 0.0, 10.0), ))
    # Create a set for that face
    region = a.Set(faces=faces1, name='Set-UPPER_PIN_DISP_CONST')
    # Apply a displacement BC (here u1 and u3 are set to zero, and other DOFs are left unset)
    mdb.models['Model-1'].DisplacementBC(name='UPPER_PIN_DISP_CONST', createStepName='Initial', 
        region=region, u1=0.0, u2=UNSET, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0,  
        localCsys=None)

    # Regenerate the assembly after applying the boundary conditions
    a = mdb.models['Model-1'].rootAssembly
    a.regenerate()

    # --------------------- Define Field Output Requests ---------------------
    # Set the output variables to be written to the results file during the analysis step
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
        'S',      # Stress tensor
        'PE',     # Plastic strain (if any)
        'PEEQ',   # Equivalent plastic strain
        'PEMAG',  # Plastic strain magnitude
        'LE',     # Logarithmic strain
        'U',      # Displacements
        'RF',     # Reaction forces
        'CSTRESS',# Contact stress
        'CDISP',  # Contact displacements
        'ENER',   # Energy quantities
        'CSMAXSCRT', 
        'CSMAXUCRT',
        'CSQUADSCRT',
        'CSQUADUCRT'))

    # --------------------- Define Step Controls ---------------------
    # Modify the time incrementation parameters in the analysis step 'Step-1'
    mdb.models['Model-1'].steps['Step-1'].control.setValues(allowPropagation=OFF, 
        resetDefaultValues=OFF, timeIncrementation=(4.0, 8.0, 9.0, 16.0, 10.0, 
        4.0, 12.0, 12.0, 6.0, 3.0, 50.0))

    # --------------------- Create a Reference Point for the MPC ---------------------
    # Create a reference point on the top pin (Part-7-1) using an interesting point on one of its edges
    a = mdb.models['Model-1'].rootAssembly
    e1 = a.instances['Part-7-1'].edges
    a.ReferencePoint(point=a.instances['Part-7-1'].InterestingPoint(edge=e1.findAt(coordinates=(0.0, PIN_RADIUS, 10)), rule=CENTER))

    # Retrieve the created reference point from the assembly
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    # r1.keys() returns the dictionary keys for the reference points; pick the first one
    refPoints1 = (r1[r1.keys()[0]], )
    # Create a set with the reference point (this set will serve as the control point in the MPC)
    region1 = a.Set(referencePoints=refPoints1, name='m_Set-17')

    # Create a set on the top pin face that will be tied to the reference point using an MPC
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Part-7-1'].faces
    faces1 = f1.findAt(((0.0, PIN_RADIUS, 10.0), ))
    region2 = a.Set(faces=faces1, name='s_Set-17')

    # --------------------- Create the Multipoint Constraint (MPC) ---------------------
    # Tie the reference point (control point) to the top pin face so that they move together
    mdb.models['Model-1'].MultipointConstraint(name='MPC_CONSTRAINT', 
        controlPoint=region1, surface=region2, mpcType=TIE_MPC, 
        userMode=DOF_MODE_MPC, userType=0, csys=None)

    # Create a set containing the reference point to later apply a displacement boundary condition
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    refPoints1 = (r1[r1.keys()[0]], )
    region = a.Set(referencePoints=refPoints1, name='Set-19')

    # --------------------- Apply the Displacement Boundary Condition ---------------------
    # In step 'Step-1', apply the prescribed displacement (applied_disp) to the reference point
    mdb.models['Model-1'].DisplacementBC(name='BC-APPLIED_DISP', createStepName='Step-1', 
        region=region, u1=UNSET, u2=applied_disp, u3=UNSET, ur1=UNSET, ur2=UNSET, 
        ur3=UNSET, amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, 
        fieldName='', localCsys=None)
