import numpy as np
import csv
from abaqus import *
from abaqusConstants import *
from odbAccess import *
import __main__

def CreateJoint():
    import math
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

    DIAMETER = int(8.0)
    HOLE_ELEM_NUM = int(30.0)
    LAYUP_J = [int(33.0), int(-33.0), int(-66.0), int(66.0), int(66.0), int(-66.0), int(-33.0), int(33.0)]

    # COMPOSITE GEOMETRY
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.rectangle(point1=(-3*DIAMETER, -6*DIAMETER -10 -50), point2=(3*DIAMETER, 6*DIAMETER))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, DIAMETER/2))
    p = mdb.models['Model-1'].Part(name='Composite', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['Composite']
    p.BaseShell(sketch=s1)
    s1.unsetPrimaryObject()
    
    # PARTITION SKETCH LOCATION
    p = mdb.models['Model-1'].parts['Composite']
    del mdb.models['Model-1'].sketches['__profile__']
    p = mdb.models['Model-1'].parts['Composite']
    f, e, d1 = p.faces, p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(0.0, (DIAMETER/2)+1.0, 0.0), normal=(0.0, 0.0, 1.0)), 
        sketchUpEdge=e.findAt(coordinates=(3*DIAMETER, 0.0, 0.0)), sketchPlaneSide=SIDE1, origin=(0.0, 0.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=247.38, gridSpacing=6.18, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)

    # PARTITION
    p = mdb.models['Model-1'].parts['Composite']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.Line(point1=(-3*DIAMETER, -6*DIAMETER -10), point2=(3*DIAMETER, -6*DIAMETER -10))
    s.HorizontalConstraint(entity=g.findAt((0.0, -6*DIAMETER -10)), addUndoState=False)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    pickedFaces = f.findAt(((0.0, (DIAMETER/2)+1.0, 0.0), ))
    e1, d2 = p.edges, p.datums
    p.PartitionFaceBySketch(sketchUpEdge=e1.findAt(coordinates=(3*DIAMETER, 0.0, 0.0)), faces=pickedFaces, sketch=s)
    s.unsetPrimaryObject()

    # PIN GEOMETRY
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, DIAMETER/2))
    p = mdb.models['Model-1'].Part(name='Pin', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['Pin']
    p.BaseSolidExtrude(sketch=s, depth=10.0)
    s.unsetPrimaryObject()

    # VTC401-T700 MATERIAL PROPERTIES
    mdb.models['Model-1'].Material(name='VTC401-T700')
    mdb.models['Model-1'].materials['VTC401-T700'].Elastic(type=LAMINA, table=((112479.9, 7230.6, 0.33, 2580.9, 2580.9, 1701.3), ))
    mdb.models['Model-1'].materials['VTC401-T700'].HashinDamageInitiation(table=((2179.9, 810.7, 32.4, 84.1, 68.8, 68.8), ))
    mdb.models['Model-1'].materials['VTC401-T700'].elastic.FailStress(table=((2179.9, 810.7, 32.4, 84.1, 68.9, 0.0, 0.0), ))

    # INF. MATERIAL PROPERTIES
    mdb.models['Model-1'].Material(name='Inf')
    mdb.models['Model-1'].materials['Inf'].Elastic(table=((999999999.0, 0.33), ))

    # FACES
    p1 = mdb.models['Model-1'].parts['Composite']
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region = regionToolset.Region(faces=faces)

    # VTC401-T700 MATERIAL PROPERTIES
    p = mdb.models['Model-1'].parts['Composite']
    s = p.faces
    side1Faces = s.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-1')
    mdb.models['Model-1'].parts['Composite'].MaterialOrientation(region=region, 
        orientationType=DISCRETE, axis=AXIS_3, normalAxisDefinition=SURFACE, 
        normalAxisRegion=normalAxisRegion, flipNormalDirection=False, 
        normalAxisDirection=AXIS_3, primaryAxisDefinition=VECTOR, 
        primaryAxisVector=(0.0, 1.0, 0.0), primaryAxisDirection=AXIS_1, 
        flipPrimaryDirection=False, additionalRotationType=ROTATION_NONE, 
        angle=0.0, additionalRotationField='')
    layupOrientation = None

    # COMPOSITE REGIONS
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region1=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region2=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region3=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region4=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region5=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region6=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region7=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region8=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    s = p.faces

    # ASSIGN PLY DEFINITIONS TO COMPOSITE REGIONS
    side1Faces = s.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-2')
    compositeLayup = mdb.models['Model-1'].parts['Composite'].CompositeLayup(
        name='CompositeLayup-1', description='', elementType=SHELL, 
        offsetType=MIDDLE_SURFACE, symmetric=False, 
        thicknessAssignment=FROM_SECTION)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, 
        thicknessType=UNIFORM, poissonDefinition=DEFAULT, temperature=GRADIENT, 
        useDensity=OFF)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-1', region=region1, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[0], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-2', region=region2, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[1], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-3', region=region3, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[2], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-4', region=region4, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[3], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-5', region=region5, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[4], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-6', region=region6, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[5], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-7', region=region7, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[6], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-8', region=region8, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[7], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, 
        additionalRotationType=ROTATION_NONE, angle=0.0, 
        additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, 
        normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, 
        normalAxisDirection=AXIS_3, flipNormalDirection=False, 
        primaryAxisDefinition=VECTOR, primaryAxisVector=(0.0, 1.0, 0.0), 
        primaryAxisDirection=AXIS_1, flipPrimaryDirection=False)

    # COMPOSITE MESH
    p = mdb.models['Model-1'].parts['Composite']
    p.seedPart(size=3.5, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models['Model-1'].parts['Composite']
    e = p.edges
    pickedEdges = e.findAt(((-DIAMETER/2, 0.0, 0.0), ))
    p.seedEdgeByNumber(edges=pickedEdges, number=HOLE_ELEM_NUM, constraint=FINER)
    p = mdb.models['Model-1'].parts['Composite']
    p.generateMesh()

    elemType1 = mesh.ElemType(elemCode=S8R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=STRI65, elemLibrary=STANDARD)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    pickedRegions =(faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))

    # PIN MESH
    p1 = mdb.models['Model-1'].parts['Pin']
    p = mdb.models['Model-1'].parts['Pin']
    p.seedPart(size=0.25, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models['Model-1'].parts['Pin']
    e = p.edges
    pickedEdges = e.findAt(((DIAMETER/2, 0.0, 10.0), ))
    p.seedEdgeByNumber(edges=pickedEdges, number=60, constraint=FINER)
    p = mdb.models['Model-1'].parts['Pin']
    p.generateMesh()

    # PIN SECTION
    mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1', material='Inf', thickness=None)
    p = mdb.models['Model-1'].parts['Pin']
    c = p.cells
    cells = c.findAt(((0.0, 0.0, 0.0), ))
    region = p.Set(cells=cells, name='Set-2')
    p = mdb.models['Model-1'].parts['Pin']
    p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

    # ASSEMBLY
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts['Composite']
    a.Instance(name='Composite-1', part=p, dependent=ON)
    a = mdb.models['Model-1'].rootAssembly
    p = mdb.models['Model-1'].parts['Pin']
    a.Instance(name='Pin-1', part=p, dependent=ON)
    a = mdb.models['Model-1'].rootAssembly
    a.translate(instanceList=('Pin-1', ), vector=(0.0, 0.0, -5.0))

    # CONTACT SURFACES
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Composite-1'].faces
    side1Faces1 = s1.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    a.Surface(side1Faces=side1Faces1, name='Surf-1')
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Pin-1'].faces
    side1Faces1 = s1.findAt(((-DIAMETER/2, 0.0, -1.0), ))
    a.Surface(side1Faces=side1Faces1, name='Surf-2')

    # STEP-1
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', maxNumInc=10000000, initialInc=0.002, minInc=1e-15)
    mdb.models['Model-1'].steps['Step-1'].control.setValues(allowPropagation=OFF, 
        resetDefaultValues=OFF, timeIncrementation=(4.0, 8.0, 9.0, 16.0, 10.0, 
        4.0, 12.0, 15.0, 6.0, 3.0, 50.0))
    mdb.models['Model-1'].steps['Step-1'].setValues(timeIncrementationMethod=FIXED, noStop=OFF)

    # REFERENCE POINT
    a = mdb.models['Model-1'].rootAssembly
    e1 = a.instances['Pin-1'].edges
    a.ReferencePoint(point=a.instances['Pin-1'].InterestingPoint(edge=e1.findAt(coordinates=(DIAMETER/2, 0.0, 5.0)), rule=CENTER))

    # INTERACTION
    mdb.models['Model-1'].ContactProperty('IntProp-1')
    mdb.models['Model-1'].interactionProperties['IntProp-1'].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
        pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, 
        table=((0.1, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
        fraction=0.005, elasticSlipStiffness=None)
    mdb.models['Model-1'].interactionProperties['IntProp-1'].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON, 
        constraintEnforcementMethod=DEFAULT)
    mdb.models['Model-1'].StdInitialization(name='CInit-1')
    mdb.models['Model-1'].ContactStd(name='Int-1', createStepName='Initial')
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    r12=mdb.models['Model-1'].rootAssembly.surfaces['Surf-2']
    mdb.models['Model-1'].interactions['Int-1'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=OFF, addPairs=((r11, r12), ))
    mdb.models['Model-1'].interactions['Int-1'].contactPropertyAssignments.appendInStep(
        stepName='Initial', assignments=((GLOBAL, SELF, 'IntProp-1'), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].initializationAssignments.appendInStep(
        stepName='Initial', assignments=((r11, r12, 'CInit-1'), ))
    mdb.models['Model-1'].interactions['Int-1'].surfaceThicknessAssignments.appendInStep(
        stepName='Initial', assignments=((r11, ORIGINAL, 1.0), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].surfaceOffsetAssignments.appendInStep(
        stepName='Initial', assignments=((r11, ORIGINAL), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].surfaceFeatureAssignments.appendInStep(
        stepName='Initial', assignments=((r11, PERIMETER), ))

    # MPC-CONSTRAINT
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    refPoints1=(r1[r1.keys()[0]], )
    region1=a.Set(referencePoints=refPoints1, name='m_Set-1')
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Pin-1'].faces
    faces1 = f1.findAt(((DIAMETER/2, 0.0, 1.0), ))
    region2=a.Set(faces=faces1, name='s_Set-1')
    mdb.models['Model-1'].MultipointConstraint(name='Constraint-1', 
        controlPoint=region1, surface=region2, mpcType=TIE_MPC, 
        userMode=DOF_MODE_MPC, userType=0, csys=None)

    # LOADING
    mdb.models['Model-1'].SmoothStepAmplitude(name='Amp-1', timeSpan=STEP, data=((0.0, 0.0), (1.0, 1.0)))
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    refPoints1=(r1[8], )
    region = a.Set(referencePoints=refPoints1, name='rpset')
    mdb.models['Model-1'].DisplacementBC(name='BC-disp', createStepName='Step-1', 
        region=region, u1=UNSET, u2=0.5, u3=UNSET, ur1=UNSET, ur2=UNSET, 
        ur3=UNSET, amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, 
        fieldName='', localCsys=None)


    # PIN BOUNDARY CONDITION
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Pin-1'].faces
    faces1 = f1.findAt(((-DIAMETER/2, 0.0, -1.0), ))
    region = a.Set(faces=faces1, name='Set-4')
    mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Initial', 
        region=region, u1=SET, u2=UNSET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', 
        localCsys=None)

    # TAB BOUNDARY CONDITION
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Composite-1'].faces
    faces1 = f1.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ))
    region = a.Set(faces=faces1, name='Set-5')
    mdb.models['Model-1'].EncastreBC(name='BC-2', createStepName='Initial', 
        region=region, localCsys=None)

    # FIELD OUTPUT REQUEST
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'CSTRESS', 'CDISP', 'CFAILURE', 
    'DMICRT', 'HSNFTCRT', 'HSNFCCRT', 'HSNMTCRT', 'HSNMCCRT'))

    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
        createStepName='Step-1', variables=('CFAILURE', 'DMICRT', 'HSNFTCRT', 'HSNFCCRT', 
        'HSNMTCRT', 'HSNMCCRT'), layupNames=('Composite-1.CompositeLayup-1', ), 
        layupLocationMethod=ALL_LOCATIONS, rebar=EXCLUDE)

    # CREATE JOB
    mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=60, 
        memoryUnits=GIGA_BYTES, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=6, 
        numDomains=6, numGPUs=4)

    # mdb = openMdb(pathName="C:\Users\s1232851\Desktop\Mesh_convergence\Reference\APA5-8mm\CAE_Template")
    D=DIAMETER
    MAG_LAYUP=[]
    for i in range(len(LAYUP_J)):
        MAG_LAYUP.append(math.sqrt(LAYUP_J[i]**2))
    print(MAG_LAYUP)

    APA=sum(MAG_LAYUP)/len(MAG_LAYUP)
    print(APA)
    PROBE_D=int((99*(D-2))/8)
    print(PROBE_D)
    PROBE_APA=int((APA/90)*99)
    print(PROBE_APA)

    with open('new_char_curve_compression.csv', 'rb') as csvfile:
        char_curve_comp = list(csv.reader(csvfile))
    with open('new_char_curve_tension.csv', 'rb') as csvfile:
        char_curve_tension = list(csv.reader(csvfile))
    d_C=float(char_curve_comp[PROBE_APA][PROBE_D])
    d_T=float(char_curve_tension[PROBE_APA][PROBE_D])
    print('d_C and d_T: direct from matlab')
    print(d_C)
    print(d_T)

    with open('new_ratio_curve_compression.csv', 'rb') as csvfile:
        ratio_curve_comp = list(csv.reader(csvfile))
    with open('new_ratio_curve_tension.csv', 'rb') as csvfile:
        ratio_curve_tension = list(csv.reader(csvfile))
    ratio_C_temp=float(ratio_curve_comp[PROBE_APA][PROBE_D])
    ratio_T_temp=float(ratio_curve_tension[PROBE_APA][PROBE_D])
    KSI_C=math.sqrt((math.sqrt(-ratio_C_temp*(23*ratio_C_temp-24))/ratio_C_temp)-1)/math.sqrt(6)
    KSI_T=math.sqrt((math.sqrt(-ratio_T_temp*(23*ratio_T_temp-24))/ratio_T_temp)-1)/math.sqrt(6)
    d_C=(DIAMETER/2)*((1/KSI_C)-1)
    d_T=(DIAMETER/2)*((1/KSI_T)-1)
    print('d_C and d_T: from matlab ratio surf')
    print(d_C)
    print(d_T)

    NORM_VAR=np.var(LAYUP_J, ddof=1)
    APA_VAR=np.var([APA,-APA,APA,-APA,-APA,APA,-APA,APA],ddof=1)
    RATIO_RATIO=NORM_VAR/APA_VAR
    d_C_adjust= 1.7423*RATIO_RATIO**2 - 5.2496*RATIO_RATIO + 4.5289
    if d_C_adjust > 1.0:
        d_C_adjust = 1.0
    d_T_adjust= -0.4163*RATIO_RATIO**2 + 1.2127*RATIO_RATIO + 0.2378
    if d_T_adjust < 1.0:
        d_T_adjust = 1.0
    print('d_C and d_T: adjustment coefficients')
    print(d_C_adjust)
    print(d_T_adjust)

    ratio_C_temp=ratio_C_temp*d_C_adjust
    if ratio_C_temp > 0.99:
        ratio_C_temp = 0.99
    ratio_T_temp=ratio_T_temp*d_T_adjust
    if ratio_T_temp > 0.99:
        ratio_T_temp = 0.99
    print('C and T ratios (respectively):')
    print(ratio_C_temp)
    print(ratio_T_temp)
    KSI_C=math.sqrt((math.sqrt(-ratio_C_temp*(23*ratio_C_temp-24))/ratio_C_temp)-1)/math.sqrt(6)
    KSI_T=math.sqrt((math.sqrt(-ratio_T_temp*(23*ratio_T_temp-24))/ratio_T_temp)-1)/math.sqrt(6)
    d_C=(DIAMETER/2)*((1/KSI_C)-1)
    d_T=(DIAMETER/2)*((1/KSI_T)-1)
    if d_C < 0.0:
        d_C = 0.0
    if d_T < 0.0:
        d_T = 0.0
    print('final d_C and d_T')
    print(d_C)
    print(d_T)
    
    implemented_dist=[]
    char_coord22=[]
    nodes=[]
    global node_labels1
    node_labels1=[]
    node_set=[]

    for i in range(0,11):
        implemented_dist.append(D/2 +d_T +(d_C -d_T)*math.cos(math.radians(i*9)))
        char_coord22.append([implemented_dist[i]*math.sin(math.radians(i*9)),implemented_dist[i]*math.cos(math.radians(i*9))])
        nodes.append(mdb.models['Model-1'].parts['Composite'].nodes.getClosest(coordinates=(((char_coord22[i][0],char_coord22[i][1],0)),)))
        node_labels1.append(nodes[i][0].label)
        str1=str(i*9)
    print(node_labels1)
    node_set=mdb.models['Model-1'].parts['Composite'].SetFromNodeLabels('Char_curve', (node_labels1))

def Failure_Analysis():
    import math
    import __main__
    import sys
    import subprocess

    odb = openOdb(path="C:\Users\s1232851\Desktop\Mesh_convergence\Reference\Chang_and_Scott_Basic\AB-8mm\Job-1.odb")
    char_curve=odb.rootAssembly.instances['COMPOSITE-1'].nodeSets['CHAR_CURVE']
    reference_point=odb.rootAssembly.instances['ASSEMBLY'].nodes[0]
    NOF=len(odb.steps['Step-1'].frames)
    print(NOF)
    TT_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['TSAIH'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)

    # ============================
    A=np.zeros((NOF, TT_length))
    frame_TSAIH=[]
    for i in range(NOF):
        frame_TSAIH.append(odb.steps['Step-1'].frames[i].fieldOutputs['TSAIH'].getSubset(region=char_curve, position=ELEMENT_NODAL))

    max_TSAIH=[]
    for i in range(NOF):
        for j in range(0,TT_length):
            A[i][j]=frame_TSAIH[i].values[j].data
        max_TSAIH.append(max(A[i]))

    for i in range(NOF):
        if max_TSAIH[i]>1:
            print('===============TSAI-HILL ANALYSIS===============')
            print('INCREMENT NUMBER:')
            print(i)
            print('LOAD AT FAILURE (N):')
            laf=odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
            print(laf)
            print('FAILURE INDEX:')
            print(max_TSAIH[i])
            print('================================================')
            break

    # ============================
    TT1_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['TSAIW'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
    B=np.zeros((NOF, TT1_length))
    frame_TSAIW=[]
    for i in range(NOF):
        frame_TSAIW.append(odb.steps['Step-1'].frames[i].fieldOutputs['TSAIW'].getSubset(region=char_curve, position=ELEMENT_NODAL))
    # print(frame_TSAIW[1].values[1].data)

    max_TSAIW=[]
    for i in range(NOF):
        for j in range(0,TT1_length):
            B[i][j]=frame_TSAIW[i].values[j].data
        max_TSAIW.append(max(B[i]))
    # print(max_TSAIW)

    for i in range(NOF):
        if max_TSAIW[i]>1:
            print('===============TSAI-WU ANALYSIS===============')
            print('INCREMENT NUMBER:')
            print(i)
            print('LOAD AT FAILURE (N):')
            laf=odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
            print(laf)
            print('FAILURE INDEX:')
            print(max_TSAIW[i])
            print('================================================')
            break

    # ============================
    TT2_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['MSTRS'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
    C=np.zeros((NOF, TT2_length))
    frame_MSTRS=[]
    for i in range(NOF):
        frame_MSTRS.append(odb.steps['Step-1'].frames[i].fieldOutputs['MSTRS'].getSubset(region=char_curve, position=ELEMENT_NODAL))

    max_MSTRS=[]
    for i in range(NOF):
        for j in range(0,TT2_length):
            C[i][j]=frame_MSTRS[i].values[j].data
        max_MSTRS.append(max(C[i]))

    for i in range(NOF):
        if max_MSTRS[i]>1:
            print('===============MAX STRESS ANALYSIS===============')
            print('INCREMENT NUMBER:')
            print(i)
            print('LOAD AT FAILURE (N):')
            laf=odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
            print(laf)
            print('FAILURE INDEX:')
            print(max_MSTRS[i])
            print('================================================')
            break

    # ============================
    TT3_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['AZZIT'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
    D=np.zeros((NOF, TT3_length))
    frame_AZZIT=[]
    for i in range(NOF):
        frame_AZZIT.append(odb.steps['Step-1'].frames[i].fieldOutputs['AZZIT'].getSubset(region=char_curve, position=ELEMENT_NODAL))

    max_AZZIT=[]
    for i in range(NOF):
        for j in range(0,TT3_length):
            D[i][j]=frame_AZZIT[i].values[j].data
        max_AZZIT.append(max(D[i]))

    for i in range(NOF):
        if max_AZZIT[i]>1:
            print('===============AZZI-TSAI-HILL ANALYSIS===============')
            print('INCREMENT NUMBER:')
            print(i)
            print('LOAD AT FAILURE (N):')
            laf=odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
            print(laf)
            print('FAILURE INDEX:')
            print(max_AZZIT[i])
            print('================================================')
            break
